#include "bench5_guard.h"
#include <math.h>
#include <string.h>
#include "control/control_layer_cfg.h"

/* 单次可视试动：5°、0.01 rad/s，理论约8.73秒，最多使能12秒。
 * 电流必须由现场确认，且不能大于0.5A；不以“限流很小”替代机械支撑。 */
#define STEP_RAD 0.0872664626f
#define SPEED_RAD_S 0.01f
#define WAIT_MS 250U
#define TOTAL_MS 15000U
#define ENABLED_MS 12000U
#define PROGRESS_MS 1000U
/* 与位置差分使用同一量化/微小漂移裕量（约0.023°）。
 * 仅用于使能前的保持目标读回，不能扩大试动位移或掩盖使能前位置漂移。 */
#define POSITION_EPS_RAD 0.0004f

int b5_active(const b5_guard_t *g)
{
    return g->state != B5_IDLE && g->state != B5_DONE && g->state != B5_FAULT;
}

static void phase(b5_guard_t *g, unsigned state, uint32_t now)
{
    g->state=state; g->phase_at=now; g->step=0; g->requested=0;
}

static void finish(b5_guard_t *g, int result, uint32_t now)
{
    if (result!=B5_OK && !g->first_error) {
        g->failure_state=g->state; g->first_error=result;
    }
    g->result=result;
    phase(g, B5_STOPPING, now);
    g->attempts=0;
}

int b5_start(b5_guard_t *g, const b5_profile_t *p, uint32_t now)
{
    if (b5_active(g) || g->used) return B5_BUSY;
    if (p->locked || (p->direction != 1 && p->direction != -1) ||
        !isfinite(p->current_a) || p->current_a <= 0 || p->current_a > 0.5f)
        return B5_LOCKED;
    memset(g,0,sizeof(*g));
    g->used=1; g->started=now;
    phase(g,B5_READ_POS,now);
    return B5_OK;
}

int b5_start_curl(b5_guard_t *g, const b5_profile_t *p,
                  const rehab_curl_config_t *config, uint32_t now)
{
    rehab_curl_planner_t check;
    if (b5_active(g) || g->used) return B5_BUSY;
    if (!config || p->locked || p->direction!=1 ||
        !isfinite(p->current_a) || p->current_a<=0 ||
        p->current_a>CONTROL_REHAB_CURL_LIMIT_CURRENT_A ||
        rehab_curl_planner_start(&check,config,config->top_target_pos_rad,0,now)!=REHAB_CURL_RESULT_OK)
        return B5_LOCKED;
    memset(g,0,sizeof(*g));
    g->legacy_curl=1; g->curl.config=*config;
    g->used=1; g->started=now;
    phase(g,B5_READ_POS,now);
    return B5_OK;
}

void b5_cancel(b5_guard_t *g, uint32_t now)
{
    /* 即使未运行也允许停止；STOP 不清故障，不隐式重启试动。 */
    finish(g,B5_CANCELLED,now);
}

static int read_value(b5_guard_t *g, const b5_io_t *io, uint16_t index,
                      float *value, uint32_t now)
{
    int r;
    if (!g->requested) {
        g->request_at=now;
        if (io->read(index)) return B5_IO;
        g->requested=1;
        return 0;
    }
    /* 调度延迟后不能因缓存里恰好有回复而接受已超时的数据。 */
    if ((uint32_t)(now-g->request_at)>=WAIT_MS) return B5_TIMEOUT;
    r=io->reply(index,g->request_at,value);
    if (r < 0) return B5_IO;
    if (r > 0) return isfinite(*value) ? 1 : B5_FEEDBACK;
    return 0;
}

static int checked_write(b5_guard_t *g, const b5_io_t *io, uint16_t index,
                          float value, float tolerance, uint32_t now)
{
    float actual=0;
    int r;
    if (!g->step) {
        g->last_index=index; g->expected=value;
        if (io->write(index,value)) return B5_IO;
        g->step=1;
        return 0;
    }
    r=read_value(g,io,index,&actual,now);
    if (r==1) g->actual=actual;
    if (r==1 && fabsf(actual-value)>tolerance) return B5_FEEDBACK;
    return r;
}

/* 实测双极性读数的中位数会产生偏置。使用五次有符号均值平滑估计，
 * 不提高0.03阈值；0.1单次硬上限、位置差分和累计路程分别独立检查。
 * 不能用均值抵消真实往复运动：200ms累计绝对路程也必须受限。 */
static int velocity_window(b5_guard_t *g, float v)
{
    float sum=0;
    unsigned i;
    if (fabsf(v)>g->peak_velocity) g->peak_velocity=fabsf(v);
    /* 旧弯举使用其原有0.35rad/s上限，不消费低速试动的0.03均值窗口。 */
    if (g->legacy_curl) {
        if (fabsf(v)>g->curl.config.max_feedback_velocity_rad_s) return -1;
        g->samples++;
        return g->samples>=5 ? 1:0;
    }
    if (fabsf(v)>=0.1f) return -1;
    g->velocities[g->samples%5]=v;
    g->samples++;
    if (g->samples<5) return 0;
    for(i=0;i<5;i++) sum+=g->velocities[i];
    g->velocity_mean=sum/5.0f;
    return fabsf(g->velocity_mean)>0.03f ? -1:1;
}

/* 复用旧规划器；主动反馈与历史参数查询共用同一到位、驻留、限位与退出规则。 */
static int curl_step(b5_guard_t *g,const b5_io_t *io,float velocity,uint32_t now)
{
    rehab_curl_output_t output;
    if (fabsf(velocity)>g->peak_velocity) g->peak_velocity=fabsf(velocity);
    rehab_curl_planner_step(&g->curl,g->position,velocity,
        (uint32_t)(now-g->last_pos_at)<WAIT_MS,RT_FALSE,now,&output);
    if (output.action==REHAB_CURL_ACTION_STOP_FAULT)
        return output.result==REHAB_CURL_RESULT_SEGMENT_TIMEOUT ? B5_TIMEOUT:B5_LIMIT;
    if (!g->repeat_curl && output.completed_repetitions>=1U) finish(g,B5_OK,now);
    else {
        if (g->repeat_curl && output.completed_repetitions!=g->completed_cycles) {
            g->completed_cycles=output.completed_repetitions;
            /* 正常完整往返才续期，卡住不能依靠主机心跳延长运动时限。 */
            g->started=now; g->enabled_at=now;
        }
        if (output.action==REHAB_CURL_ACTION_COMMAND_POSITION) {
            g->target=output.target_pos_rad; g->target_attempts++;
            if (io->write(0x7016,g->target)) return B5_IO;
        }
        phase(g,B5_MONITOR_POS,now);
    }
    return 0;
}

void b5_tick(b5_guard_t *g, const b5_profile_t *p, const b5_io_t *io, uint32_t now)
{
    int r=0;
    float v=0;
    if (!b5_active(g)) return;
    if (g->state != B5_STOPPING &&
        ((uint32_t)(now-g->started)>=(g->legacy_curl ? 45000U:TOTAL_MS) ||
         ((g->state==B5_VERIFY_ENABLE || g->state==B5_TARGET ||
           g->state==B5_MONITOR_POS || g->state==B5_MONITOR_VEL) &&
          (uint32_t)(now-g->enabled_at)>=(g->legacy_curl ? 42000U:ENABLED_MS))))
        finish(g,B5_TIMEOUT,now);
    /* 使能前再次取得的位置不能偏离初始基线；运动过程中读取位置和速度交替进行。
     * 这里是本地软件超时保护，不是硬件安全停机；CAN断线时仍须现场断动力。 */
    switch (g->state) {
    case B5_READ_POS:
        r=read_value(g,io,0x7019,&v,now);
        if (r==1) {
                    if (g->legacy_curl && g->relative_curl==1U && !g->samples) {
                        rehab_curl_config_t *c=&g->curl.config;
                        float travel=c->bottom_target_pos_rad-c->top_target_pos_rad;
                        if(v<CONTROL_MOTOR_P_MIN_RAD || v>CONTROL_MOTOR_P_MAX_RAD ||
                           !isfinite(travel) || travel<=0) {r=B5_LIMIT;break;}
                        float target=fminf(v+travel,CONTROL_MOTOR_P_MAX_RAD-0.05f);
                        if(target-v<=2*c->position_tolerance_rad) {r=B5_LIMIT;break;}
                        c->top_target_pos_rad=v;c->bottom_target_pos_rad=target;
                        c->hard_min_pos_rad=fmaxf(CONTROL_MOTOR_P_MIN_RAD,v-0.06f);
                        c->hard_max_pos_rad=fminf(CONTROL_MOTOR_P_MAX_RAD,target+0.06f);
                        g->relative_curl=2U; /* Never rebase on repeated cycles or feedback drift. */
                    }
                    if (g->legacy_curl && (v<g->curl.config.hard_min_pos_rad ||
                        v>g->curl.config.hard_max_pos_rad)) { r=B5_LIMIT; break; }
                    if (g->samples && fabsf(v-g->origin)>0.002f) { r=B5_LIMIT; break; }
                    if (!g->samples) { g->origin=v; g->target=v+p->direction*STEP_RAD; }
                    g->position=v;
                    phase(g,B5_READ_VEL,now); }
        break;
    case B5_READ_VEL:
        r=read_value(g,io,0x701b,&v,now);
        if (r==1) {
            int qualified=velocity_window(g,v);
            if (qualified<0) r=B5_LIMIT;
            else {
                if (qualified && g->legacy_curl) {
                    rehab_curl_config_t config=g->curl.config;
                    if (rehab_curl_planner_start_flexion_first(&g->curl,&config,g->position,v,now)!=REHAB_CURL_RESULT_OK) {
                        r=B5_LIMIT; break;
                    }
                    g->target=config.bottom_target_pos_rad;
                }
                phase(g,qualified ? B5_STOP_FIRST:B5_READ_POS,now);
            }
        }
        break;
    case B5_STOP_FIRST:
        if (io->stop()) r=B5_IO;
        else phase(g,B5_VERIFY_STOP,now);
        break;
    case B5_VERIFY_STOP:
    case B5_VERIFY_ENABLE:
        r=io->status(g->state==B5_VERIFY_STOP ? 0U:2U);
        if ((uint32_t)(now-g->phase_at)>=WAIT_MS) r=B5_TIMEOUT;
        if (r==1) phase(g,g->state==B5_VERIFY_STOP ? B5_MODE:B5_TARGET,now);
        break;
    case B5_MODE:
        r=checked_write(g,io,0x7005,5,0,now);
        if (r==1) phase(g,B5_CURRENT,now);
        break;
    case B5_CURRENT:
        r=checked_write(g,io,0x7018,p->current_a,0.001f,now);
        if (r==1) phase(g,B5_SPEED,now);
        break;
    case B5_SPEED:
        r=checked_write(g,io,0x7017,g->legacy_curl ? CONTROL_REHAB_CURL_COMMAND_SPEED_RAD_S:SPEED_RAD_S,0.0001f,now);
        if (r==1) phase(g,B5_HOLD,now);
        break;
    case B5_HOLD:
        r=checked_write(g,io,0x7016,g->origin,POSITION_EPS_RAD,now);
        if (r==1) phase(g,B5_ENABLE,now);
        break;
    case B5_ENABLE:
        r=read_value(g,io,0x7019,&v,now);
        if (r==1) {
            if (fabsf(v-g->origin)>0.002f) r=B5_LIMIT;
            else {
                g->enable_attempts++;
                if (io->enable()) { r=B5_IO; break; }
                g->enabled_at=now; g->last_pos_at=now;
                   g->previous_at=now; g->previous_position=v;
                   g->travel_at=g->progress_at=now;
                   g->progress_position=v; g->travel=0;
                   g->samples=0; phase(g,B5_VERIFY_ENABLE,now); }
        }
        break;
    case B5_TARGET:
        /* 第一个目标也由旧规划器给出；使能前耗时不计入其运动段时限。 */
        if (g->legacy_curl) {
            g->curl.phase_started_ms=now;
            g->curl.last_command_ms=now;
        }
        g->target_attempts++;
        if (io->write(0x7016,g->target)) r=B5_IO;
        else phase(g,B5_MONITOR_POS,now);
        break;
    case B5_MONITOR_POS:
        if (g->legacy_curl && io->feedback) {
            float position,velocity;
            /* 运动中沿用原control_get_motor_feedback主动上报路径，不再逐次查询
             * 0x7019/0x701b。重复缓存不能累计到位样本；250ms无新样本仍退出。 */
            r=io->feedback(g->last_pos_at,&position,&velocity);
            if ((uint32_t)(now-g->last_pos_at)>=WAIT_MS) r=B5_TIMEOUT;
            if (r==1) {
                if (!isfinite(position) || !isfinite(velocity)) { r=B5_FEEDBACK; break; }
                g->position=position; g->last_pos_at=now;
                r=curl_step(g,io,velocity,now);
            }
            break;
        }
        r=read_value(g,io,0x7019,&v,now);
        if (r==1) {
            if (g->legacy_curl) {
                g->position=v; g->last_pos_at=now;
                if (io->status(2U)!=1) { r=B5_FEEDBACK; break; }
                if (v<g->curl.config.hard_min_pos_rad || v>g->curl.config.hard_max_pos_rad) {
                    r=B5_LIMIT; break;
                }
                phase(g,B5_MONITOR_VEL,now); break;
            }
            float displacement=(v-g->origin)*p->direction;
            uint32_t dt=now-g->previous_at;
            /* 量化裕量0.0004rad；与协议速度估计独立，防止噪声滤波掩盖真实位移。 */
            float allowed=POSITION_EPS_RAD+0.03f*(float)dt/1000.0f;
            if (!dt || fabsf(v-g->previous_position)>allowed) { r=B5_LIMIT; break; }
            g->travel+=fabsf(v-g->previous_position);
            if ((uint32_t)(now-g->travel_at)>=200U) {
                float budget=POSITION_EPS_RAD+0.03f*(float)(uint32_t)(now-g->travel_at)/1000.0f;
                if (g->travel>budget) { r=B5_LIMIT; break; }
                g->travel=0; g->travel_at=now;
            }
            g->previous_position=v; g->previous_at=now;
            g->position=v; g->last_pos_at=now;
            if (displacement < -0.002f || displacement > STEP_RAD+0.002f) r=B5_LIMIT;
            else if ((v-g->progress_position)*p->direction>=0.001f) {
                g->progress_position=v; g->progress_at=now;
                phase(g,B5_MONITOR_VEL,now);
            } else if ((uint32_t)(now-g->progress_at)>=PROGRESS_MS &&
                       fabsf(v-g->target)>=0.001f) r=B5_NO_PROGRESS;
            else phase(g,B5_MONITOR_VEL,now);
        }
        break;
    case B5_MONITOR_VEL:
        r=read_value(g,io,0x701b,&v,now);
        if (r==1) {
            if (g->legacy_curl) {
                if (io->status(2U)!=1) { r=B5_FEEDBACK; break; }
                r=curl_step(g,io,v,now);
                break;
            }
            int qualified=velocity_window(g,v);
            if (qualified<0) r=B5_LIMIT;
            else if (qualified && fabsf(g->position-g->target)<0.001f)
                finish(g,B5_OK,now);
            else phase(g,B5_MONITOR_POS,now);
        }
        break;
    case B5_STOPPING:
        /* 最多三次，不把发送成功宣称为物理停止验收。失败锁定，必须人工处理。 */
        if (!g->attempts || (uint32_t)(now-g->phase_at)>=50U) {
            g->stop_result=io->stop(); g->attempts++; g->phase_at=now;
            if (!g->stop_result) g->state=g->result==B5_OK ? B5_DONE : B5_FAULT;
            else if (g->attempts>=3) g->state=B5_FAULT;
        }
        break;
    default: finish(g,B5_FEEDBACK,now); break;
    }
    if (r<0) finish(g,r,now);
}
