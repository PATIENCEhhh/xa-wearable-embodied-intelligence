#include "bench_support_hold.h"
#include "bench5_guard.h"
#include "control/control_layer_cfg.h"
#include <math.h>
#include <string.h>

/* 采用原固定动作的1A限流量级，不承诺足以承载。保持失效就退出，不自动加流。
 * 以下偏移/速度都是电机协议原始单位；不是机械硬限位，也不是关节角。 */
#define BH_CURRENT_A CONTROL_REHAB_FIXED_ACTION_LIMIT_CURRENT_A
#define BH_SPEED_RAD_S CONTROL_REHAB_FIXED_ACTION_COMMAND_SPEED_RAD_S
#define BH_DRIFT_RAD 0.002f
#define BH_POSITION_EPS_RAD 0.0004f
#define BH_STILL_SPEED_RAD_S 0.03f
#define BH_SAMPLE_PEAK_RAD_S 0.10f
#define BH_HOLD_ERROR_RAD 0.035f
#define BH_HOLD_SPEED_RAD_S CONTROL_REHAB_FIXED_ACTION_MAX_FEEDBACK_SPEED_RAD_S
#define BH_WAIT_MS 250U
#define BH_PREP_MS 10000U
#define BH_TOTAL_MS 55000U

static void enter(bh_guard_t *g,unsigned phase,uint32_t now)
{
    g->phase=phase; g->phase_at=now; g->requested=0; g->written=0;
}
static int fail(bh_guard_t *g,int result)
{
    if (g->phase!=BH_FAULT) {
        g->failure_phase=g->phase; g->failed_joint=g->joints[g->axis];
        g->result=result; g->phase=BH_FAULT;
    }
    return g->result;
}
void bh_start(bh_guard_t *g,uint32_t now)
{
    memset(g,0,sizeof(*g)); g->started=now;
    g->joints[0]=4U;g->joints[1]=6U;g->count=2;
    g->speed=BH_SPEED_RAD_S;g->current=BH_CURRENT_A;
    enter(g,BH_SAMPLE_POS,now);
}
void bh_start_axes(bh_guard_t *g,uint32_t now,unsigned first,unsigned second,
                   unsigned count,float speed,float current,unsigned moving)
{
    bh_start(g,now);
    if(first<4U || first>6U || count<1U || count>2U ||
       (count==2U && (second<4U || second>6U || second==first)) ||
       !isfinite(speed) || speed<=0 || speed>1.30f ||
       !isfinite(current) || current<=0 || current>2.0f || moving>1U) {
        fail(g,B5_LIMIT);return;
    }
    g->joints[0]=first;g->joints[1]=second;g->count=count;
    g->speed=speed;g->current=current;g->moving=moving;
}
static int read_value(bh_guard_t *g,const bh_io_t *io,uint16_t index,
                      float *value,uint32_t now)
{
    int r;
    if (!g->requested) {
        if (io->read(g->joints[g->axis],index)) return B5_IO;
        g->request_at=now; g->requested=1; return 0;
    }
    if ((uint32_t)(now-g->request_at)>=BH_WAIT_MS) return B5_TIMEOUT;
    r=io->reply(g->joints[g->axis],index,value);
    if (r<0) return B5_IO;
    if (r>0 && !isfinite(*value)) return B5_FEEDBACK;
    return r;
}
static int write_checked(bh_guard_t *g,const bh_io_t *io,uint16_t index,
                         float value,float tolerance,uint32_t now)
{
    float actual=0;
    int r;
    if (!g->written) {
        if (io->write(g->joints[g->axis],index,value)) return B5_IO;
        g->written=1; return 0;
    }
    r=read_value(g,io,index,&actual,now);
    if (r==1 && fabsf(actual-value)>tolerance) return B5_FEEDBACK;
    return r;
}
static int monitor(bh_guard_t *g,const bh_io_t *io,uint32_t now)
{
    unsigned i;
    float p=0,v=0;
    if(g->moving && g->phase==BH_READY) return 1; /* 运动期由原轨迹反馈检查负责。 */
    for (i=0;i<g->count;i++) if (g->enabled_mask&(1U<<i)) {
        int r=io->feedback(g->joints[i],2U,g->enabled_at[i],&p,&v);
        if (r<0 || (!r && (uint32_t)(now-g->enabled_at[i])>=BH_WAIT_MS) ||
            (r>0 && (!isfinite(p) || !isfinite(v) ||
             fabsf(p-g->origin[i])>BH_HOLD_ERROR_RAD || fabsf(v)>BH_HOLD_SPEED_RAD_S))) {
            /* 先保存触发时的有效样本和准入掩码，再走原有整组退出路径。
             * r<=0时p/v不是有效反馈；上层必须显示为空，不能显示成实测零。 */
            g->trip.reason=r<0 ? BH_TRIP_FEEDBACK:!r ? BH_TRIP_STALE:
                (!isfinite(p) || !isfinite(v) ? BH_TRIP_NONFINITE:0U) |
                (isfinite(p) && fabsf(p-g->origin[i])>BH_HOLD_ERROR_RAD ? BH_TRIP_POSITION:0U) |
                (isfinite(v) && fabsf(v)>BH_HOLD_SPEED_RAD_S ? BH_TRIP_SPEED:0U);
            g->trip.enabled_mask=g->enabled_mask;
            g->trip.position=p; g->trip.velocity=v; g->trip.origin=g->origin[i];
            g->axis=i;
            return fail(g,r<=0 ? B5_FEEDBACK:B5_LIMIT);
        }
        /* 两轴刚完成使能但尚未收到新报告时，不允许启动5号。 */
        if (!r && g->phase==BH_READY) return 0;
    }
    return 1;
}
int bh_tick(bh_guard_t *g,const bh_io_t *io,uint32_t now)
{
    float value=0,velocity=0;
    unsigned joint;
    int r=0,monitored;
    if (g->phase==BH_IDLE) return B5_LOCKED;
    if (g->phase==BH_FAULT) return g->result;
    if ((uint32_t)(now-g->started)>=BH_TOTAL_MS ||
        (g->phase!=BH_READY && (uint32_t)(now-g->started)>=BH_PREP_MS))
        return fail(g,B5_TIMEOUT);
    monitored=monitor(g,io,now);
    if (monitored<0) return monitored;
    joint=g->joints[g->axis];
    switch(g->phase) {
    case BH_SAMPLE_POS:
        r=read_value(g,io,0x7019,&value,now);
        if (r==1) {
            if (value < -12.57f || value > 12.57f ||
                (g->samples && fabsf(value-g->origin[g->axis])>BH_DRIFT_RAD))
                return fail(g,B5_LIMIT);
            if (!g->samples) {
                g->origin[g->axis]=value; g->sample_started=now;
                g->sample_travel=0; g->velocity_sum=0; g->velocity_peak=0;
            } else {
                float distance=fabsf(value-g->sample_position);
                float interval=(float)(uint32_t)(now-g->sample_at)/1000.0f;
                float duration=(float)(uint32_t)(now-g->sample_started)/1000.0f;
                g->sample_travel+=distance;
                /* 速度均值不能掩盖真实移动：位置差分和绝对累计路程各自限幅。
                 * 量化裕量0.0004rad与原单轴准备检查一致，不能当作目标偏移。 */
                if (distance>BH_POSITION_EPS_RAD+BH_STILL_SPEED_RAD_S*interval ||
                    g->sample_travel>BH_POSITION_EPS_RAD+BH_STILL_SPEED_RAD_S*duration)
                    return fail(g,B5_LIMIT);
            }
            g->sample_position=value; g->sample_at=now;
            enter(g,BH_SAMPLE_VEL,now);
        }
        break;
    case BH_SAMPLE_VEL:
        r=read_value(g,io,0x701b,&value,now);
        if (r==1) {
            /* 实测4号静止时0x701B存在正负波动；不再把单次0.03判作移动。
             * 五次有符号均值仍限制0.03，单次硬上限0.10，且同时核对实际位移。 */
            if (fabsf(value)>=BH_SAMPLE_PEAK_RAD_S) return fail(g,B5_LIMIT);
            g->velocity_sum+=value;
            if (fabsf(value)>g->velocity_peak) g->velocity_peak=fabsf(value);
            if (g->samples==4U && fabsf(g->velocity_sum/5.0f)>BH_STILL_SPEED_RAD_S)
                return fail(g,B5_LIMIT);
            if (++g->samples<5) enter(g,BH_SAMPLE_POS,now);
            else if (++g->axis<g->count) { g->samples=0; enter(g,BH_SAMPLE_POS,now); }
            else { g->axis=0; enter(g,BH_STOP,now); }
        }
        break;
    case BH_STOP:
        if (io->stop(joint)) r=B5_IO;
        else enter(g,BH_STOPPED,now);
        break;
    case BH_STOPPED:
        r=io->feedback(joint,0U,g->phase_at,&value,&velocity);
        if ((uint32_t)(now-g->phase_at)>=BH_WAIT_MS) r=B5_TIMEOUT;
        if (r==1) enter(g,BH_MODE,now);
        break;
    case BH_MODE:
        r=write_checked(g,io,0x7005,5,0,now);
        if (r==1) enter(g,BH_CURRENT,now);
        break;
    case BH_CURRENT:
        r=write_checked(g,io,0x7018,g->current,0.001f,now);
        if (r==1) enter(g,BH_SPEED,now);
        break;
    case BH_SPEED:
        r=write_checked(g,io,0x7017,g->speed,0.0001f,now);
        if (r==1) enter(g,BH_TARGET,now);
        break;
    case BH_TARGET:
        r=write_checked(g,io,0x7016,g->origin[g->axis],0.0004f,now);
        if (r==1) {
            if (++g->axis<g->count) enter(g,BH_STOP,now);
            else { g->axis=0; enter(g,BH_RECHECK,now); }
        }
        break;
    case BH_RECHECK:
        r=read_value(g,io,0x7019,&value,now);
        if (r==1) {
            if (fabsf(value-g->origin[g->axis])>BH_DRIFT_RAD) return fail(g,B5_LIMIT);
            if (++g->axis<g->count) enter(g,BH_RECHECK,now);
            else { g->axis=0; enter(g,BH_ENABLE,now); }
        }
        break;
    case BH_ENABLE:
        /* 使能前最后一刻再核对位置，避免准备另一轴期间机构被手动移位。 */
        r=read_value(g,io,0x7019,&value,now);
        if (r==1) {
            if (fabsf(value-g->origin[g->axis])>BH_DRIFT_RAD) return fail(g,B5_LIMIT);
            g->enabled_at[g->axis]=now;
            g->enabled_mask|=1U<<g->axis; /* 发送失败也视为可能已使能，必须整组STOP。 */
            if (io->enable(joint)) r=B5_IO;
            else enter(g,BH_ENABLED,now);
        }
        break;
    case BH_ENABLED:
        r=io->feedback(joint,2U,g->enabled_at[g->axis],&value,&velocity);
        if ((uint32_t)(now-g->phase_at)>=BH_WAIT_MS) r=B5_TIMEOUT;
        if (r==1) {
            if (++g->axis<g->count) enter(g,BH_ENABLE,now);
            else { g->axis=0; enter(g,BH_READY,now); }
        }
        break;
    case BH_READY: return monitored;
    default: return fail(g,B5_FEEDBACK);
    }
    return r<0 ? fail(g,r):0;
}
