#include "m33_bench5.h"
#include "bench5_guard.h"
#include "bench_support_hold.h"
#include "bench_resist.h"
#include <math.h>
#include "control/control_layer.h"
#include "control/control_layer_cfg.h"
#include "ican_fixed_actions.h"
#include "control/rehab_arm_lift.h"
#include "control/rehab_trajectory_bank.h"
#include "../../shared/ican_curl_ui.h"

/* 三轴演示：4/6保持本轮位置，5复用旧弯举规划器先屈曲后伸展。
 * 网络授权一次一轮；屏幕明确启动后连续循环，正常停止后可再次点击启动。
 * 保持轴未准备好就不启动肘部，上电不自动使能，故障必须检查后复位。 */
/* 复用旧弯举的速度/限流和规划器，姿态范围使用本次两点实测。 */
static const b5_profile_t profile={0,1,CONTROL_REHAB_CURL_LIMIT_CURRENT_A};
static rehab_curl_config_t curl_config;
/* 仅由本机有线DAP在人工准备后写RAM；不提供网络解锁，不写Flash。 */
volatile rt_uint32_t g_bench5_arm;
#define ARM_KEY 0x42354131U
static b5_guard_t guard;
static struct rt_mutex lock;
static struct rt_mutex tx_lock;
static rt_thread_t owner;
static volatile unsigned cancelled;
static rt_bool_t ready;
static unsigned queued;
static rt_uint32_t read_tick;
static rt_uint32_t read_seq;
static rt_uint32_t state_tick;
static uint32_t armed_at;
static unsigned arm_seen;
static bh_guard_t support;
enum { GROUP_IDLE, GROUP_PREPARE, GROUP_CURL, GROUP_STOPPING, GROUP_FINISHED,
       GROUP_AXIS_PREPARE, GROUP_AXIS_RUN, GROUP_RECORD_PREPARE, GROUP_RECORD, GROUP_PLAY_PREPARE, GROUP_PLAY_RUN,
       GROUP_RESIST_PREPARE,GROUP_RESIST_RUN };
static br_guard_t resistance;
static unsigned group_phase, group_used;
static unsigned local_action=ICAN_UI_START_CURL, motion_joint, motion_started;
static bh_guard_t drive;
static rehab_arm_lift_t axis_motion;
static rehab_arm_lift_config_t axis_config;
static void axis_action_step(uint32_t now);
/* Slot zero keeps the original elbow/time bank; sidecar positions commit only with a full frame. */
static unsigned record_valid,record_requested,record_axis,play_index,play_count,play_arrivals,play_prepare_axis;
static uint32_t record_started,record_at,record_request_at,record_frame_at,play_at,play_duration,play_started;
static uint32_t record_sample_at[3],record_previous_at[3],play_feedback_at[3];
static float record_last[3],record_pending[3],record_sidecar[CONTROL_REHAB_TRAJECTORY_MAX_SAMPLES][2];
static float play_origin[3],play_first[3],play_from[3],play_to[3],play_min[3],play_max[3];
static bh_guard_t teach_prepare[3];
static const float teach_speed[3]={1.15f,0.12f,0.24f};
static const float teach_feedback_speed[3]={1.60f,0.35f,0.35f};
/* Passive teaching does not drive motors. Keep acquisition plausibility separate
 * from powered playback limits: allow up to 2rad/s joint motion, accounting for
 * joint4's external reduction. Playback still uses the accepted limits above. */
static const float record_capture_speed[3]={2.0f*CONTROL_MOTOR_JOINT4_GEAR_RATIO,
    2.0f*CONTROL_MOTOR_JOINT5_GEAR_RATIO,2.0f*CONTROL_MOTOR_JOINT6_GEAR_RATIO};
static int play_validate_axis(unsigned axis,float origin);
static void record_step(uint32_t now);
static void playback_step(uint32_t now);
__attribute__((aligned(32))) volatile ican_curl_ui_t g_ican_curl_ui = {
    .magic=ICAN_UI_MAGIC
};
static unsigned local_session, local_fault;
static uint32_t ui_seen_seq, ui_heartbeat, ui_heartbeat_at;
static unsigned stop_attempts[3], stop_done;
static int stop_results[3];
static uint32_t stop_at;

static uint32_t millis(void)
{
    /* 本机 RT_TICK_PER_SECOND=1000；显式换算用于主机测试和配置检查。 */
    return (uint32_t)(((rt_uint64_t)rt_tick_get()*1000U)/RT_TICK_PER_SECOND);
}
static int axis_read(unsigned joint,uint16_t index)
{
    control_motor_param_report_t p;
    if (control_get_last_motor_param(&p)!=RT_EOK) return -1;
    read_seq=p.response_seq;
    read_tick=rt_tick_get();
    return control_motor_read_parameter(joint,index);
}
static int axis_reply(unsigned joint,uint16_t index,float *value)
{
    control_motor_param_report_t p;
    if (control_get_last_motor_param(&p)!=RT_EOK) return -1;
    /* 写回显不能冒充读回；序号消除同一tick内真实回复被误判为旧缓存的问题。 */
    if (!p.valid || p.motor_id!=joint || p.index!=index || p.response_type!=0x11U ||
        p.response_seq==read_seq || (rt_int32_t)(p.timestamp-read_tick)<0) return 0;
    *value=index==0x7005 ? (float)p.raw_u8 : p.value_f32;
    return 1;
}
static int read_param(uint16_t index) { return axis_read(5,index); }
static int reply(uint16_t index,uint32_t after_ms,float *value)
{ RT_UNUSED(after_ms); return axis_reply(5,index,value); }
static int axis_write(unsigned joint,uint16_t index,float value)
{
    /* 保持轴使用采集到的原始位置直接写loc_ref，避免4号外部减速比/旧零偏叠加。 */
    return control_motor_write_parameter(joint,index,value,index==0x7005);
}
/* Moving axis baseline comes from five fresh parameter samples, without an old absolute gate.
 * Use the accepted direction/travel; clip only to protocol representability. */
static int axis_relative_config(unsigned joint,float origin)
{
    const float travel=joint==4U ? -11.029982281f:1.482063365f;
    if(!isfinite(origin) || origin<CONTROL_MOTOR_P_MIN_RAD || origin>CONTROL_MOTOR_P_MAX_RAD)
        return B5_LIMIT;
    float target=fminf(fmaxf(origin+travel,CONTROL_MOTOR_P_MIN_RAD+0.05f),
                       CONTROL_MOTOR_P_MAX_RAD-0.05f);
    if(fabsf(target-origin)<=0.06f || (target-origin)*travel<=0) return B5_LIMIT;
    axis_config.raised_raw=target;
    axis_config.min_raw=fmaxf(CONTROL_MOTOR_P_MIN_RAD,fminf(target,origin)-0.05f);
    axis_config.max_raw=fminf(CONTROL_MOTOR_P_MAX_RAD,fmaxf(target,origin)+0.05f);
    guard.origin=origin;guard.position=origin;guard.target=target;
    return 0;
}
static int axis_enable(unsigned joint)
{
    float ratio,zero,direction,position;
    if(local_action==ICAN_UI_START_LIFT || local_action==ICAN_UI_START_ROTATE || local_action==ICAN_UI_PLAY) {
        bh_guard_t *g=local_action==ICAN_UI_PLAY ? &teach_prepare[joint-4U]:joint==motion_joint ? &drive:&support;
        unsigned index=g->joints[0]==joint ? 0U:1U;
        if(index>=g->count || g->joints[index]!=joint) return -RT_EINVAL;
        if(local_action==ICAN_UI_PLAY || joint==motion_joint) {
            int r=local_action==ICAN_UI_PLAY ? play_validate_axis(joint-4U,g->origin[index]):
                axis_relative_config(joint,g->origin[index]);
            if(r) return r;
        }
        if(local_action!=ICAN_UI_PLAY && joint==motion_joint && (g->origin[index]<axis_config.min_raw ||
           g->origin[index]>axis_config.max_raw ||
           fabsf(g->origin[index]-axis_config.raised_raw)<=2*axis_config.tolerance))
            return B5_LIMIT;
        ratio=joint==4U ? CONTROL_MOTOR_JOINT4_GEAR_RATIO:joint==5U ?
            CONTROL_MOTOR_JOINT5_GEAR_RATIO:CONTROL_MOTOR_JOINT6_GEAR_RATIO;
        zero=joint==4U ? CONTROL_MOTOR_JOINT4_ZERO_OFFSET_RAD:joint==5U ?
            CONTROL_MOTOR_JOINT5_ZERO_OFFSET_RAD:CONTROL_MOTOR_JOINT6_ZERO_OFFSET_RAD;
        direction=joint==4U ? CONTROL_MOTOR_JOINT4_DIRECTION:joint==5U ?
            CONTROL_MOTOR_JOINT5_DIRECTION:CONTROL_MOTOR_JOINT6_DIRECTION;
        position=(g->origin[index]-zero)/ratio*direction;
        int r=control_motor_set_active_report(joint,RT_TRUE);
        return r ? r:control_motor_position_control_with_current_limit(joint,position,
            g->speed/ratio,g->current,RT_TRUE);
    }
    /* 复用旧CSP完整位置控制入口；先把本轮原始位置逆变换为它接受的关节坐标。
     * 旧入口会再做正变换，所以4号负方向/减速比、6号历史零偏不会变成新目标。
     * 两轴此前均已在停用态预置并读回同一原始目标与限流，避免旧使能顺序沿用旧目标。 */
    if (joint==4U) {
        ratio=CONTROL_MOTOR_JOINT4_GEAR_RATIO; zero=CONTROL_MOTOR_JOINT4_ZERO_OFFSET_RAD;
        direction=CONTROL_MOTOR_JOINT4_DIRECTION;
    } else if (joint==6U) {
        ratio=CONTROL_MOTOR_JOINT6_GEAR_RATIO; zero=CONTROL_MOTOR_JOINT6_ZERO_OFFSET_RAD;
        direction=CONTROL_MOTOR_JOINT6_DIRECTION;
    } else return -RT_EINVAL;
    if (!isfinite(ratio) || ratio<=0 || !isfinite(zero) || !isfinite(direction)) return -RT_EINVAL;
    direction=direction<0 ? -1.0f:1.0f;
    position=(support.origin[joint==4U ? 0:1]-zero)/ratio*direction;
    int r=control_motor_set_active_report(joint,RT_TRUE);
    return r ? r:control_motor_position_control_with_current_limit(joint,position,
        CONTROL_REHAB_FIXED_ACTION_COMMAND_SPEED_RAD_S/ratio,
        CONTROL_REHAB_FIXED_ACTION_LIMIT_CURRENT_A,RT_TRUE);
}
static int axis_stop(unsigned joint) { return control_motor_stop(joint,RT_FALSE); }
static int axis_feedback(unsigned joint,unsigned mode,uint32_t since,float *p,float *v)
{
    control_motor_feedback_t fb;
    uint32_t timestamp;
    if (control_get_motor_feedback(joint,&fb)!=RT_EOK) return 0;
    timestamp=(uint32_t)(((rt_uint64_t)fb.timestamp*1000U)/RT_TICK_PER_SECOND);
    if ((int32_t)(timestamp-since)<=0 || (uint32_t)(millis()-timestamp)>250U) return 0;
    if (fb.motor_id!=joint || fb.fault_summary || fb.mode_state!=mode) return B5_FEEDBACK;
    /* 私有协议ctrl_update_motor_feedback直接保存原始值，没有关节换算。
     * 必须与0x7019基线同单位，不能因control_layer.h的泛化注释再乘减速比。 */
    *p=fb.pos_rad; *v=fb.vel_rad_s;
    return isfinite(*p) && isfinite(*v) ? 1:B5_FEEDBACK;
}
static const bh_io_t hold_io={axis_read,axis_reply,axis_write,axis_enable,axis_stop,axis_feedback};
static int write_param(uint16_t index,float value)
{
    if (index==0x7016) {
        /* 旧CSP接口接受关节坐标，内部再转回电机坐标；原始目标不能直接传入。 */
        float joint=(value-CONTROL_MOTOR_JOINT5_ZERO_OFFSET_RAD)/
            CONTROL_MOTOR_JOINT5_GEAR_RATIO*CONTROL_MOTOR_JOINT5_DIRECTION;
        return control_motor_csp_setpoint(5,joint);
    }
    return control_motor_write_parameter(5,index,value,index==0x7005);
}
static int enable(void) {
    int r=control_motor_set_active_report(5,RT_TRUE);
    if (r) return r;
    state_tick=rt_tick_get(); return control_motor_enable(5);
}
static int stop(void) { state_tick=rt_tick_get(); return control_motor_stop(5,RT_FALSE); }
static int status(unsigned mode)
{
    control_motor_feedback_t fb;
    if (control_get_motor_feedback(5,&fb)!=RT_EOK) return 0;
    if ((rt_int32_t)(fb.timestamp-state_tick)<=0) return 0;
    if ((rt_uint32_t)(rt_tick_get()-fb.timestamp)>250U) return 0;
    if (fb.fault_summary || fb.mode_state!=mode) return B5_FEEDBACK;
    return 1;
}
static int curl_feedback(uint32_t after_ms,float *position,float *velocity)
{
    /* 原控制层已解码为电机原始单位，沿用4/6保持的时间戳与故障检查。 */
    return axis_feedback(5U,2U,after_ms,position,velocity);
}
static const b5_io_t io={read_param,reply,write_param,enable,stop,status,curl_feedback};

rt_err_t m33_bench5_tx_enter(rt_uint32_t id,rt_uint8_t ide)
{
    unsigned type=(id>>24)&31U, motor=id&255U;
    rt_bool_t mine=rt_thread_self()==owner;
    /* 三轴仍由同一线程独占：只开放4/5/6；4/6目标由保持状态机内部决定。
     * 旧ROS、本地模式、归零及网络任意参数写入仍不能启动电机。 */
    if (!ready) return -RT_EBUSY;
    rt_mutex_take(&tx_lock,RT_WAITING_FOREVER);
    if (ide==1U) {
        if (type==4U) {
            if (!mine) { cancelled=1; g_bench5_arm=0; }
        } else if (type==0U || type==0x11U) {
            if (!mine && (queued || (group_phase!=GROUP_IDLE && group_phase!=GROUP_FINISHED))) goto denied;
        } else if (!(mine && motor>=4U && motor<=6U && !cancelled &&
                     (type==3U || type==0x12U || type==0x18U))) goto denied;
    } else {
        /* 仅保留原工程对上层的状态帧；标准电机控制也不能绕过专用模式。 */
        if (id<0x320U || id>0x37fU) goto denied;
    }
    return RT_EOK;
denied:
    rt_mutex_release(&tx_lock);
    return -RT_EBUSY;
}
void m33_bench5_tx_leave(void) { rt_mutex_release(&tx_lock); }

rt_bool_t m33_bench5_busy(void)
{
    rt_bool_t busy;
    if (!ready) return RT_FALSE;
    rt_mutex_take(&lock,RT_WAITING_FOREVER);
    busy=queued || (group_phase!=GROUP_IDLE && group_phase!=GROUP_FINISHED);
    rt_mutex_release(&lock);
    return busy;
}
rt_err_t m33_bench5_command(unsigned op)
{
    int r=RT_EOK;
    rehab_curl_config_t selected;
    if (!ready) return -RT_ERROR;
    rt_mutex_take(&lock,RT_WAITING_FOREVER);
    if (op==5) { queued=5; g_bench5_arm=0; cancelled=1; } /* STOP锁存，不会再自动启动。 */
    else if (ican_fixed_action_config(op,&selected)) {
        if (profile.locked || g_bench5_arm!=ARM_KEY || cancelled) r=B5_LOCKED;
        else if (queued || group_phase!=GROUP_IDLE || group_used) r=B5_BUSY;
        else { local_action=ICAN_UI_START_CURL;local_session=0; curl_config=selected; g_bench5_arm=0; queued=6; }
    } else if (op!=7) r=-RT_EINVAL;
    rt_mutex_release(&lock);
    return r;
}
void m33_bench5_snapshot(rt_uint32_t *state,rt_uint32_t *flags,
                         rt_int32_t *result,rt_int32_t *stop_result)
{
    if (!ready) { *state=0; *flags=1; *result=-RT_ERROR; *stop_result=-RT_ERROR; return; }
    rt_mutex_take(&lock,RT_WAITING_FOREVER);
    *state=group_phase==GROUP_PREPARE ? B5_READ_POS:guard.state;
    *flags=(profile.locked || g_bench5_arm!=ARM_KEY || cancelled ? 1U:0U) | (group_used ? 2U:0U) |
           ((group_phase!=GROUP_IDLE && group_phase!=GROUP_FINISHED) || queued ? 4U:0U);
    *result=guard.result; *stop_result=guard.stop_result;
    rt_mutex_release(&lock);
}
void m33_bench5_detail(float *origin,float *position,float *target,rt_uint32_t *attempts)
{
    rt_mutex_take(&lock,RT_WAITING_FOREVER);
    *origin=guard.origin; *position=guard.position; *target=guard.target; *attempts=guard.attempts;
    rt_mutex_release(&lock);
}
void m33_bench5_support(float *origin4,float *origin6,rt_uint32_t *phase,
                         rt_uint32_t *failed_joint,rt_uint32_t *failed_phase)
{
    rt_mutex_take(&lock,RT_WAITING_FOREVER);
    if(local_action==ICAN_UI_START_LIFT || local_action==ICAN_UI_START_ROTATE || local_action==ICAN_UI_PLAY) {
        *origin4=NAN;*origin6=NAN;
        for(unsigned i=0;i<support.count;i++) {
            if(support.joints[i]==4U) *origin4=support.origin[i];
            if(support.joints[i]==6U) *origin6=support.origin[i];
        }
        if(drive.samples) {
            if(motion_joint==4U) *origin4=drive.origin[0];
            if(motion_joint==6U) *origin6=drive.origin[0];
        }
    } else { *origin4=support.origin[0]; *origin6=support.origin[1]; }
    if((local_action==ICAN_UI_START_LIFT || local_action==ICAN_UI_START_ROTATE || local_action==ICAN_UI_PLAY) && drive.phase==BH_FAULT) {
        *phase=drive.phase;*failed_joint=drive.failed_joint;*failed_phase=drive.failure_phase;
    } else {
        *phase=support.phase; *failed_joint=support.failed_joint; *failed_phase=support.failure_phase;
    }
    rt_mutex_release(&lock);
}
void m33_bench5_failure(volatile rt_uint32_t *state,volatile rt_uint32_t *error,
    volatile rt_uint32_t *index,volatile rt_uint32_t *enables,volatile rt_uint32_t *targets)
{
    rt_mutex_take(&lock,RT_WAITING_FOREVER);
    *state=guard.failure_state; *error=(rt_uint32_t)guard.first_error;
    *index=guard.last_index; *enables=guard.enable_attempts; *targets=guard.target_attempts;
    rt_mutex_release(&lock);
}
void m33_bench5_trip(float *position,float *velocity,float *origin,
                     rt_uint32_t *reason,rt_uint32_t *enabled_mask)
{
    rt_mutex_take(&lock,RT_WAITING_FOREVER);
    *position=support.trip.position; *velocity=support.trip.velocity;
    *origin=support.trip.origin; *reason=support.trip.reason;
    *enabled_mask=support.trip.enabled_mask;
    rt_mutex_release(&lock);
}
static void group_stop(int result,uint32_t now)
{
    if (group_phase==GROUP_STOPPING) return;
    if(local_action==ICAN_UI_RESIST) {
        resistance.current=0;rehab_resist_strategy_reset(&resistance.strategy);
    }
    if(group_phase==GROUP_RECORD || group_phase==GROUP_RECORD_PREPARE) {
        record_valid=group_phase==GROUP_RECORD && !local_fault &&
            (result==0 || result==B5_CANCELLED) && rehab_trajectory_bank_count(0)>=2U;
    }
    if (result && !guard.first_error) guard.first_error=result;
    if (local_session && result && result!=B5_CANCELLED) local_fault=1;
    guard.result=result; guard.state=B5_STOPPING;
    memset(stop_attempts,0,sizeof(stop_attempts));
    memset(stop_results,0,sizeof(stop_results));
    stop_done=0; stop_at=now-50U; group_phase=GROUP_STOPPING;
}
static void local_ui_poll(uint32_t now)
{
    uint32_t heartbeat=g_ican_curl_ui.heartbeat, seq=g_ican_curl_ui.seq;
    if (heartbeat!=ui_heartbeat) { ui_heartbeat=heartbeat; ui_heartbeat_at=now; }
    if (seq && seq!=ui_seen_seq) {
        unsigned command=g_ican_curl_ui.command;
        int result=0;
        ui_seen_seq=seq;
        if (command==2U) {
            /* 空闲时先点停止也属于本地会话，不能把下一次正常点击启动永久锁住。
             * 已排队的网络动作或已结束的网络故障不借此转成本地授权。 */
            if (group_phase==GROUP_IDLE && !queued && !group_used) local_session=1;
            queued=5;cancelled=1;g_bench5_arm=0;
        } else if (ican_ui_action_opcode(command) || command==ICAN_UI_START_LIFT ||
                   command==ICAN_UI_START_ROTATE || command==ICAN_UI_RECORD || command==ICAN_UI_PLAY ||
                   command==ICAN_UI_RESIST) {
            if (!heartbeat || (uint32_t)(now-ui_heartbeat_at)>=1500U || local_fault)
                result=B5_LOCKED;
            else if (queued || (group_phase!=GROUP_IDLE && group_phase!=GROUP_FINISHED))
                result=B5_BUSY;
            else if(command==ICAN_UI_PLAY && (!record_valid || rehab_trajectory_bank_count(0)<2U))
                result=ICAN_UI_NO_RECORD;
            else if (group_phase==GROUP_FINISHED && (!local_session || guard.stop_result))
                result=B5_LOCKED;
            else {
                /* 新的屏幕点击才可建新会话，不以内部改RAM方式重放网络单次机会。
                 * 故障不可重启；正常STOP且三轴停止发送成功后允许再次点击启动。 */
                memset(&guard,0,sizeof(guard));memset(&support,0,sizeof(support));
                /* 复用已实现的固定动作配置，只切换目标/驻留，不复制运动控制。 */
                ican_fixed_action_config(ican_ui_action_opcode(command),&curl_config);
                local_action=command;
                local_session=1;cancelled=0;group_used=0;group_phase=GROUP_IDLE;
                g_bench5_arm=0;queued=ican_ui_action_opcode(command) ? 6U:command==ICAN_UI_RECORD ? 30U:
                    command==ICAN_UI_RESIST ? 40U:20U;
            }
        } else result=-RT_EINVAL;
        g_ican_curl_ui.result=(uint32_t)result;
        g_ican_curl_ui.done=seq;
    }
    if (local_session && (queued==6 || queued==20 || queued==30 || queued==40 || group_phase==GROUP_PREPARE || group_phase==GROUP_CURL ||
        group_phase==GROUP_AXIS_PREPARE || group_phase==GROUP_AXIS_RUN ||
        group_phase==GROUP_RECORD_PREPARE || group_phase==GROUP_RECORD ||
        group_phase==GROUP_PLAY_PREPARE || group_phase==GROUP_PLAY_RUN ||
        group_phase==GROUP_RESIST_PREPARE || group_phase==GROUP_RESIST_RUN) &&
        (uint32_t)(now-ui_heartbeat_at)>=1500U) {
        local_fault=1;cancelled=1;queued=5;
    }
}
static void local_ui_snapshot(void)
{
    g_ican_curl_ui.state=guard.state;
    g_ican_curl_ui.active=queued || (group_phase!=GROUP_IDLE && group_phase!=GROUP_FINISHED);
    g_ican_curl_ui.cycles=guard.completed_cycles;
    unsigned phase=(local_action==ICAN_UI_START_LIFT || local_action==ICAN_UI_START_ROTATE) ? axis_motion.state:guard.curl.phase;
    if(local_action==ICAN_UI_RESIST) phase=resistance.phase;
    g_ican_curl_ui.phase=(phase&0xffffU) | ((uint32_t)((record_valid || group_phase==GROUP_RECORD) ? rehab_trajectory_bank_count(0):0U)<<16);
    g_ican_curl_ui.fault=local_fault ? (uint32_t)(guard.first_error ? guard.first_error:B5_FEEDBACK):0;
}
static int motion_target(unsigned joint,float raw)
{
    float ratio=joint==4 ? CONTROL_MOTOR_JOINT4_GEAR_RATIO:joint==5 ?
        CONTROL_MOTOR_JOINT5_GEAR_RATIO:CONTROL_MOTOR_JOINT6_GEAR_RATIO;
    float zero=joint==4 ? CONTROL_MOTOR_JOINT4_ZERO_OFFSET_RAD:joint==5 ?
        CONTROL_MOTOR_JOINT5_ZERO_OFFSET_RAD:CONTROL_MOTOR_JOINT6_ZERO_OFFSET_RAD;
    float direction=joint==4 ? CONTROL_MOTOR_JOINT4_DIRECTION:joint==5 ?
        CONTROL_MOTOR_JOINT5_DIRECTION:CONTROL_MOTOR_JOINT6_DIRECTION;
    return isfinite(raw) ? control_motor_csp_setpoint(joint,(raw-zero)/ratio*direction):B5_LIMIT;
}
static int motion_feedback(rehab_arm_lift_feedback_t *f,uint32_t now)
{
    unsigned physical[3]={motion_joint,support.joints[0],support.joints[1]};
    memset(f,0,sizeof(*f));
    for(unsigned i=0;i<3;i++) {
        control_motor_feedback_t fb;
        if(control_get_motor_feedback(physical[i],&fb)!=RT_EOK || fb.motor_id!=physical[i] ||
           fb.mode_state!=2U || fb.fault_summary || !isfinite(fb.pos_rad) || !isfinite(fb.vel_rad_s))
            return B5_FEEDBACK;
        uint32_t tick=(uint32_t)(((rt_uint64_t)fb.timestamp*1000U)/RT_TICK_PER_SECOND);
        if((uint32_t)(now-tick)>=250U) return B5_FEEDBACK;
        f->axis[i]=(rehab_arm_lift_axis_t){1U,0U,tick,fb.pos_rad,fb.vel_rad_s};
    }
    return 0;
}

/* Sequential CAN parameter transactions form one frame (maximum 100ms skew).
 * No enabled feedback is required for teaching, and no ENABLE/loc_ref is sent. */
static void record_step(uint32_t now)
{
    int r;
    float position=0,velocity=0;
    if(group_phase==GROUP_RECORD_PREPARE) {
        for(unsigned j=4;j<=6;j++) {
            r=axis_feedback(j,0U,record_started,&position,&velocity);
            if(r<0 || (!r && (uint32_t)(now-record_started)>=250U)) {group_stop(B5_FEEDBACK,now);return;}
            if(!r) return;
        }
        if(rehab_trajectory_bank_clear(0)) {group_stop(B5_IO,now);return;}
        record_valid=0;record_requested=record_axis=0;record_at=now;
        group_phase=GROUP_RECORD;guard.state=B5_MONITOR_POS;
    }
    for(unsigned j=4;j<=6;j++) {
        control_motor_feedback_t fb;
        if(control_get_motor_feedback(j,&fb)==RT_EOK && fb.motor_id==j &&
           (fb.fault_summary || fb.mode_state!=0U)) {group_stop(B5_FEEDBACK,now);return;}
    }
    if(!record_requested) {
        if(!record_axis && rehab_trajectory_bank_count(0) && (uint32_t)(now-record_at)<50U) return;
        if(!record_axis) record_frame_at=now;
        if(axis_read(4U+record_axis,0x7019)) {group_stop(B5_IO,now);return;}
        record_request_at=now;record_requested=1;return;
    }
    if((uint32_t)(now-record_request_at)>=250U || (uint32_t)(now-record_frame_at)>100U) {
        group_stop(B5_TIMEOUT,now);return;
    }
    r=axis_reply(4U+record_axis,0x7019,&position);
    if(r<0) {group_stop(B5_IO,now);return;}
    if(!r) return;
    record_requested=0;
    if(!isfinite(position) || position<CONTROL_MOTOR_P_MIN_RAD || position>CONTROL_MOTOR_P_MAX_RAD) {
        group_stop(B5_LIMIT,now);return;
    }
    record_pending[record_axis]=position;record_sample_at[record_axis]=now;
    if(++record_axis<3U) return;
    record_axis=0;
    unsigned count=rehab_trajectory_bank_count(0);
    uint32_t dt=count ? now-record_at:0U;
    if(count && (!dt || dt>1000U)) {group_stop(B5_LIMIT,now);return;}
    for(unsigned i=0;i<3;i++) if(count) {
        uint32_t axis_dt=record_sample_at[i]-record_previous_at[i];
        if(!axis_dt || axis_dt>1000U || fabsf(record_pending[i]-record_last[i])>
           0.0004f+record_capture_speed[i]*(float)axis_dt/1000.0f) {
            /* Preserve the triggering axis/sample before STOP; do not infer it
             * later from motor feedback after the operator has stopped moving. */
            guard.failure_state=guard.state;guard.last_index=0x7019U;
            support.failed_joint=i+4U;support.failure_phase=GROUP_RECORD;
            support.trip.position=record_pending[i];support.trip.origin=record_last[i];
            support.trip.velocity=axis_dt ? (record_pending[i]-record_last[i])*1000.0f/axis_dt:0.0f;
            support.trip.reason=(!axis_dt || axis_dt>1000U) ? BH_TRIP_STALE:BH_TRIP_SPEED;
            support.trip.enabled_mask=0;
            group_stop(B5_LIMIT,now);return;
        }
    }
    rehab_trajectory_sample_t sample={(rt_uint16_t)dt,record_pending[1],
        count ? (record_pending[1]-record_last[1])*1000.0f/dt:0.0f,0.0f};
    record_sidecar[count][0]=record_pending[0];record_sidecar[count][1]=record_pending[2];
    if(rehab_trajectory_bank_append(0,&sample)) {group_stop(B5_IO,now);return;}
    memcpy(record_last,record_pending,sizeof(record_last));
    memcpy(record_previous_at,record_sample_at,sizeof(record_previous_at));
    record_at=now;guard.position=sample.pos_rad;
    if(!count) guard.origin=sample.pos_rad;
    if(rehab_trajectory_bank_count(0)>=CONTROL_REHAB_TRAJECTORY_MAX_SAMPLES) group_stop(0,now);
}
static int teach_frame(unsigned index,float position[3],uint32_t *dt)
{
    rehab_trajectory_sample_t sample;
    if(index>=CONTROL_REHAB_TRAJECTORY_MAX_SAMPLES || rehab_trajectory_bank_get(0,index,&sample)) return B5_IO;
    position[0]=record_sidecar[index][0];position[1]=sample.pos_rad;position[2]=record_sidecar[index][1];
    *dt=sample.dt_ms;
    if(index && (!*dt || *dt>1000U)) return B5_LIMIT;
    for(unsigned i=0;i<3;i++) if(!isfinite(position[i]) || position[i]<CONTROL_MOTOR_P_MIN_RAD ||
       position[i]>CONTROL_MOTOR_P_MAX_RAD) return B5_LIMIT;
    return 0;
}
static int play_validate_axis(unsigned axis,float origin)
{
    float first[3],position[3];uint32_t dt;
    play_count=rehab_trajectory_bank_count(0);
    if(!record_valid || play_count<2U) return ICAN_UI_NO_RECORD;
    if(teach_frame(0,first,&dt) || !isfinite(origin)) return B5_LIMIT;
    play_origin[axis]=origin;play_first[axis]=first[axis];play_min[axis]=play_max[axis]=origin;
    for(unsigned i=0;i<play_count;i++) {
        if(teach_frame(i,position,&dt)) return B5_LIMIT;
        float target=origin+position[axis]-first[axis];
        if(!isfinite(target) || target<CONTROL_MOTOR_P_MIN_RAD || target>CONTROL_MOTOR_P_MAX_RAD) return B5_LIMIT;
        play_min[axis]=fminf(play_min[axis],target);play_max[axis]=fmaxf(play_max[axis],target);
    }
    play_min[axis]=fmaxf(CONTROL_MOTOR_P_MIN_RAD,play_min[axis]-0.06f);
    play_max[axis]=fminf(CONTROL_MOTOR_P_MAX_RAD,play_max[axis]+0.06f);
    return 0;
}
static int play_segment(unsigned index,uint32_t now)
{
    float position[3];uint32_t dt;
    int r=teach_frame(index,position,&dt);if(r) return r;
    play_duration=dt>50U ? dt:50U;
    for(unsigned i=0;i<3;i++) {
        play_from[i]=play_to[i];play_to[i]=play_origin[i]+position[i]-play_first[i];
        uint32_t limited=(uint32_t)(fabsf(play_to[i]-play_from[i])*1000.0f/teach_speed[i])+1U;
        if(limited>play_duration) play_duration=limited;
    }
    play_at=now;play_arrivals=0;
    return 0;
}
static int teach_feedback(unsigned axis,uint32_t now,float *position,float *velocity,uint32_t *timestamp)
{
    control_motor_feedback_t fb;
    if(control_get_motor_feedback(axis+4U,&fb)!=RT_EOK || fb.motor_id!=axis+4U || fb.mode_state!=2U ||
       fb.fault_summary || !isfinite(fb.pos_rad) || !isfinite(fb.vel_rad_s)) return B5_FEEDBACK;
    *timestamp=(uint32_t)(((rt_uint64_t)fb.timestamp*1000U)/RT_TICK_PER_SECOND);
    if((uint32_t)(now-*timestamp)>=250U) return B5_FEEDBACK;
    *position=fb.pos_rad;*velocity=fb.vel_rad_s;return 0;
}
static void playback_step(uint32_t now)
{
    int r;float position[3],velocity[3];uint32_t timestamp[3];
    if(group_phase==GROUP_PLAY_PREPARE) {
        for(unsigned i=0;i<play_prepare_axis;i++) {
            r=teach_feedback(i,now,&position[i],&velocity[i],&timestamp[i]);
            if(r || fabsf(position[i]-play_origin[i])>0.035f || fabsf(velocity[i])>teach_feedback_speed[i]) {
                group_stop(r ? r:B5_LIMIT,now);return;
            }
        }
        r=bh_tick(&teach_prepare[play_prepare_axis],&hold_io,now);
        if(r<0) {group_stop(r,now);return;}
        if(!r) return;
        if(++play_prepare_axis<3U) {
            unsigned i=play_prepare_axis;
            bh_start_axes(&teach_prepare[i],now,i+4U,0,1,teach_speed[i],i==1U ? 2.0f:1.0f,1);
            return;
        }
        for(unsigned i=0;i<3;i++) {
            r=teach_feedback(i,now,&position[i],&velocity[i],&timestamp[i]);
            if(r || fabsf(position[i]-play_origin[i])>0.035f) {group_stop(r ? r:B5_LIMIT,now);return;}
            play_to[i]=play_origin[i];play_feedback_at[i]=timestamp[i];
        }
        play_index=1;play_started=now;
        r=play_segment(play_index,now);if(r) {group_stop(r,now);return;}
        group_phase=GROUP_PLAY_RUN;guard.state=B5_TARGET;
    }
    if((uint32_t)(now-play_started)>=180000U) {group_stop(B5_TIMEOUT,now);return;}
    uint32_t elapsed=now-play_at;
    if(elapsed>=play_duration+2000U) {group_stop(B5_TIMEOUT,now);return;}
    float fraction=elapsed>=play_duration ? 1.0f:(float)elapsed/play_duration;
    unsigned arrived=elapsed>=play_duration,fresh=1;
    for(unsigned i=0;i<3;i++) {
        r=teach_feedback(i,now,&position[i],&velocity[i],&timestamp[i]);
        if(r) {group_stop(r,now);return;}
        float target=play_from[i]+(play_to[i]-play_from[i])*fraction;
        if(position[i]<play_min[i] || position[i]>play_max[i] || fabsf(velocity[i])>teach_feedback_speed[i] ||
           fabsf(position[i]-target)>(i==0U ? 0.30f:0.12f)) {group_stop(B5_LIMIT,now);return;}
        if(fabsf(position[i]-play_to[i])>0.03f) arrived=0;
        if(timestamp[i]==play_feedback_at[i]) fresh=0;
    }
    if(fresh) {
        memcpy(play_feedback_at,timestamp,sizeof(play_feedback_at));
        play_arrivals=arrived ? play_arrivals+1U:0U;
    }
    if(play_arrivals>=(play_index+1U==play_count ? 5U:1U)) {
        if(++play_index>=play_count) {guard.completed_cycles=1;group_stop(0,now);return;}
        r=play_segment(play_index,now);if(r) {group_stop(r,now);return;}
        fraction=0;
    }
    for(unsigned i=0;i<3;i++) {
        float target=play_from[i]+(play_to[i]-play_from[i])*fraction;
        if(i==1U) {guard.position=position[i];guard.target=target;}
        if(motion_target(i+4U,target)) {group_stop(B5_IO,now);return;}
    }
}

static void axis_action_step(uint32_t now)
{
    int r;
    r=bh_tick(&support,&hold_io,now);
    if(r<0) {group_stop(r,now);return;}
    if(r==0) return;
    if(!motion_started) {
        float speed=motion_joint==4 ? 1.30f:motion_joint==6 ? 0.28f:0.12f;
        bh_start_axes(&drive,now,motion_joint,0,1,speed,motion_joint==5 ? 2.0f:1.0f,1);
        motion_started=1;
    }
    if(group_phase==GROUP_AXIS_PREPARE) {
        r=bh_tick(&drive,&hold_io,now);
        if(r<0) {group_stop(r,now);return;}
        if(r==0) return;
        rehab_arm_lift_feedback_t f;
        if(motion_feedback(&f,now)) {group_stop(B5_FEEDBACK,now);return;}
        r=rehab_arm_lift_start(&axis_motion,&axis_config,&f,now);
        guard.origin=f.axis[0].position;
        if(r) {group_stop(r,now);return;}
        group_phase=GROUP_AXIS_RUN;guard.state=B5_TARGET;
    }
    rehab_arm_lift_feedback_t f;
    if(motion_feedback(&f,now)) {group_stop(B5_FEEDBACK,now);return;}
    guard.position=f.axis[0].position;
    float moving_target;
    {
        rehab_arm_lift_output_t out;
        unsigned previous=axis_motion.cycles;
        rehab_arm_lift_step(&axis_motion,&f,now,0,&out);
        guard.completed_cycles=out.cycles;
        if(axis_motion.cycles!=previous) support.started=now;
        if(out.stop_all) {group_stop(out.result,now);return;}
        if(!out.write_targets) return;
        moving_target=out.target_raw[0];guard.target=moving_target;
    }
    for(unsigned i=0;i<2;i++) {
        if(motion_target(support.joints[i],support.origin[i])) {group_stop(B5_IO,now);return;}
    }
    if(motion_target(motion_joint,moving_target)) group_stop(B5_IO,now);
}
static void group_step(uint32_t now)
{
    static const unsigned stop_joints[3]={5U,4U,6U};
    int r;
    unsigned i;
    local_ui_poll(now);
    if (queued==5 || (cancelled && group_phase!=GROUP_STOPPING &&
                      group_phase!=GROUP_FINISHED)) group_stop(B5_CANCELLED,now);
    else if (queued==6) {
        group_used=1; bh_start(&support,now); group_phase=GROUP_PREPARE;
    }
    else if(queued==30) {
        group_used=1;
        int result=0;
        for(unsigned j=4;j<=6;j++) {int e=control_motor_set_active_report(j,RT_TRUE);if(e && !result) result=e;}
        for(unsigned j=4;j<=6;j++) {int stop_result=axis_stop(j);if(stop_result && !result) result=stop_result;}
        record_valid=0;record_started=now;record_requested=0;
        if(result) group_stop(B5_IO,now);
        else {group_phase=GROUP_RECORD_PREPARE;guard.state=B5_VERIFY_STOP;}
    }
    else if(queued==40) {
        group_used=1;memset(&resistance,0,sizeof(resistance));
        bh_start(&support,now);guard.state=B5_READ_POS;group_phase=GROUP_RESIST_PREPARE;
        if(control_motor_set_active_report(5U,RT_TRUE)) group_stop(B5_IO,now);
    }
    else if(queued==20 && local_action==ICAN_UI_PLAY) {
        group_used=1;play_prepare_axis=0;
        memset(teach_prepare,0,sizeof(teach_prepare));
        bh_start_axes(&teach_prepare[0],now,4U,0,1,teach_speed[0],1.0f,1);
        guard.state=B5_READ_POS;group_phase=GROUP_PLAY_PREPARE;
    }
    else if(queued==20) {
        group_used=1;motion_started=0;
        memset(&axis_motion,0,sizeof(axis_motion));memset(&drive,0,sizeof(drive));
        {
            motion_joint=local_action==ICAN_UI_START_LIFT ? 4U:local_action==ICAN_UI_START_ROTATE ? 6U:5U;
            /* Both moving axes use this session's fresh position, with unchanged speed/current. */
            axis_config=(rehab_arm_lift_config_t){1U,1U,
                CONTROL_MOTOR_P_MIN_RAD,CONTROL_MOTOR_P_MAX_RAD,0.0f,
                motion_joint==4 ? 1.15f:0.24f,
                motion_joint==4 ? 0.60f:0.20f,
                motion_joint==4 ? 1.50f:0.50f,
                motion_joint==4 ? 1.60f:0.35f,0.035f,0.03f,5U,500U,40000U};
            /* Admission uses fresh BH sampling for both actions, not idle cached feedback. */
            unsigned held[2],count=0;
            for(unsigned j=4;j<=6;j++) if(j!=motion_joint) held[count++]=j;
            bh_start_axes(&support,now,held[0],held[1],2,
                CONTROL_REHAB_FIXED_ACTION_COMMAND_SPEED_RAD_S,1.0f,0);
            guard.state=B5_READ_POS;group_phase=GROUP_AXIS_PREPARE;
        }
    }
    queued=0;
    if(group_phase==GROUP_RESIST_PREPARE || group_phase==GROUP_RESIST_RUN) {
        /* Reuse accepted 4/6 holds; the resistance guard bounds the full session to 120s. */
        if(support.phase==BH_READY) support.started=now;
        r=bh_tick(&support,&hold_io,now);
        if(r<0) group_stop(r,now);
        else if(r==1) {
            if(resistance.phase==BR_IDLE) br_start(&resistance,now);
            r=br_tick(&resistance,&io,now);
            guard.origin=resistance.origin;guard.position=resistance.position;guard.target=resistance.origin;
            if(r<0) {
                guard.failure_state=resistance.failure_phase;
                guard.last_index=resistance.last_index;
                group_stop(r,now);
            } else if(r==2) {guard.completed_cycles=120U;group_stop(0,now);}
            else if(r==1) {
                group_phase=GROUP_RESIST_RUN;guard.state=B5_MONITOR_POS;
                guard.completed_cycles=(now-resistance.running_at)/1000U;
            }
        }
    }
    if(group_phase==GROUP_RECORD_PREPARE || group_phase==GROUP_RECORD) record_step(now);
    if(group_phase==GROUP_PLAY_PREPARE || group_phase==GROUP_PLAY_RUN) playback_step(now);
    if(group_phase==GROUP_AXIS_PREPARE || group_phase==GROUP_AXIS_RUN)
        axis_action_step(now);
    if (group_phase==GROUP_PREPARE || group_phase==GROUP_CURL) {
        r=bh_tick(&support,&hold_io,now);
        if (r<0) group_stop(r,now);
        else if (group_phase==GROUP_PREPARE && r==1) {
            r=b5_start_curl(&guard,&profile,&curl_config,now);
            if (!r) {
                guard.relative_curl=1U;
                if(local_session) guard.repeat_curl=1;
            }
            if (r) group_stop(r,now);
            else group_phase=GROUP_CURL;
        }
        if (group_phase==GROUP_CURL) {
            unsigned previous_cycles=guard.completed_cycles;
            b5_tick(&guard,&profile,&io,now);
            if (local_session && guard.completed_cycles!=previous_cycles) support.started=now;
            if (!b5_active(&guard)) group_stop(guard.result,now);
        }
    }
    if (group_phase==GROUP_STOPPING && (uint32_t)(now-stop_at)>=50U) {
        stop_at=now;
        /* 每个轴独立最多三次；一个发送失败也继续停止另外两轴。
         * 成功结束同样释放三轴，机械支撑必须保留，STOP不是机械锁。 */
        for (i=0;i<3;i++) if (!(stop_done&(1U<<i))) {
            stop_results[i]=axis_stop(stop_joints[i]); stop_attempts[i]++;
            if (!stop_results[i] || stop_attempts[i]>=3U) stop_done|=1U<<i;
        }
        guard.attempts=stop_attempts[0];
        if (stop_done==7U) {
            guard.stop_result=0;
            for(i=0;i<3;i++) if (stop_results[i] && !guard.stop_result)
                guard.stop_result=stop_results[i];
            if (!guard.result && guard.stop_result) guard.result=B5_IO;
            if (local_session && guard.stop_result) local_fault=1;
            guard.state=guard.result ? B5_FAULT:B5_DONE;
            group_phase=GROUP_FINISHED;
        }
    }
    local_ui_snapshot();
}
static void entry(void *unused)
{
    RT_UNUSED(unused);
    while (1) {
        rt_mutex_take(&lock,RT_WAITING_FOREVER);
        /* 一次授权最多等待60秒，重启清空；失败、停止和重复请求不会自动重开。 */
        if (g_bench5_arm==ARM_KEY && !arm_seen) { arm_seen=1; armed_at=millis(); }
        if (arm_seen && (uint32_t)(millis()-armed_at)>=60000U) g_bench5_arm=0;
        group_step(millis());
        rt_mutex_release(&lock);
        rt_thread_mdelay(10);
    }
}
static int init(void)
{
    rt_thread_t thread;
    if (rt_mutex_init(&lock,"bench5",RT_IPC_FLAG_PRIO)!=RT_EOK) return -RT_ERROR;
    if (rt_mutex_init(&tx_lock,"b5tx",RT_IPC_FLAG_PRIO)!=RT_EOK) return -RT_ERROR;
    thread=rt_thread_create("bench5",entry,RT_NULL,3072,18,5);
    if (!thread) return -RT_ENOMEM;
    owner=thread;
    ready=RT_TRUE;
    return rt_thread_startup(thread);
}
INIT_APP_EXPORT(init);
