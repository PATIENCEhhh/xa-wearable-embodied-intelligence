/* 编译实际M33包装器；只替换RT线程与控制IO，不复制准入/独占实现。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <rtthread.h>
#include "control/control_layer_cfg.h"
#define __CONTROL_LAYER_H__
typedef struct {
    unsigned valid,motor_id,index,timestamp,raw_u8,response_type,response_seq;
    float value_f32;
} control_motor_param_report_t;
typedef struct { unsigned timestamp,fault_summary,mode_state,motor_id; float pos_rad,vel_rad_s,torque_nm; } control_motor_feedback_t;
static rt_tick_t clock_ms;
static rt_thread_t self;
static control_motor_param_report_t cached_param;
static float positions[8],targets[8],parameters[8][4];
static float currents[8];
static unsigned current_setpoint_error;
static float feedback_velocities[8];
static unsigned modes[8],enables[8],stops[8],target_writes[8];
static int stop_errors[8];
static unsigned legacy_hold_calls[8];
static unsigned running_param_reads,freeze5,feedback5_tick,feedback5_fault,feedback5_id;
static unsigned feedback5_mode_override, testing_axis_actions, stale_idle4, stale_idle6;
static unsigned parameter_read_failure;
rt_tick_t rt_tick_get(void) { return clock_ms; }
rt_thread_t rt_thread_self(void) { return self; }
int rt_mutex_take(struct rt_mutex *m,int timeout) { (void)timeout; assert(!m->held); m->held=1; return 0; }
int rt_mutex_release(struct rt_mutex *m) { assert(m->held); m->held=0; return 0; }
int rt_mutex_init(struct rt_mutex *m,const char *n,int f) { (void)n; (void)f; m->held=0; return 0; }
rt_thread_t rt_thread_create(const char *n,void (*f)(void *),void *p,unsigned s,unsigned a,unsigned b)
{ (void)n;(void)f;(void)p;(void)s;(void)a;(void)b;return (void *)1; }
int rt_thread_startup(rt_thread_t t) { (void)t;return 0; }
void rt_thread_mdelay(unsigned ms) { clock_ms+=ms; }
int control_motor_read_parameter(unsigned a,unsigned b) {
    if(parameter_read_failure==a) return -1;
    if (a==5 && modes[5]==2) { running_param_reads++;return -1; }
    float value=b==0x7019 ? positions[a]:b==0x701b ? 0:
        b==0x7005 ? parameters[a][0]:b==0x7018 ? parameters[a][1]:
        b==0x7017 ? parameters[a][2]:b==0x7006 ? currents[a]:targets[a];
    cached_param=(control_motor_param_report_t){.valid=1,.motor_id=a,.index=b,
        .timestamp=clock_ms,.response_type=0x11,.response_seq=cached_param.response_seq+1,
        .value_f32=value,.raw_u8=(unsigned)value};
    return 0;
}
int control_get_last_motor_param(control_motor_param_report_t *p) { *p=cached_param;return 0; }
int control_motor_write_parameter(unsigned a,unsigned b,float v,int mode) {
    (void)mode;
    if (b==0x7006) currents[a]=v;
    else if (b==0x7016) { targets[a]=v; target_writes[a]++; }
    else parameters[a][b==0x7005 ? 0:b==0x7018 ? 1:2]=v;
    return 0;
}
int control_motor_enable(unsigned a) {
    if (!testing_axis_actions && a==5) assert(modes[4]==2 && modes[6]==2);
    if (testing_axis_actions==a) {
        for(unsigned j=4;j<=6;j++) if(j!=a) assert(modes[j]==2);
    }
    modes[a]=2; enables[a]++;return 0;
}
int control_motor_csp_setpoint(unsigned a,float v) {
    assert(testing_axis_actions || a==5);
    if(testing_axis_actions==9U) assert(modes[4]==2 && modes[5]==2 && modes[6]==2);
    float ratio=a==4 ? CONTROL_MOTOR_JOINT4_GEAR_RATIO:a==5 ? CONTROL_MOTOR_JOINT5_GEAR_RATIO:CONTROL_MOTOR_JOINT6_GEAR_RATIO;
    float zero=a==4 ? CONTROL_MOTOR_JOINT4_ZERO_OFFSET_RAD:a==5 ? CONTROL_MOTOR_JOINT5_ZERO_OFFSET_RAD:CONTROL_MOTOR_JOINT6_ZERO_OFFSET_RAD;
    float direction=a==4 ? CONTROL_MOTOR_JOINT4_DIRECTION:a==5 ? CONTROL_MOTOR_JOINT5_DIRECTION:CONTROL_MOTOR_JOINT6_DIRECTION;
    return control_motor_write_parameter(a,0x7016,v*ratio*direction+zero,0);
}
int control_motor_position_control_with_current_limit(unsigned a,float p,float speed,float current,int csp) {
    assert((testing_axis_actions ? a>=4 && a<=6 : a==4 || a==6) && csp);
    assert(current==(testing_axis_actions==9U && a==5U ? 2.0f:CONTROL_REHAB_FIXED_ACTION_LIMIT_CURRENT_A));
    const float ratio=a==4 ? CONTROL_MOTOR_JOINT4_GEAR_RATIO:a==5 ? CONTROL_MOTOR_JOINT5_GEAR_RATIO:CONTROL_MOTOR_JOINT6_GEAR_RATIO;
    const float zero=a==4 ? CONTROL_MOTOR_JOINT4_ZERO_OFFSET_RAD:a==5 ? CONTROL_MOTOR_JOINT5_ZERO_OFFSET_RAD:CONTROL_MOTOR_JOINT6_ZERO_OFFSET_RAD;
    const float direction=a==4 ? CONTROL_MOTOR_JOINT4_DIRECTION:a==5 ? CONTROL_MOTOR_JOINT5_DIRECTION:CONTROL_MOTOR_JOINT6_DIRECTION;
    float raw=p*direction*ratio+zero,raw_speed=speed*ratio;
    assert(fabsf(raw-targets[a])<0.0004f); /* 必须预置本轮目标，不得使能后追逐历史零点。 */
    float expected_speed=testing_axis_actions==9U ? (a==4 ? 1.15f:a==5 ? 0.12f:0.24f):
        testing_axis_actions==a ? (a==4 ? 1.30f:0.28f):CONTROL_REHAB_FIXED_ACTION_COMMAND_SPEED_RAD_S;
    assert(fabsf(raw_speed-expected_speed)<0.000001f);
    assert(parameters[a][1]==current && fabsf(parameters[a][2]-raw_speed)<1e-6f);
    legacy_hold_calls[a]++;
    control_motor_write_parameter(a,0x7005,5,1);
    control_motor_enable(a);
    control_motor_write_parameter(a,0x7018,current,0);
    control_motor_write_parameter(a,0x7017,raw_speed,0);
    return control_motor_write_parameter(a,0x7016,raw,0);
}
int control_motor_set_active_report(unsigned a,int v) { (void)a;(void)v;return 0; }
int control_motor_current_setpoint(unsigned a,float current) {
    assert(a==5 && modes[a]==2 && parameters[a][0]==3.0f && fabsf(current)<=1.0f);
    if(current_setpoint_error) return -1;
    currents[a]=current;return 0;
}
int control_motor_stop(unsigned a,int clear) {
    assert(!clear);stops[a]++;
    if (!stop_errors[a]) {modes[a]=0;currents[a]=0;}
    return stop_errors[a];
}
int control_get_motor_feedback(unsigned a,control_motor_feedback_t *p) {
    *p=(control_motor_feedback_t){.timestamp=((a==4 && stale_idle4) || (a==6 && stale_idle6)) ? clock_ms-1000U:a==5 && freeze5 ? feedback5_tick:clock_ms,
        .motor_id=a==5 && feedback5_id ? feedback5_id:a,
        .fault_summary=a==5 ? feedback5_fault:0,
        .mode_state=a==5 && feedback5_mode_override ? 1:modes[a],
        .pos_rad=positions[a],.vel_rad_s=feedback_velocities[a]};return 0;
}
#include "../../firmware/m33/applications/m33_bench5.c"
static unsigned frame(unsigned type,unsigned motor) { return (type<<24)|motor; }
static void allowed(unsigned type,unsigned motor) {
    assert(m33_bench5_tx_enter(frame(type,motor),1)==0); m33_bench5_tx_leave();
}
static void denied(unsigned type,unsigned motor) {
    assert(m33_bench5_tx_enter(frame(type,motor),1)==-RT_EBUSY);
    assert(!tx_lock.held);
}
int main(void) {
    unsigned type;
    float value;
    self=(void *)2;
    assert(m33_bench5_tx_enter(frame(3,5),1)==-RT_EBUSY);
    assert(init()==0);
    cached_param=(control_motor_param_report_t){.valid=1,.motor_id=5,.index=0x7018,
        .timestamp=0,.response_type=0x11,.response_seq=1,.value_f32=0.2f};
    parameters[5][1]=0.2f;
    assert(read_param(0x7018)==0);
    cached_param.response_seq=read_seq; /* 模拟尚未到达新回复，不能接受请求前缓存。 */
    assert(reply(0x7018,0,&value)==0);
    cached_param.response_seq++;
    cached_param.response_type=0x12;
    assert(reply(0x7018,0,&value)==0); /* 同索引写回显不能证明参数读回。 */
    cached_param.response_type=0x11;
    assert(reply(0x7018,0,&value)==1 && value==0.2f); /* 同tick新回复有效。 */
    assert(m33_bench5_command(6)==B5_LOCKED);
    for(type=1;type<32;type++) if(type!=4 && type!=17) denied(type,5);
    allowed(0,5); allowed(17,4);
    assert(m33_bench5_tx_enter(0x0a3,0)==-RT_EBUSY); /* 标准帧使能也不可绕过。 */
    assert(m33_bench5_tx_enter(0x322,0)==0); m33_bench5_tx_leave();
    g_bench5_arm=ARM_KEY; assert(m33_bench5_command(6)==0);
    assert(!g_bench5_arm && queued==6);
    denied(17,5); /* 活动期间外部读取不能污染参数回复缓存。 */
    self=owner; allowed(17,5); allowed(18,5); allowed(3,5);
    allowed(18,4); allowed(3,6); denied(18,7); denied(3,3); denied(6,5);
    self=(void *)2; allowed(4,5);
    assert(cancelled && !g_bench5_arm);
    self=owner; denied(3,5); denied(18,5); allowed(4,5);
    assert(m33_bench5_command(5)==0 && queued==5);
    g_bench5_arm=ARM_KEY; assert(m33_bench5_command(6)==B5_LOCKED);
    assert(queued==5); /* 迟到的启动不能覆盖STOP。 */
    puts("PASS actual M33 read-reply provenance/same-tick freshness/admission/TX ownership/external STOP latch");

    /* 执行真实包装器：4/6先完成保持，5随后运动，最终三轴停用。 */
    memset(&guard,0,sizeof(guard)); memset(&support,0,sizeof(support));
    group_phase=GROUP_IDLE; group_used=0; queued=0;cancelled=0;
    positions[4]=3.1f;positions[5]=5.021363544f;positions[6]=5.2f;
    g_bench5_arm=ARM_KEY;assert(m33_bench5_command(6)==0);self=owner;
    for(clock_ms=10;clock_ms<50000;clock_ms+=10) {
        if(modes[5]==2) {
            float delta=targets[5]-positions[5];
            positions[5]+=delta>0.0012f ? 0.0012f:delta< -0.0012f ? -0.0012f:delta;
        }
        group_step(clock_ms);
        assert(positions[4]==3.1f && positions[6]==5.2f);
        if(group_phase==GROUP_FINISHED) break;
    }
    assert(clock_ms<50000 && guard.result==0 && guard.state==B5_DONE);
    assert(enables[4]==1 && enables[5]==1 && enables[6]==1);
    assert(fabsf(targets[4]-3.1f)<0.000001f && fabsf(targets[6]-5.2f)<0.000001f);
    assert(legacy_hold_calls[4]==1 && legacy_hold_calls[6]==1);
    assert(positions[5]<5.06f && modes[4]==0 && modes[5]==0 && modes[6]==0);
    assert(m33_bench5_command(6)==B5_LOCKED || m33_bench5_command(6)==B5_BUSY);
    puts("PASS actual three-axis prepare/hold/curl-one-cycle/all-axis STOP");

    /* 两种跨状态机故障都必须整组退出：准备期间取消；弯肘期间保持轴被带偏。 */
    for(unsigned scenario=0;scenario<3;scenario++) {
        unsigned before5=enables[5],before4=stops[4],before6=stops[6];
        memset(&guard,0,sizeof(guard));memset(&support,0,sizeof(support));
        group_phase=GROUP_IDLE;group_used=0;queued=0;cancelled=0;
        positions[4]=3.1f;positions[5]=5.021363544f;positions[6]=5.2f;
        g_bench5_arm=ARM_KEY;assert(m33_bench5_command(6)==0);
        unsigned injected=0;
        for(unsigned elapsed=0;elapsed<12000;elapsed+=10) {
            clock_ms+=10;
            if(!injected && (scenario==0 ? support.enabled_mask==1U:
                            group_phase==GROUP_CURL && modes[5]==2)) {
                if(scenario==0) assert(m33_bench5_command(5)==0);
                else positions[scenario==1 ? 4:6]+=0.04f;
                injected=1;
            }
            group_step(clock_ms);
            if(group_phase==GROUP_FINISHED) break;
        }
        assert(injected && group_phase==GROUP_FINISHED && guard.state==B5_FAULT);
        assert(guard.result==(scenario==0 ? B5_CANCELLED:B5_LIMIT));
        assert(!modes[4] && !modes[5] && !modes[6] && stops[4]>before4 && stops[6]>before6);
        if(scenario==0) assert(enables[5]==before5);
        else {
            assert(support.failed_joint==(scenario==1 ? 4U:6U));
            float p,v,o; rt_uint32_t reason,mask;
            m33_bench5_trip(&p,&v,&o,&reason,&mask);
            assert(reason==BH_TRIP_POSITION && mask==3U && fabsf(p-o)>0.035f && v==0);
        }
    }
    puts("PASS partial-enable cancel and each support axis drift abort whole group");

    /* 复用旧接口/旧0.35上限后，实测0.102218628不再被新增0.10判据拦截。
     * 重放位置几乎不动而速度有读数的场景，验证完整弯举及停止，不证明实物噪声根因。 */
    {
        unsigned before5=enables[5],injected=0;
        memset(&guard,0,sizeof(guard));memset(&support,0,sizeof(support));
        group_phase=GROUP_IDLE;group_used=0;queued=0;cancelled=0;
        positions[4]=1.14642048f;positions[5]=5.1126814f;positions[6]=1.3300467f;
        g_bench5_arm=ARM_KEY;assert(m33_bench5_command(6)==0);
        for(unsigned elapsed=0;elapsed<50000;elapsed+=10) {
            clock_ms+=10;
            if(!injected && group_phase==GROUP_CURL && guard.state==B5_READ_VEL) {
                positions[4]=1.14642334f;feedback_velocities[4]=0.102218628f;injected=1;
            }
            if(modes[5]==2) {
                float delta=targets[5]-positions[5];
                positions[5]+=delta>0.0012f ? 0.0012f:delta< -0.0012f ? -0.0012f:delta;
            }
            group_step(clock_ms);
            if(group_phase==GROUP_FINISHED) break;
        }
        assert(injected && group_phase==GROUP_FINISHED && guard.result==0 && guard.first_error==0);
        assert(support.phase==BH_READY && support.trip.reason==0);
        assert(enables[5]==before5+1 && guard.enable_attempts==1 && guard.target_attempts>0);
        assert(!modes[4] && !modes[5] && !modes[6]);
        feedback_velocities[4]=0;
    }
    puts("PASS legacy CSP hold uses calibrated round-trip/raw baseline and shared limits; recorded 0.102218628 permits one complete curl cycle");

    /* 主动反馈路径：使能后参数查询故意失败，正常周期必须仍完成。
     * 同一缓存帧不得多次计入到位；缺帧250ms、故障、错ID/状态、超速/限位仍整组停。 */
    for(unsigned scenario=0;scenario<9;scenario++) {
        unsigned injected=0,injected_at=0;
        memset(&guard,0,sizeof(guard));memset(&support,0,sizeof(support));
        group_phase=GROUP_IDLE;group_used=0;queued=0;cancelled=0;
        positions[4]=1.1464f;positions[5]=5.0963f;positions[6]=1.3298f;
        running_param_reads=freeze5=feedback5_fault=feedback5_id=feedback5_mode_override=0;
        feedback_velocities[5]=0;
        if(scenario==8) clock_ms=UINT32_MAX-1200U;
        g_bench5_arm=ARM_KEY;assert(m33_bench5_command(6)==0);
        for(unsigned elapsed=0;elapsed<50000;elapsed+=10) {
            clock_ms+=10;
            if(modes[5]==2) {
                float delta=targets[5]-positions[5];
                positions[5]+=delta>0.0012f ? 0.0012f:delta< -0.0012f ? -0.0012f:delta;
            }
            if(!injected && guard.state==B5_MONITOR_POS) {
                injected=1;injected_at=clock_ms;
                if(scenario==1 || scenario==7) { freeze5=1;feedback5_tick=clock_ms; }
                if(scenario==2) feedback5_fault=1;
                if(scenario==3) feedback5_id=4;
                if(scenario==4) feedback5_mode_override=1;
                if(scenario==5) feedback_velocities[5]=0.4f;
                if(scenario==6) positions[5]=NAN;
            }
            if(scenario==7 && injected && (uint32_t)(clock_ms-injected_at)>=200U) freeze5=0;
            group_step(clock_ms);
            if(scenario==1 && injected && (uint32_t)(clock_ms-injected_at)<200U)
                assert(guard.curl.arrival_count<=1U);
            if(group_phase==GROUP_FINISHED) break;
        }
        assert(injected && group_phase==GROUP_FINISHED && !running_param_reads);
        assert(!modes[4] && !modes[5] && !modes[6]);
        assert(support.trip.reason==0);
        if(scenario==0 || scenario==7 || scenario==8) assert(guard.result==0);
        else if(scenario==1) assert(guard.first_error==B5_TIMEOUT);
        else if(scenario==5) assert(guard.first_error==B5_LIMIT);
        else assert(guard.first_error==B5_FEEDBACK);
        freeze5=feedback5_fault=feedback5_id=feedback5_mode_override=0;
        feedback_velocities[5]=0;
    }
    puts("PASS old active feedback: no running parameter reads, coherent/new samples, 200ms recovery, 250ms loss, fault/ID/mode/NaN/speed rejection and tick wrap");

    /* 每个动作执行真实三轴包装器：目标/驻留不同，门禁、反馈和STOP保持一致。 */
    for(unsigned op=12;op<=14;op++) for(unsigned scenario=0;scenario<4;scenario++) {
        rehab_curl_config_t selected;
        float peak=0;
        unsigned dwell_seen=0,injected=0;
        uint32_t dwell_at=0,dwell_time=0;
        memset(&guard,0,sizeof(guard));memset(&support,0,sizeof(support));
        group_phase=GROUP_IDLE;group_used=0;queued=0;cancelled=0;
        memset(modes,0,sizeof(modes));
        positions[4]=1.14665f;positions[5]=5.021363544f;positions[6]=1.36546f;
        feedback_velocities[4]=feedback_velocities[5]=feedback_velocities[6]=0;
        running_param_reads=freeze5=feedback5_fault=feedback5_id=feedback5_mode_override=0;
        assert(ican_fixed_action_config(op,&selected));
        assert(selected.bottom_target_pos_rad<=CONTROL_REHAB_CURL_BOTTOM_TARGET_RAW_RAD);
        assert(selected.top_target_pos_rad==CONTROL_REHAB_CURL_TOP_TARGET_RAW_RAD);
        assert(selected.dwell_ms==(op==14 ? 2000U:500U));
        assert(selected.segment_timeout_ms==CONTROL_REHAB_CURL_SEGMENT_TIMEOUT_MS);
        assert(m33_bench5_command(op)==B5_LOCKED); /* 无授权不能选择新动作。 */
        g_bench5_arm=ARM_KEY;assert(m33_bench5_command(op)==0);
        /* 已排队时不允许换动作；不能覆盖本轮配置。 */
        g_bench5_arm=ARM_KEY;assert(m33_bench5_command(op==12 ? 13:12)==B5_BUSY);
        g_bench5_arm=0;
        assert(curl_config.bottom_target_pos_rad==selected.bottom_target_pos_rad);
        for(unsigned elapsed=0;elapsed<50000;elapsed+=10) {
            clock_ms+=10;
            if(modes[5]==2) {
                float delta=targets[5]-positions[5];
                positions[5]+=delta>0.0012f ? 0.0012f:delta< -0.0012f ? -0.0012f:delta;
                if(positions[5]>peak) peak=positions[5];
            }
            if(!injected && guard.state==B5_MONITOR_POS) {
                injected=1;
                if(scenario==1) {freeze5=1;feedback5_tick=clock_ms;}
                if(scenario==2) assert(m33_bench5_command(5)==0);
                if(scenario==3) positions[6]+=0.04f;
            }
            group_step(clock_ms);
            if(guard.curl.phase==REHAB_CURL_PHASE_DWELL_BOTTOM) {
                if(!dwell_seen) {dwell_seen=1;dwell_at=clock_ms;}
                dwell_time=clock_ms-dwell_at;
            }
            if(group_phase==GROUP_FINISHED) break;
        }
        assert(group_phase==GROUP_FINISHED && !running_param_reads);
        assert(!modes[4] && !modes[5] && !modes[6]);
        if(scenario==0) {
            assert(guard.state==B5_DONE && guard.result==0 && !guard.first_error);
            assert(fabsf(peak-selected.bottom_target_pos_rad)<0.001f);
            assert(fabsf(positions[5]-selected.top_target_pos_rad)<0.03f);
            assert(dwell_seen && dwell_time+20U>=selected.dwell_ms);
        } else if(scenario==1) assert(guard.first_error==B5_TIMEOUT);
        else if(scenario==2) assert(guard.result==B5_CANCELLED);
        else assert(support.trip.reason!=0 && guard.result!=0);
        g_bench5_arm=ARM_KEY;
        assert(m33_bench5_command(op)!=0); /* 修改RAM授权也不能重放已用周期。 */
        g_bench5_arm=0;freeze5=0;
    }
    assert(!ican_fixed_action_config(15,&curl_config));
    puts("PASS 30/60/pause fixed actions: target/dwell, no-arm/busy/replay rejection, stale/cancel/hold-drift all-axis STOP");

    /* 某轴STOP失败不妨碍停止其他轴；失败有界且不能冒充完成。 */
    for(unsigned failed=4;failed<=6;failed++) {
        unsigned before4=stops[4],before5=stops[5],before6=stops[6];
        stop_errors[failed]=-1;group_phase=GROUP_CURL;
        group_stop(B5_OK,clock_ms);
        for(unsigned tick=0;tick<200;tick+=10) { group_step(clock_ms);clock_ms+=10; }
        assert(group_phase==GROUP_FINISHED && guard.state==B5_FAULT && guard.stop_result==-1);
        assert(stops[4]>before4 && stops[5]>before5 && stops[6]>before6);
        assert(stop_attempts[failed==5 ? 0:failed==4 ? 1:2]==3);
        stop_errors[failed]=0;
    }
    puts("PASS independent bounded STOP failure for each of 4/5/6");

    /* 四个屏幕动作都通过真实包装器验证连续循环及停止，不新增控制算法。 */
    for(unsigned action=0;action<4;action++)
    for(unsigned scenario=0;scenario<5;scenario++) {
        static const unsigned commands[]={ICAN_UI_START_CURL,ICAN_UI_START_SMALL,
            ICAN_UI_START_MEDIUM,ICAN_UI_START_PAUSE};
        rehab_curl_config_t expected;
        unsigned injected=0,before5=enables[5],completed_before_stop=0;
        memset(&guard,0,sizeof(guard));memset(&support,0,sizeof(support));
        memset(modes,0,sizeof(modes));
        memset((void *)&g_ican_curl_ui,0,sizeof(g_ican_curl_ui));
        g_ican_curl_ui.magic=ICAN_UI_MAGIC;
        group_phase=GROUP_IDLE;group_used=0;queued=0;cancelled=0;
        local_session=local_fault=ui_seen_seq=ui_heartbeat=ui_heartbeat_at=0;
        positions[4]=1.1467f;positions[5]=5.02136f;positions[6]=1.3656f;
        freeze5=feedback5_fault=feedback5_id=feedback5_mode_override=0;
        feedback_velocities[4]=feedback_velocities[5]=feedback_velocities[6]=0;
        assert(ican_fixed_action_config(ican_ui_action_opcode(commands[action]),&expected));
        g_ican_curl_ui.heartbeat=1;g_ican_curl_ui.command=commands[action];g_ican_curl_ui.seq=1;
        for(unsigned elapsed=0;elapsed<120000;elapsed+=10) {
            clock_ms+=10;
            if(!(scenario==1 && injected)) g_ican_curl_ui.heartbeat++;
            if(modes[5]==2) {
                float delta=targets[5]-positions[5];
                positions[5]+=delta>0.0012f ? 0.0012f:delta< -0.0012f ? -0.0012f:delta;
            }
            if(!injected && ((scenario==0 && guard.completed_cycles>=3) ||
                (scenario!=0 && guard.state==B5_MONITOR_POS))) {
                injected=1;completed_before_stop=guard.completed_cycles;
                if(scenario==0 || scenario==4) {g_ican_curl_ui.command=2;g_ican_curl_ui.seq++;}
                if(scenario==2) {freeze5=1;feedback5_tick=clock_ms;}
                if(scenario==3) positions[4]+=0.04f;
            }
            group_step(clock_ms);
            assert(curl_config.bottom_target_pos_rad==expected.bottom_target_pos_rad);
            assert(curl_config.dwell_ms==expected.dwell_ms);
            if(group_phase==GROUP_FINISHED) break;
        }
        assert(injected && group_phase==GROUP_FINISHED);
        assert(!modes[4] && !modes[5] && !modes[6]);
        assert(enables[5]==before5+1U); /* 连续多轮绝不反复STOP/重新使能。 */
        if(scenario==0) assert(completed_before_stop>=3);
        if(scenario==0 || scenario==4) {
            assert(!local_fault && guard.result==B5_CANCELLED);
            g_ican_curl_ui.command=1;g_ican_curl_ui.seq++;g_ican_curl_ui.heartbeat++;
            clock_ms+=10;group_step(clock_ms);
            assert((int32_t)g_ican_curl_ui.result==0 && group_phase==GROUP_PREPARE);
            /* 准备中立即STOP也不得启动5号。 */
            g_ican_curl_ui.command=2;g_ican_curl_ui.seq++;
            for(unsigned t=0;t<250;t+=10) {clock_ms+=10;g_ican_curl_ui.heartbeat++;group_step(clock_ms);}
            assert(group_phase==GROUP_FINISHED && !modes[4] && !modes[5] && !modes[6]);
            assert(enables[5]==before5+1U);
        } else {
            assert(local_fault);
            g_ican_curl_ui.command=1;g_ican_curl_ui.seq++;g_ican_curl_ui.heartbeat++;
            clock_ms+=10;group_step(clock_ms);
            assert((int32_t)g_ican_curl_ui.result==B5_LOCKED && group_phase==GROUP_FINISHED);
        }
        freeze5=0;
    }
    puts("PASS all four LVGL actions: correct target/dwell, 3-cycle repeat, STOP/restart, heartbeat/stale/hold fault latch");
    /* 空闲STOP、重复点击和停止发送失败均走真实包装器，避免只测理想循环。 */
    for(unsigned scenario=0;scenario<4;scenario++) {
        unsigned before5=enables[5];
        memset(&guard,0,sizeof(guard));memset(&support,0,sizeof(support));
        memset(modes,0,sizeof(modes));
        memset((void *)&g_ican_curl_ui,0,sizeof(g_ican_curl_ui));
        group_phase=GROUP_IDLE;group_used=queued=cancelled=0;
        local_session=local_fault=ui_seen_seq=ui_heartbeat=ui_heartbeat_at=0;
        positions[4]=1.1467f;positions[5]=5.02136f;positions[6]=1.3656f;
        /* 仅刷新屏幕心跳绝不等于启动；包括32位时钟回绕。 */
        clock_ms=UINT32_MAX-100U;
        for(unsigned t=0;t<300;t+=10) {
            clock_ms+=10;g_ican_curl_ui.heartbeat++;group_step(clock_ms);
        }
        assert(group_phase==GROUP_IDLE && enables[5]==before5);
        if(scenario==0 || scenario==3) {
            if(scenario==3) stop_errors[6]=-1;
            g_ican_curl_ui.command=2;g_ican_curl_ui.seq++;
            for(unsigned t=0;t<250;t+=10) {
                clock_ms+=10;g_ican_curl_ui.heartbeat++;group_step(clock_ms);
            }
            assert(group_phase==GROUP_FINISHED && enables[5]==before5);
            if(scenario==3) assert(local_fault && guard.stop_result==-1);
        }
        if(scenario==1) { /* 无心跳的新START必须拒绝。 */
            g_ican_curl_ui.heartbeat=0;
        }
        g_ican_curl_ui.command=1;g_ican_curl_ui.seq++;
        clock_ms+=10;group_step(clock_ms);
        if(scenario==1 || scenario==3) {
            assert((int32_t)g_ican_curl_ui.result==B5_LOCKED && enables[5]==before5);
        } else {
            assert((int32_t)g_ican_curl_ui.result==0 && group_phase==GROUP_PREPARE);
            g_ican_curl_ui.seq++; /* 第二次新点击不得重采保持基线或重置流程。 */
            clock_ms+=10;g_ican_curl_ui.heartbeat++;group_step(clock_ms);
            assert((int32_t)g_ican_curl_ui.result==B5_BUSY && group_phase==GROUP_PREPARE);
            g_ican_curl_ui.command=2;g_ican_curl_ui.seq++;
            for(unsigned t=0;t<250;t+=10) {
                clock_ms+=10;g_ican_curl_ui.heartbeat++;group_step(clock_ms);
            }
            assert(group_phase==GROUP_FINISHED && !local_fault);
            /* 停止后旧序号即使改成START也不应被再次执行。 */
            g_ican_curl_ui.command=1;
            clock_ms+=10;g_ican_curl_ui.heartbeat++;group_step(clock_ms);
            assert(group_phase==GROUP_FINISHED && enables[5]==before5);
        }
        stop_errors[6]=0;
    }
    puts("PASS LVGL idle STOP/restart, no implicit start, missing heartbeat, duplicate click/replay, STOP failure and tick wrap");
    /* Run actual LVGL wrapper for lift and rotation: hold first, repeat, STOP, fault. */
    for(unsigned moving=4;moving<=6;moving+=2)
    for(unsigned pose=0;pose<4U;pose++)
    for(unsigned scenario=0;scenario<4;scenario++) {
        testing_axis_actions=moving;
        memset(&guard,0,sizeof(guard));memset(&support,0,sizeof(support));
        memset(modes,0,sizeof(modes));
        memset((void *)&g_ican_curl_ui,0,sizeof(g_ican_curl_ui));
        group_phase=GROUP_IDLE;group_used=queued=cancelled=0;
        local_session=local_fault=ui_seen_seq=ui_heartbeat=ui_heartbeat_at=0;
        static const float lift_starts[]={1.128856182f,6.103863239f,8.0561991f,-10.0f};
        static const float rotate_starts[]={1.417203f,-2.0f,7.0f,12.0f};
        positions[4]=moving==4 ? lift_starts[pose]:6.103863239f;
        positions[5]=5.02136f;positions[6]=moving==6 ? rotate_starts[pose]:1.417203f;
        stale_idle4=moving==4U;stale_idle6=moving==6U;
        float origin[8];memcpy(origin,positions,sizeof(origin));
        unsigned before[8];memcpy(before,enables,sizeof(before));
        unsigned injected=0,completed=0,visited=0;
        g_ican_curl_ui.heartbeat=1;g_ican_curl_ui.seq=1;
        g_ican_curl_ui.command=moving==4 ? ICAN_UI_START_LIFT:ICAN_UI_START_ROTATE;
        for(unsigned elapsed=0;elapsed<120000;elapsed+=10) {
            clock_ms+=10;g_ican_curl_ui.heartbeat++;
            if(modes[moving]==2) {
                float delta=targets[moving]-positions[moving];
                float step=moving==4 ? 0.013f:0.0028f;
                positions[moving]+=delta>step ? step:delta < -step ? -step:delta;
            }
            if(!injected && (scenario==3 ? group_phase==GROUP_AXIS_PREPARE:
                scenario==0 ? guard.completed_cycles>=2:group_phase==GROUP_AXIS_RUN)) {
                injected=1;completed=guard.completed_cycles;
                if(scenario==0 || scenario==3) {g_ican_curl_ui.command=ICAN_UI_STOP;g_ican_curl_ui.seq++;}
                if(scenario==1) positions[moving==4 ? 6:4]+=0.04f;
                if(scenario==2) {freeze5=1;feedback5_tick=clock_ms-300U;}
            }
            if(drive.phase==BH_STOP) stale_idle4=stale_idle6=0;
            group_step(clock_ms);
            if(group_phase==GROUP_AXIS_RUN && axis_motion.state==LIFT_DWELL_HIGH) visited=1;
            if(scenario==0) for(unsigned j=4;j<=6;j++) if(j!=moving) {
                assert(positions[j]==origin[j]);
                if(group_phase==GROUP_AXIS_RUN) assert(fabsf(targets[j]-origin[j])<0.00001f);
            }
            if(group_phase==GROUP_FINISHED) break;
        }
        assert(injected && group_phase==GROUP_FINISHED);
        assert(!modes[4] && !modes[5] && !modes[6]);
        if(scenario==0) {
            assert(completed>=2 && visited && !local_fault);
            if(moving==4) {
                float expected=fmaxf(origin[4]-11.029982281f,CONTROL_MOTOR_P_MIN_RAD+0.05f);
                assert(fabsf(axis_config.raised_raw-expected)<0.00001f);
                assert(axis_config.min_raw>=CONTROL_MOTOR_P_MIN_RAD);
                assert(axis_config.max_raw<=CONTROL_MOTOR_P_MAX_RAD);
                assert(origin[4]-axis_config.raised_raw<=11.0300f);
            } else {
                float expected=fminf(origin[6]+1.482063365f,CONTROL_MOTOR_P_MAX_RAD-0.05f);
                assert(fabsf(axis_config.raised_raw-expected)<0.00001f);
                assert(axis_config.min_raw>=CONTROL_MOTOR_P_MIN_RAD && axis_config.max_raw<=CONTROL_MOTOR_P_MAX_RAD);
            }
            assert(fabsf(positions[moving]-origin[moving])<0.03f);
            for(unsigned j=4;j<=6;j++) assert(enables[j]==before[j]+1U);
        } else if(scenario==3) assert(enables[moving]==before[moving] && !local_fault);
        else assert(local_fault && guard.result!=0);
        freeze5=0;stale_idle4=stale_idle6=0;
    }
    /* Out-of-envelope start must not enable any motor. */
    for(unsigned moving=4;moving<=6;moving+=2) {
        testing_axis_actions=moving;
        memset(&guard,0,sizeof(guard));memset(&support,0,sizeof(support));
        memset(modes,0,sizeof(modes));memset((void *)&g_ican_curl_ui,0,sizeof(g_ican_curl_ui));
        group_phase=GROUP_IDLE;group_used=queued=cancelled=0;
        local_session=local_fault=ui_seen_seq=ui_heartbeat=ui_heartbeat_at=0;
        positions[4]=6.103863239f;positions[5]=5.02136f;positions[6]=1.417203f;
        positions[moving]=13.0f;
        unsigned before=enables[moving];
        g_ican_curl_ui.command=moving==4 ? ICAN_UI_START_LIFT:ICAN_UI_START_ROTATE;
        g_ican_curl_ui.heartbeat=1;g_ican_curl_ui.seq=1;
        for(unsigned t=0;t<2000;t+=10) {clock_ms+=10;g_ican_curl_ui.heartbeat++;group_step(clock_ms);}
        assert(group_phase==GROUP_FINISHED && local_fault);
        assert(enables[moving]==before);
        assert(!modes[4] && !modes[5] && !modes[6]);
    }
    testing_axis_actions=0;
    /* The same legacy curl planner is bound once to fresh start for all four elbow actions. */
    for(unsigned action=0;action<4;action++)
    for(unsigned pose=0;pose<3;pose++) {
        static const unsigned commands[]={ICAN_UI_START_CURL,ICAN_UI_START_SMALL,ICAN_UI_START_MEDIUM,ICAN_UI_START_PAUSE};
        static const float starts[]={-5.0f,7.5f,12.0f};
        rehab_curl_config_t config;
        assert(ican_fixed_action_config(ican_ui_action_opcode(commands[action]),&config));
        float origin=starts[pose],expected=fminf(origin+config.bottom_target_pos_rad-config.top_target_pos_rad,
                                               CONTROL_MOTOR_P_MAX_RAD-0.05f);
        memset(&guard,0,sizeof(guard));memset(&support,0,sizeof(support));
        memset(modes,0,sizeof(modes));memset((void *)&g_ican_curl_ui,0,sizeof(g_ican_curl_ui));
        group_phase=GROUP_IDLE;group_used=queued=cancelled=0;
        local_session=local_fault=ui_seen_seq=ui_heartbeat=ui_heartbeat_at=0;
        freeze5=feedback5_fault=feedback5_id=feedback5_mode_override=stale_idle4=stale_idle6=0;
        feedback_velocities[4]=feedback_velocities[5]=feedback_velocities[6]=0;
        positions[4]=6.103863239f;positions[5]=origin;positions[6]=-2.0f;
        unsigned before=enables[5],stopped=0,completed=0;float peak=origin;
        g_ican_curl_ui.heartbeat=1;g_ican_curl_ui.command=commands[action];g_ican_curl_ui.seq=1;
        for(unsigned elapsed=0;elapsed<120000;elapsed+=10) {
            clock_ms+=10;g_ican_curl_ui.heartbeat++;
            if(modes[5]==2) {
                float delta=targets[5]-positions[5];
                positions[5]+=delta>0.0012f ? 0.0012f:delta< -0.0012f ? -0.0012f:delta;
                if(positions[5]>peak) peak=positions[5];
            }
            if(!stopped && guard.completed_cycles>=3U) {
                stopped=1;completed=guard.completed_cycles;
                g_ican_curl_ui.command=ICAN_UI_STOP;g_ican_curl_ui.seq++;
            }
            group_step(clock_ms);
            if(guard.relative_curl==2U) {
                assert(fabsf(guard.curl.config.top_target_pos_rad-origin)<0.00001f);
                assert(fabsf(guard.curl.config.bottom_target_pos_rad-expected)<0.00001f);
            }
            assert(positions[4]==6.103863239f && positions[6]==-2.0f);
            if(group_phase==GROUP_FINISHED) break;
        }
        assert(stopped && completed>=3 && group_phase==GROUP_FINISHED && !local_fault);
        assert(enables[5]==before+1 && !modes[4] && !modes[5] && !modes[6]);
        assert(fabsf(peak-expected)<0.001f && fabsf(positions[5]-origin)<0.03f);
        assert(guard.curl.config.dwell_ms==config.dwell_ms);
    }
    puts("PASS all elbow actions: 3 shifted starts, protocol target clipping, 3-cycle return without drift, unchanged holds/dwell and STOP");
    puts("PASS LVGL lift-relative/rotate: four start positions including protocol clipping and stale idle cache, hold-first enable, 2-cycle repeat/return/STOP/drift/stale");
    return 0;
}
