/* Exercise the actual owner with the same CAN stubs as fixed-action regression. */
#define main fixed_action_regression
#include "bench5_owner_test.c"
#undef main

static void fresh_ui(void)
{
    memset(&guard,0,sizeof(guard));memset(&support,0,sizeof(support));
    memset(modes,0,sizeof(modes));memset(feedback_velocities,0,sizeof(feedback_velocities));
    memset((void *)&g_ican_curl_ui,0,sizeof(g_ican_curl_ui));
    group_phase=GROUP_IDLE;group_used=queued=cancelled=0;
    local_session=local_fault=ui_seen_seq=ui_heartbeat=ui_heartbeat_at=0;
    freeze5=feedback5_fault=feedback5_id=feedback5_mode_override=stale_idle4=stale_idle6=0;
    parameter_read_failure=0;testing_axis_actions=9;self=owner;
    positions[4]=2.0f;positions[5]=3.0f;positions[6]=4.0f;
}
static void click(unsigned command)
{
    g_ican_curl_ui.command=command;g_ican_curl_ui.seq++;g_ican_curl_ui.heartbeat++;
}
static void step(void)
{
    clock_ms+=10;g_ican_curl_ui.heartbeat++;group_step(clock_ms);
}
static unsigned total_enables(void) {return enables[4]+enables[5]+enables[6];}
static unsigned total_targets(void) {return target_writes[4]+target_writes[5]+target_writes[6];}
static void finish_stop(void)
{
    for(unsigned i=0;i<100 && group_phase!=GROUP_FINISHED;i++) step();
    assert(group_phase==GROUP_FINISHED && !modes[4] && !modes[5] && !modes[6]);
}
#include "bench_resist_owner_cases.h"
int main(void)
{
    assert(fixed_action_regression()==0);
    fresh_ui();record_valid=0;rehab_trajectory_bank_clear(0);
    unsigned before=total_enables();click(ICAN_UI_PLAY);step();
    assert((int32_t)g_ican_curl_ui.result==ICAN_UI_NO_RECORD && total_enables()==before);

    fresh_ui();unsigned writes=total_targets();before=total_enables();
    click(ICAN_UI_RECORD);
    for(unsigned i=0;i<400;i++) {
        if(group_phase==GROUP_RECORD) {
            float direction=i<200 ? 1.0f:-0.5f;
            positions[4]-=0.003f*direction;positions[5]+=0.0006f*direction;positions[6]+=0.0009f*direction;
        }
        step();
        assert(total_enables()==before && total_targets()==writes);
    }
    click(ICAN_UI_STOP);finish_stop();
    assert(record_valid && !local_fault && rehab_trajectory_bank_count(0)>20U);
    float first[3],last[3],expected[3];uint32_t dt;
    unsigned count=rehab_trajectory_bank_count(0);
    assert(teach_frame(0,first,&dt)==0 && dt==0);
    assert(teach_frame(count-1,last,&dt)==0 && dt>0);
    assert(last[0]<first[0] && last[1]>first[1] && last[2]>first[2]);
    puts("PASS three-axis teaching: complete timestamped frames, no ENABLE/targets, STOP seals RAM path");

    for(unsigned scenario=0;scenario<8;scenario++) {
        fresh_ui();positions[4]=-1.0f;positions[5]=5.0f;positions[6]=-2.0f;
        if(scenario==6) positions[5]=12.56f;
        for(unsigned a=0;a<3;a++) expected[a]=positions[a+4]+last[a]-first[a];
        unsigned injected=0,start_writes=total_targets(),before5=enables[5];
        click(ICAN_UI_PLAY);
        for(unsigned elapsed=0;elapsed<60000;elapsed+=10) {
            for(unsigned j=4;j<=6;j++) if(modes[j]==2) {
                float delta=targets[j]-positions[j],limit=teach_speed[j-4]*0.01f;
                positions[j]+=fminf(fmaxf(delta,-limit),limit);
            }
            if(!injected && group_phase==GROUP_PLAY_RUN && scenario>=1 && scenario<=5) {
                if(scenario==1) click(ICAN_UI_STOP);
                if(scenario==2) stale_idle4=1;
                if(scenario==3) feedback5_fault=1;
                if(scenario==4) {ui_heartbeat_at=clock_ms-1600U;ui_heartbeat=g_ican_curl_ui.heartbeat+1;}
                if(scenario==5) positions[6]+=0.5f;
                injected=1;
            }
            if(!injected && scenario==7 && group_phase==GROUP_PLAY_PREPARE && play_prepare_axis==1U) {positions[4]+=0.10f;injected=1;}
            step();
            if(group_phase==GROUP_FINISHED) break;
        }
        assert(group_phase==GROUP_FINISHED && !modes[4] && !modes[5] && !modes[6]);
        if(scenario==0) {
            assert(!guard.result && !local_fault && guard.completed_cycles==1U && record_valid);
            for(unsigned i=0;i<3;i++) assert(fabsf(positions[i+4]-expected[i])<0.031f);
            assert(total_targets()>start_writes+30U);
        } else if(scenario==1) assert(injected && guard.result==B5_CANCELLED && !local_fault);
        else {assert(local_fault && guard.result<0);if(scenario==6) assert(enables[5]==before5);}
    }
    puts("PASS three-axis relative playback: shifted starts, synchronized targets, completion/all STOP, cancel, stale/fault/heartbeat/drift/range and preparation drift");

    for(unsigned scenario=0;scenario<4;scenario++) {
        fresh_ui();before=total_enables();writes=total_targets();click(ICAN_UI_RECORD);
        unsigned injected=0;
        for(unsigned i=0;i<7000;i++) {
            if(!injected && group_phase==GROUP_RECORD && rehab_trajectory_bank_count(0)>=2U && scenario<3) {
                if(scenario==0) parameter_read_failure=6;
                if(scenario==1) positions[4]+=5.0f;
                if(scenario==2) {ui_heartbeat_at=clock_ms-1600U;ui_heartbeat=g_ican_curl_ui.heartbeat+1;}
                injected=1;
            }
            step();if(group_phase==GROUP_FINISHED) break;
        }
        assert(group_phase==GROUP_FINISHED && total_enables()==before && total_targets()==writes);
        if(scenario==3) assert(record_valid && !guard.result && rehab_trajectory_bank_count(0)==512U);
        else {
            assert(injected && !record_valid && local_fault);
            if(scenario==1) assert(support.failed_joint==4U && support.trip.reason==BH_TRIP_SPEED &&
                support.trip.enabled_mask==0 && guard.last_index==0x7019U);
        }
    }
    /* Reproduce the field failure: 6 moved 0.065194249rad over 103ms
     * (0.633rad/s), a normal manual gesture that exceeded the old 0.35 gate.
     * Exercise all three joints above the former capture gate, then verify
     * powered playback remains synchronized at its unchanged speed limits. */
    fresh_ui();before=total_enables();writes=total_targets();click(ICAN_UI_RECORD);
    for(unsigned i=0;i<140;i++) {
        if(group_phase==GROUP_RECORD) {
            positions[4]-=CONTROL_MOTOR_JOINT4_GEAR_RATIO*0.633f*0.01f;
            positions[5]+=0.633f*0.01f;positions[6]+=0.633f*0.01f;
        }
        step();assert(!local_fault && total_enables()==before && total_targets()==writes);
    }
    click(ICAN_UI_STOP);finish_stop();assert(record_valid);
    count=rehab_trajectory_bank_count(0);
    assert(teach_frame(0,first,&dt)==0 && teach_frame(count-1,last,&dt)==0);
    fresh_ui();
    for(unsigned i=0;i<3;i++) expected[i]=positions[i+4]+last[i]-first[i];
    click(ICAN_UI_PLAY);
    for(unsigned elapsed=0;elapsed<60000;elapsed+=10) {
        for(unsigned j=4;j<=6;j++) if(modes[j]==2) {
            float delta=targets[j]-positions[j],limit=teach_speed[j-4]*0.01f;
            positions[j]+=fminf(fmaxf(delta,-limit),limit);
        }
        step();
        if(group_phase==GROUP_PLAY_RUN) {
            assert(play_duration>=50U);
            for(unsigned i=0;i<3;i++)
                assert(fabsf(play_to[i]-play_from[i])*1000.0f/play_duration<=teach_speed[i]+0.00001f);
        }
        if(group_phase==GROUP_FINISHED) break;
    }
    assert(group_phase==GROUP_FINISHED && !local_fault && !guard.result && guard.completed_cycles==1U);
    for(unsigned i=0;i<3;i++) assert(fabsf(positions[i+4]-expected[i])<0.031f);
    puts("PASS field 0.633rad/s manual gesture on all axes (joint4 ratio included), no capture drive; synchronized playback remains at accepted speeds and completes");
    fresh_ui();click(ICAN_UI_RECORD);
    for(unsigned i=0;i<20;i++) {step();if(group_phase==GROUP_RECORD && rehab_trajectory_bank_count(0)==0U) break;}
    click(ICAN_UI_STOP);finish_stop();assert(!record_valid);
    puts("PASS teaching IO/overspeed/heartbeat faults discard partial trajectory, empty STOP rejected, capacity seals 512 frames without motor enable");
    resistance_regression();
    return 0;
}
