#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "bench5_guard.h"
#include "control/control_layer_cfg.h"

static float params[0x30],pos,vel;
static unsigned calls,enables,stops,writes;
static int fail_at,drop,wrong,nan_reply,stop_fail,status_fail,status_drop;
static float hold_error;
static uint16_t last_index;
static b5_guard_t g;
static b5_profile_t profile;
static int io_fail(void) { return ++calls==(unsigned)fail_at; }
static int rd(uint16_t i) { last_index=i; return io_fail() ? -1:0; }
static int rp(uint16_t i,uint32_t after,float *v) {
    (void)after;
    assert(i==last_index);
    if (drop) return 0;
    *v=i==0x7019 ? pos:i==0x701b ? vel:params[i-0x7000];
    if (i==0x7016) *v+=hold_error;
    if (wrong && i==0x7018) *v+=0.1f;
    if (nan_reply) *v=NAN;
    return 1;
}
static int wr(uint16_t i,float v) {
    if (io_fail()) return -1;
    writes++; params[i-0x7000]=v; return 0;
}
static int en(void) {
    assert(params[5]==5 && params[0x18]>0 && params[0x18]<=(g.legacy_curl ? 2.0f:0.5f));
    assert(fabsf(params[0x17]-(g.legacy_curl ? 0.12f:0.01f))<0.00001f);
    assert(fabsf(params[0x16]-pos)<0.0021f);
    assert(stops>0); /* 初始STOP及所有参数读回在使能前完成。 */
    if (io_fail()) return -1;
    enables++; return 0;
}
static int st(void) { stops++; return io_fail() || stop_fail ? -1:0; }
static int status(unsigned mode) {
    assert(mode==0 || mode==2);
    if (io_fail() || status_fail) return B5_FEEDBACK;
    return status_drop ? 0:1;
}
static const b5_io_t io={rd,rp,wr,en,st,status,NULL};
static void reset(void) {
    memset(&g,0,sizeof(g)); memset(params,0,sizeof(params));
    profile=(b5_profile_t){0,1,0.2f};
    calls=enables=stops=writes=0; fail_at=drop=wrong=nan_reply=stop_fail=status_fail=status_drop=0;
    pos=5.0f; vel=0;
    hold_error=0;
}
static uint32_t run(uint32_t start,int follow) {
    uint32_t elapsed;
    for(elapsed=0;elapsed<20000 && b5_active(&g);elapsed+=10) {
        if(follow && (g.state==B5_MONITOR_POS || g.state==B5_MONITOR_VEL)) {
            float delta=g.target-pos;
            pos+=fmaxf(-0.0001f,fminf(0.0001f,delta));
        }
        b5_tick(&g,&profile,&io,start+elapsed);
    }
    assert(!b5_active(&g)); return elapsed;
}
static void advance(unsigned state) {
    uint32_t t=0;
    while(g.state!=state && t<1000) {
        b5_tick(&g,&profile,&io,t); t+=10;
    }
    assert(g.state==state);
}
static const rehab_curl_config_t legacy_config={
    CONTROL_REHAB_CURL_HARD_MIN_RAW_RAD,CONTROL_REHAB_CURL_HARD_MAX_RAW_RAD,
    CONTROL_REHAB_CURL_TOP_TARGET_RAW_RAD,CONTROL_REHAB_CURL_BOTTOM_TARGET_RAW_RAD,
    CONTROL_REHAB_CURL_POSITION_TOLERANCE_RAD,CONTROL_REHAB_CURL_MAX_FEEDBACK_SPEED_RAD_S,
    CONTROL_REHAB_CURL_DWELL_MS,CONTROL_REHAB_CURL_SEGMENT_TIMEOUT_MS,
    CONTROL_REHAB_CURL_COMMAND_REFRESH_MS,CONTROL_REHAB_CURL_ARRIVAL_SAMPLES
};
static void reset_legacy(void) {
    reset(); profile.current_a=2.0f;
    pos=5.139257431f;
}
static void run_legacy(uint32_t start,int follow) {
    for(unsigned t=0;t<47000 && b5_active(&g);t+=10) {
        if(follow && (g.state==B5_MONITOR_POS || g.state==B5_MONITOR_VEL)) {
            float delta=g.target-pos;
            pos+=fmaxf(-0.0012f,fminf(0.0012f,delta));
        }
        b5_tick(&g,&profile,&io,start+t);
        if(g.curl.phase==REHAB_CURL_PHASE_DWELL_BOTTOM)
            assert(!g.curl.completed_repetitions && g.state!=B5_DONE);
    }
    assert(!b5_active(&g));
}
static void test_legacy(void) {
    unsigned legacy_calls;
    reset_legacy(); assert(b5_start_curl(&g,&profile,&legacy_config,0)==0);
    advance(B5_TARGET);
    assert(g.curl.first_move_phase==REHAB_CURL_PHASE_MOVE_BOTTOM);
    assert(fabsf(g.target-legacy_config.bottom_target_pos_rad)<0.00001f);
    reset_legacy(); vel=-0.07522015f;
    assert(b5_start_curl(&g,&profile,&legacy_config,0)==0);
    run_legacy(0,1);
    assert(g.state==B5_DONE && enables==1 && g.curl.completed_repetitions==1);
    assert(fabsf(g.target-legacy_config.top_target_pos_rad)<0.00001f);
    assert(pos>=legacy_config.hard_min_pos_rad && pos<=legacy_config.hard_max_pos_rad);
    assert(stops==2 && g.target_attempts>2);
    legacy_calls=calls;
    assert(b5_start_curl(&g,&profile,&legacy_config,46000)==B5_BUSY);
    for(unsigned failure=1;failure<=legacy_calls;failure++) {
        reset_legacy(); fail_at=(int)failure;
        assert(b5_start_curl(&g,&profile,&legacy_config,0)==0);
        run_legacy(0,1);
        assert(stops>0 && (g.state==B5_FAULT || g.state==B5_DONE));
        if(g.state==B5_DONE) assert(g.attempts>1);
    }
    reset_legacy(); pos=4.95f;
    assert(b5_start_curl(&g,&profile,&legacy_config,0)==0); run_legacy(0,0);
    assert(g.first_error==B5_LIMIT && enables==0);
    reset_legacy(); vel=0.351f;
    assert(b5_start_curl(&g,&profile,&legacy_config,0)==0); run_legacy(0,0);
    assert(g.first_error==B5_LIMIT && enables==0);
    reset_legacy(); assert(b5_start_curl(&g,&profile,&legacy_config,0)==0);
    advance(B5_MONITOR_POS); vel=0.351f; run_legacy(g.phase_at+10,1);
    assert(g.first_error==B5_LIMIT);
    reset_legacy(); assert(b5_start_curl(&g,&profile,&legacy_config,0)==0);
    advance(B5_MONITOR_POS); status_drop=1; run_legacy(g.phase_at+10,1);
    assert(g.first_error==B5_FEEDBACK);
    reset_legacy(); assert(b5_start_curl(&g,&profile,&legacy_config,0)==0);
    advance(B5_MONITOR_POS); drop=1; run_legacy(g.phase_at+10,1);
    assert(g.first_error==B5_TIMEOUT);
    reset_legacy(); assert(b5_start_curl(&g,&profile,&legacy_config,0)==0);
    run_legacy(0,0); assert(g.first_error==B5_TIMEOUT && enables==1);
    reset_legacy(); assert(b5_start_curl(&g,&profile,&legacy_config,0)==0);
    advance(B5_MONITOR_POS); b5_cancel(&g,g.phase_at+10); run_legacy(g.phase_at,0);
    assert(g.result==B5_CANCELLED);
    reset_legacy(); profile.current_a=2.01f;
    assert(b5_start_curl(&g,&profile,&legacy_config,0)==B5_LOCKED && !calls);
    puts("PASS legacy calibrated planner: one cycle, actual noise, every I/O failure, bounds/speed/stale/drop/stall/cancel/current cap");
}
int main(void) {
    unsigned n,total_calls;
    reset(); profile.locked=1;
    assert(b5_start(&g,&profile,0)==B5_LOCKED && calls==0 && !g.used);
    for(n=0;n<5;n++) {
        reset();
        if(n==0) profile.direction=0;
        if(n==1) profile.current_a=0;
        if(n==2) profile.current_a=NAN;
        if(n==3) profile.current_a=0.51f;
        if(n==4) profile.direction=2;
        assert(b5_start(&g,&profile,0)==B5_LOCKED && calls==0);
    }
    puts("PASS locked/invalid profiles never touch motor I/O");
    reset(); assert(b5_start(&g,&profile,0)==0);
    assert(b5_start(&g,&profile,0)==B5_BUSY);
    run(0,1); assert(g.state==B5_DONE && enables==1 && stops==2);
    total_calls=calls;
    assert(fabsf(g.target-g.origin-0.0872664626f)<0.000001f);
    assert(b5_start(&g,&profile,20000)==B5_BUSY);
    puts("PASS preparation order/readback/one-shot completion");
    for(n=1;n<=total_calls;n++) {
        reset(); fail_at=(int)n; assert(b5_start(&g,&profile,0)==0);
        run(0,1);
        assert(g.state==B5_FAULT || g.state==B5_DONE);
        assert(stops>0);
        if(g.state==B5_DONE) assert(g.attempts>1); /* 终止STOP失败后重试成功。 */
    }
    puts("PASS failure injected at every read/write/enable/stop boundary");
    reset(); wrong=1; assert(b5_start(&g,&profile,0)==0);
    run(0,0); assert(g.result==B5_FEEDBACK && enables==0 && stops>=2);
    reset(); drop=1; assert(b5_start(&g,&profile,0)==0);
    run(0,0); assert(g.result==B5_TIMEOUT && enables==0);
    reset(); nan_reply=1; assert(b5_start(&g,&profile,0)==0);
    run(0,0); assert(g.result==B5_FEEDBACK && enables==0);
    reset(); assert(b5_start(&g,&profile,0)==0);
    b5_tick(&g,&profile,&io,0); b5_tick(&g,&profile,&io,250);
    run(260,0); assert(g.result==B5_TIMEOUT && enables==0);
    reset(); vel=0.1f; assert(b5_start(&g,&profile,0)==0);
    run(0,0); assert(g.result==B5_LIMIT && enables==0);
    puts("PASS wrong readback/missing or nonfinite feedback/preflight speed");
    reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_ENABLE);
    pos+=0.01f; run(g.phase_at+10,0); assert(g.result==B5_LIMIT && enables==0);
    reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_MONITOR_POS);
    pos-=0.01f; run(g.phase_at+10,0); assert(g.result==B5_LIMIT);
    reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_MONITOR_POS);
    pos+=0.02f; run(g.phase_at+10,0); assert(g.result==B5_LIMIT);
    reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_MONITOR_POS);
    vel=0.1f; run(g.phase_at+10,0); assert(g.result==B5_LIMIT);
    reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_MONITOR_POS);
    drop=1; run(g.phase_at+10,0); assert(g.result==B5_TIMEOUT);
    puts("PASS pre-enable drift/reverse/overtravel/overspeed/feedback loss");
    reset(); assert(b5_start(&g,&profile,0)==0); run(0,0);
    assert(g.result==B5_NO_PROGRESS && stops>=2 && g.enable_attempts==1);
    reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_MONITOR_POS);
    b5_tick(&g,&profile,&io,g.enabled_at+12000U); run(g.phase_at,0);
    assert(g.result==B5_TIMEOUT && stops>=2);
    reset(); assert(b5_start(&g,&profile,0)==0);
    b5_tick(&g,&profile,&io,15000); run(15010,0);
    assert(g.result==B5_TIMEOUT && enables==0);
    reset(); profile.direction=-1;
    assert(b5_start(&g,&profile,0xfffffff0U)==0); run(0xfffffff0U,1);
    assert(g.state==B5_DONE && g.target<g.origin);
    puts("PASS enable/total deadlines/clock wrap/negative flexion direction");
    reset(); b5_cancel(&g,0); run(0,0);
    assert(g.result==B5_CANCELLED && stops==1 && enables==0);
    reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_MONITOR_POS);
    b5_cancel(&g,g.phase_at+10); run(g.phase_at,0); assert(g.result==B5_CANCELLED);
    reset(); stop_fail=1; b5_cancel(&g,0); run(0,0);
    assert(g.state==B5_FAULT && stops==3 && g.stop_result!=0);
    puts("PASS idle/active cancel and bounded stop retries (send != physical stop)");
    reset(); status_drop=1; assert(b5_start(&g,&profile,0)==0); run(0,0);
    assert(g.result==B5_TIMEOUT && enables==0);
    reset(); status_fail=1; assert(b5_start(&g,&profile,0)==0); run(0,0);
    assert(g.result==B5_FEEDBACK && enables==0);
    reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_VERIFY_ENABLE);
    status_fail=1; run(g.phase_at+10,0); assert(g.result==B5_FEEDBACK && enables==1);
    puts("PASS fresh disabled/running feedback mandatory; stale or fault aborts");
    reset(); assert(b5_start(&g,&profile,0)==0);
    for(n=0;n<2000 && b5_active(&g);n++) {
        static const float noise[5]={-0.058f,0.089f,-0.04f,0.002f,0.043f};
        vel=noise[g.samples%5];
        if(g.state==B5_MONITOR_POS || g.state==B5_MONITOR_VEL)
            pos+=fminf(0.0001f,g.target-pos);
        b5_tick(&g,&profile,&io,n*10);
    }
    assert(g.state==B5_DONE && enables==1);
    reset(); vel=0.04f; assert(b5_start(&g,&profile,0)==0); run(0,0);
    assert(g.result==B5_LIMIT && enables==0);
    reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_MONITOR_POS);
    pos+=0.004f; run(g.previous_at+10,0); assert(g.result==B5_LIMIT);
    puts("PASS observed signed noise tolerated; sustained speed and position jumps rejected");
    reset(); hold_error=-0.0001153946f;
    assert(b5_start(&g,&profile,0)==0); run(0,1);
    assert(g.state==B5_DONE && g.enable_attempts==1 && g.target_attempts==1);
    reset(); hold_error=0.00041f;
    assert(b5_start(&g,&profile,0)==0); run(0,0);
    assert(g.state==B5_FAULT && g.first_error==B5_FEEDBACK && g.failure_state==B5_HOLD);
    assert(g.enable_attempts==0 && g.target_attempts==0 && g.last_index==0x7016);
    b5_cancel(&g,3000); run(3000,0);
    assert(g.first_error==B5_FEEDBACK && g.failure_state==B5_HOLD && g.result==B5_CANCELLED);
    reset(); hold_error=-0.00041f;
    assert(b5_start(&g,&profile,0)==0); run(0,0);
    assert(g.first_error==B5_FEEDBACK && !g.enable_attempts && !g.target_attempts);
    puts("PASS observed tiny hold readback difference accepted; both larger errors reject before enable; first cause persists");
    /* 回放实际导致中位数偏置的五次速度；均值阈值仍0.03，位置保护独立生效。 */
    for(unsigned follow=0;follow<2;follow++) {
        static const float actual[5]={0.0402235836f,-0.0228798389f,-0.0679498911f,0.0692755356f,0.0470793657f};
        reset(); assert(b5_start(&g,&profile,0)==0);
        for(n=0;n<2000 && b5_active(&g);n++) {
            vel=(g.state==B5_MONITOR_POS || g.state==B5_MONITOR_VEL) ? actual[g.samples%5]:0;
            if(follow && (g.state==B5_MONITOR_POS || g.state==B5_MONITOR_VEL))
                pos+=fminf(0.0001f,g.target-pos);
            b5_tick(&g,&profile,&io,n*10);
        }
        assert(!b5_active(&g));
        assert(follow ? g.state==B5_DONE:g.first_error==B5_NO_PROGRESS);
    }
    puts("PASS recorded bipolar velocity replay: 5deg follows; stalled motor stops within progress deadline");
    /* 新一次实测五个负速度必须继续触发保护，不能为了可视效果放宽阈值。 */
    reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_MONITOR_POS);
    for(n=0;n<100 && b5_active(&g);n++) {
        static const float actual[5]={-0.07522015f,-0.06103474f,-0.06635690f,-0.06241166f,-0.01810261f};
        vel=actual[g.samples%5];
        b5_tick(&g,&profile,&io,g.enabled_at+20+n*10);
    }
    assert(g.first_error==B5_LIMIT && g.failure_state==B5_MONITOR_VEL);
    puts("PASS latest recorded negative velocity window still aborts; limits unchanged");
    /* 0.035rad/s单次差分落在量化裕量内，累计绝对路程仍须检出，往复不能抵消。 */
    for(unsigned reverse=0;reverse<2;reverse++) {
        reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_MONITOR_POS);
        uint32_t start=g.phase_at+10;
        for(n=0;n<100 && b5_active(&g);n++) {
            if(g.state==B5_MONITOR_POS || g.state==B5_MONITOR_VEL) {
                /* 仅在位置回复时推进40ms路程，每两次回复反向。
                 * 明确构造采样可见的往复，不宣称检测采样间抵消的运动。 */
                if(g.state==B5_MONITOR_POS && g.requested) {
                    float sign=reverse && (n/8)%2 ? -1.0f:1.0f;
                    pos+=sign*0.0014f;
                }
            }
            b5_tick(&g,&profile,&io,start+n*10);
        }
        assert(g.first_error==B5_LIMIT && g.enable_attempts==1);
    }
    reset(); assert(b5_start(&g,&profile,0)==0); advance(B5_MONITOR_POS);
    vel=0.1f; run(g.phase_at+10,0); assert(g.first_error==B5_LIMIT);
    puts("PASS cumulative one-way/oscillatory overspeed and instantaneous velocity ceiling");
    test_legacy();
    return 0;
}
