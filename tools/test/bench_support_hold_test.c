/* 编译真实保持状态机，模拟两轴回复和故障；不接硬件、不复制状态转换。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "bench_support_hold.h"
#include "bench5_guard.h"
static unsigned calls,fail_at,request_joint,request_index;
static unsigned enabled[8],enable_count[8],written[8];
static float origin[8],position[8],target[8],reg[8][3],reply_value;
static unsigned fault_axis,stale_axis,moving_axis,nan_reply,wrong_readback;
static unsigned replay_kind,trace_index;
static const float recorded_velocity[5]={-0.055018172f,-0.048175011f,0.042997077f,0.005272212f,-0.000710647f};
static const float recorded_position_offset[5]={0,0,0.000114918f,0.000076771f,0.000076771f};
static int point(void) { return ++calls==fail_at ? -1:0; }
static int read_param(unsigned j,uint16_t index) {
    if(point()) return -1;
    request_joint=j;request_index=index;
    reply_value=index==0x7019 ? position[j]:index==0x701b ? (moving_axis==j ? 0.2f:0):
        index==0x7005 ? reg[j][0]:index==0x7018 ? reg[j][1]:index==0x7017 ? reg[j][2]:target[j];
    if (replay_kind && j==4) {
        unsigned sample=trace_index<5 ? trace_index:4;
        if (index==0x7019) {
            float offset=replay_kind==1 ? recorded_position_offset[sample]:
                replay_kind==3 ? (sample%2 ? 0.0015f:0):
                replay_kind==4 ? (sample%2 ? 0.0018f:0):0;
            reply_value=origin[j]+offset;
        } else if (index==0x701b) {
            reply_value=replay_kind==2 ? 0.04f:replay_kind==1 ? recorded_velocity[sample]:
                (sample%2 ? -0.08f:0.08f);
            trace_index++;
        }
    }
    return 0;
}
static int reply(unsigned j,uint16_t index,float *value) {
    assert(j==request_joint && index==request_index);
    if(point()) return -1;
    *value=nan_reply ? NAN:reply_value+(wrong_readback && index==0x7018 ? 0.1f:0);
    return 1;
}
static int write_param(unsigned j,uint16_t index,float value) {
    if(point()) return -1;
    assert(!enabled[4] && !enabled[6]);
    if(index==0x7016) { assert(value==origin[j]);target[j]=value;written[j]=1; }
    else reg[j][index==0x7005 ? 0:index==0x7018 ? 1:2]=value;
    return 0;
}
static int enable(unsigned j) {
    assert(written[4] && written[6]); /* 两轴参数都准备完才能使能任一轴。 */
    if(point()) return -1;
    enabled[j]=1;enable_count[j]++;return 0;
}
static int stop(unsigned j) { if(point()) return -1;enabled[j]=0;return 0; }
static int feedback(unsigned j,unsigned mode,uint32_t since,float *p,float *v) {
    (void)since;
    if(point() || fault_axis==j) return -1;
    if(stale_axis==j) return 0;
    assert(mode==(enabled[j] ? 2U:0U));
    *p=position[j];*v=moving_axis==j ? 0.4f:0;return 1;
}
static const bh_io_t io={read_param,reply,write_param,enable,stop,feedback};
static bh_guard_t g;
static uint32_t now;
static void reset(unsigned inject) {
    calls=0;fail_at=inject;fault_axis=stale_axis=moving_axis=nan_reply=wrong_readback=0;
    replay_kind=trace_index=0;
    memset(enabled,0,sizeof(enabled));memset(enable_count,0,sizeof(enable_count));
    memset(written,0,sizeof(written));memset(reg,0,sizeof(reg));
    origin[4]=position[4]=3.1f;origin[6]=position[6]=5.2f;
    now=0;bh_start(&g,now);
}
static int prepare(void) {
    int r=0;
    for(;now<11000 && r==0;now+=10) r=bh_tick(&g,&io,now);
    assert(r!=0);return r;
}
int main(void) {
    unsigned baseline;
    reset(0);assert(prepare()==1);
    baseline=calls;
    assert(g.phase==BH_READY && enable_count[4]==1 && enable_count[6]==1);
    assert(target[4]==origin[4] && target[6]==origin[6]);
    for(unsigned fail=1;fail<=baseline;fail++) {
        reset(fail);assert(prepare()<0);assert(g.phase==BH_FAULT);
    }
    for(unsigned joint=4;joint<=6;joint+=2) {
        reset(0);moving_axis=joint;assert(prepare()==B5_LIMIT);
        assert(!enabled[4] && !enabled[6]);
        reset(0);assert(prepare()==1);position[joint]+=0.04f;
        assert(bh_tick(&g,&io,now)==B5_LIMIT && g.failed_joint==joint);
        assert(g.trip.reason==BH_TRIP_POSITION && g.trip.enabled_mask==3U);
        assert(g.trip.position==position[joint] && g.trip.origin==origin[joint]);
        bh_trip_t first=g.trip;
        position[joint]=origin[joint]; moving_axis=joint;
        assert(bh_tick(&g,&io,now+10)==B5_LIMIT);
        assert(memcmp(&first,&g.trip,sizeof(first))==0); /* 后续反馈不能覆盖首次触发。 */
        reset(0);assert(prepare()==1);moving_axis=joint;
        assert(bh_tick(&g,&io,now)==B5_LIMIT);
        assert(g.trip.reason==BH_TRIP_SPEED && g.trip.velocity==0.4f);
        reset(0);assert(prepare()==1);moving_axis=joint;position[joint]+=0.04f;
        assert(bh_tick(&g,&io,now)==B5_LIMIT);
        assert(g.trip.reason==(BH_TRIP_POSITION|BH_TRIP_SPEED));
        reset(0);assert(prepare()==1);fault_axis=joint;
        assert(bh_tick(&g,&io,now)==B5_FEEDBACK);
        assert(g.trip.reason==BH_TRIP_FEEDBACK);
        reset(0);assert(prepare()==1);stale_axis=joint;now+=300;
        assert(bh_tick(&g,&io,now)==B5_FEEDBACK);
        assert(g.trip.reason==BH_TRIP_STALE);
        reset(0);assert(prepare()==1);position[joint]=NAN;
        assert(bh_tick(&g,&io,now)==B5_LIMIT);
        assert(g.trip.reason==BH_TRIP_NONFINITE);
        /* 第一轴已使能时，第二轴准备失败仍必须返回故障。 */
        reset(0);
        while(g.phase!=BH_ENABLE) { assert(bh_tick(&g,&io,now)>=0);now+=10; }
        position[joint]+=0.01f;assert(prepare()==B5_LIMIT);
    }
    reset(0);nan_reply=1;assert(prepare()==B5_FEEDBACK);
    reset(0);wrong_readback=1;assert(prepare()==B5_FEEDBACK);
    reset(0);stale_axis=4;assert(prepare()==B5_TIMEOUT);
    reset(0);assert(prepare()==1);now=55000;
    assert(bh_tick(&g,&io,now)==B5_TIMEOUT);
    reset(0);replay_kind=1;assert(prepare()==1);
    assert(enable_count[4]==1 && enable_count[6]==1);
    reset(0);replay_kind=2;assert(prepare()==B5_LIMIT);
    assert(!enabled[4] && !enabled[6]);
    reset(0);replay_kind=3;assert(prepare()==B5_LIMIT);
    assert(g.sample_travel>0.0028f && g.failure_phase==BH_SAMPLE_POS && g.samples==2);
    /* 两段各0.0015rad均在逐段量化裕量内，80ms累计0.003rad已超限。 */
    reset(0);replay_kind=4;assert(prepare()==B5_LIMIT);
    assert(!enabled[4] && !enabled[6]);
    puts("PASS recorded axis4 stationary noise/position replay; sustained speed/oscillatory travel/position jump still reject before enable");
    /* uint32回绕仍能完成准备，不能因绝对时钟大小错误超时。 */
    reset(0);now=UINT32_MAX-800;bh_start(&g,now);
    for(unsigned i=0;i<400 && g.phase!=BH_READY;i++,now+=10)
        assert(bh_tick(&g,&io,now)>=0);
    assert(g.phase==BH_READY);
    reset(0);assert(g.trip.reason==0 && g.trip.enabled_mask==0);
    puts("PASS first hold-trip reason/position/speed/mask latching, nonfinite/stale/error and restart clearing");
    printf("PASS support hold ordering/raw targets/all %u IO boundaries/drift/speed/fault/stale/NaN/readback/timeout/wrap\n",baseline);
    return 0;
}
