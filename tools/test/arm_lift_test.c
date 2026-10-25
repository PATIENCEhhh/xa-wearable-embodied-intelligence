#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "control/rehab_arm_lift.h"

static rehab_arm_lift_config_t config(void)
{
    /* 仅为仿真夹具，不是本台机械臂的已标定目标。 */
    rehab_arm_lift_config_t c={1,0,0.5f,1.5f,0.95f,0.12f,0.20f,0.50f,
        0.35f,0.035f,0.01f,5,500,15000};
    return c;
}
static rehab_arm_lift_feedback_t feedback(uint32_t now)
{
    rehab_arm_lift_feedback_t f={0};
    for(unsigned i=0;i<3;i++) f.axis[i]=(rehab_arm_lift_axis_t){1,0,now,1.15f+i,0};
    return f;
}
/* 模拟CAN适配器：验证发送顺序、部分发送失败和逐轴STOP重试。 */
typedef struct {
    rehab_arm_lift_feedback_t f;
    int ready, feedback_error, target_fail, stop_fail;
    unsigned writes, stops[3];
} fake_io_t;
static int prepare(void *p) {return ((fake_io_t *)p)->ready;}
static int read_feedback(void *p,rehab_arm_lift_feedback_t *f)
{
    fake_io_t *x=p;*f=x->f;return x->feedback_error;
}
static int write_target(void *p,unsigned id,float raw)
{
    fake_io_t *x=p;
    static const unsigned order[3]={5,6,4};
    assert(id==order[x->writes%3]);x->writes++;
    if((int)id==x->target_fail) return -1;
    x->f.axis[id-4].position=raw;return 0;
}
static int stop_axis(void *p,unsigned id)
{
    fake_io_t *x=p;assert(id>=4 && id<=6);x->stops[id-4]++;
    return (int)id==x->stop_fail ? -7:0;
}
static void test_session(void)
{
    const rehab_arm_lift_io_t io={prepare,read_feedback,write_target,stop_axis};
    for(unsigned scenario=0;scenario<9;scenario++) {
        rehab_arm_lift_session_t s={0};
        rehab_arm_lift_config_t c=config();
        fake_io_t x={0};x.f=feedback(100);x.ready=1;
        if(scenario==1) x.ready=0; /* 准备超时。 */
        if(scenario==2) x.ready=-1;
        if(scenario==3) x.feedback_error=-1;
        if(scenario==4) x.target_fail=6;
        if(scenario==5) x.stop_fail=4;
        assert(!rehab_arm_lift_session_start(&s,&c,100));
        for(uint32_t t=100;t<40000 && s.active;t+=20) {
            for(unsigned i=0;i<3;i++) x.f.axis[i].timestamp_ms=t;
            /* 6:准备阶段停止；7:运行中停止；8:短暂准备等待后成功。 */
            if(scenario==6) x.ready=0;
            if(scenario==8) x.ready=t>=200;
            rehab_arm_lift_session_step(&s,&io,&x,t,
                scenario==6 || (scenario==7 && t>=700));
        }
        assert(!s.active && s.stop_done==7);
        assert(x.stops[1]==1 && x.stops[2]==1);
        assert(x.stops[0]==(scenario==5 ? 3U:1U));
        if(scenario==0 || scenario==8) assert(!s.result && s.motion.cycles==1);
        else if(scenario==1) assert(s.result==LIFT_TIMEOUT && !x.writes);
        else if(scenario==6 || scenario==7) assert(s.result==LIFT_CANCELLED);
        else assert(s.result==LIFT_FEEDBACK);
        if(scenario==4) assert(x.writes==2); /* 失败后不得发送4号目标。 */
        if(scenario==5) assert(s.stop_result==-7);
        unsigned writes=x.writes;
        rehab_arm_lift_session_step(&s,&io,&x,41000,0);
        assert(x.writes==writes); /* 结束后不会自行重启。 */
        int r=rehab_arm_lift_session_start(&s,&c,41000);
        assert((r==0)==(scenario==0 || scenario==6 || scenario==7 || scenario==8));
    }
}
int main(void)
{
    for(unsigned scenario=0;scenario<9;scenario++) {
        uint32_t now=scenario==8 ? UINT32_MAX-3000U:100;
        rehab_arm_lift_config_t c=config();
        rehab_arm_lift_feedback_t f=feedback(now);
        rehab_arm_lift_t g={0};
        rehab_arm_lift_output_t out;
        unsigned injected=0;
        float previous=f.axis[0].position;
        c.repeat=scenario==1;
        assert(!rehab_arm_lift_start(&g,&c,&f,now));
        for(unsigned elapsed=0;elapsed<40000;elapsed+=20) {
            now+=20;
            for(unsigned i=0;i<3;i++) f.axis[i].timestamp_ms=now;
            if(!injected && elapsed>=600) {
                injected=1;
                if(scenario==3) f.axis[1].position+=0.04f;
                if(scenario==4) f.axis[2].position=NAN;
                if(scenario==5) f.axis[0].fault=1;
                if(scenario==6) f.axis[0].velocity=0.36f;
            }
            if(scenario==2 && elapsed>=600) f.axis[0].timestamp_ms=now-250;
            rehab_arm_lift_step(&g,&f,now,scenario==1 && g.cycles>=3,&out);
            if(out.stop_all) break;
            if(out.write_targets) {
                assert(out.target_raw[0]>=c.min_raw && out.target_raw[0]<=c.max_raw);
                assert(out.target_raw[1]==2.15f && out.target_raw[2]==3.15f);
                assert(fabsf(out.target_raw[0]-previous)<=0.12f*0.02f+0.00001f);
                if(scenario!=7) f.axis[0].position=out.target_raw[0];
                previous=out.target_raw[0];
            }
        }
        assert(out.stop_all && !out.write_targets);
        if(scenario==0 || scenario==8) {
            assert(g.state==LIFT_DONE && g.cycles==1 && !g.result);
            assert(fabsf(f.axis[0].position-1.15f)<=c.tolerance);
        } else if(scenario==1) assert(g.cycles==3 && g.result==LIFT_CANCELLED);
        else if(scenario==2 || scenario==4 || scenario==5) assert(g.result==LIFT_FEEDBACK);
        else if(scenario==7) assert(g.result==LIFT_TIMEOUT);
        else assert(g.result==LIFT_LIMIT);
    }
    {
        rehab_arm_lift_config_t c=config();
        rehab_arm_lift_feedback_t f=feedback(100);
        rehab_arm_lift_t g={0};rehab_arm_lift_output_t out;
        c.calibrated=0;assert(rehab_arm_lift_start(&g,&c,&f,100)==LIFT_INVALID);
        c=config();c.raised_raw=NAN;assert(rehab_arm_lift_start(&g,&c,&f,100)==LIFT_INVALID);
        c=config();assert(!rehab_arm_lift_start(&g,&c,&f,100));
        /* 缓存重复不能累计到位；超过250ms即退出。 */
        f.axis[0].position=c.raised_raw;
        for(uint32_t t=120;t<350;t+=20) rehab_arm_lift_step(&g,&f,t,0,&out);
        assert(g.samples==0);
        rehab_arm_lift_step(&g,&f,350,0,&out);
        assert(g.result==LIFT_FEEDBACK && out.stop_all);
    }
    test_session();
    puts("PASS lift: S-curve round-trip/repeat, feedback guards, IO failures, prepare timeout, independent STOP retries, restart lock");
    return 0;
}
