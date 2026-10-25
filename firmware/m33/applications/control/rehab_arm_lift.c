#include "rehab_arm_lift.h"
#include <math.h>
#include <string.h>

static int config_valid(const rehab_arm_lift_config_t *c)
{
    return c && c->calibrated==1U && c->repeat<=1U &&
        isfinite(c->min_raw) && isfinite(c->max_raw) && isfinite(c->raised_raw) &&
        c->min_raw<c->max_raw && c->raised_raw>=c->min_raw && c->raised_raw<=c->max_raw &&
        isfinite(c->speed) && c->speed>0 && c->speed<=1.15f &&
        isfinite(c->accel) && c->accel>0 && c->accel<=0.60f &&
        isfinite(c->jerk) && c->jerk>0 && c->jerk<=1.50f &&
        isfinite(c->feedback_speed) && c->feedback_speed>0 && c->feedback_speed<=1.60f &&
        isfinite(c->hold_error) && c->hold_error>0 && c->hold_error<=0.035f &&
        isfinite(c->tolerance) && c->tolerance>0 && c->tolerance<=0.03f &&
        c->arrival_samples>=5 && c->arrival_samples<=100 &&
        c->dwell_ms>=500 && c->dwell_ms<=5000 &&
        c->segment_timeout_ms>=1000 && c->segment_timeout_ms<=120000;
}
static int feedback_valid(const rehab_arm_lift_config_t *c,
                          const rehab_arm_lift_feedback_t *f,uint32_t now)
{
    unsigned i;
    if(!f) return LIFT_FEEDBACK;
    for(i=0;i<3;i++) {
        const rehab_arm_lift_axis_t *a=&f->axis[i];
        if(!a->valid || a->fault || (uint32_t)(now-a->timestamp_ms)>=250U ||
           !isfinite(a->position) || !isfinite(a->velocity)) return LIFT_FEEDBACK;
        if(fabsf(a->velocity)>c->feedback_speed) return LIFT_LIMIT;
    }
    if(f->axis[0].position<c->min_raw || f->axis[0].position>c->max_raw) return LIFT_LIMIT;
    return 0;
}
static int plan(rehab_arm_lift_t *g,float from,float to,uint32_t now,unsigned state)
{
    if(rehab_scurve_plan(&g->segment,from,to,g->config.speed,g->config.accel,g->config.jerk))
        return LIFT_INVALID;
    /* 为到位确认和滞后预留至少1秒；拒绝理论上必然超时的配置。 */
    if(g->segment.duration_ms+1000U>=g->config.segment_timeout_ms) return LIFT_INVALID;
    g->target=to;g->phase_at=now;g->state=state;g->samples=0;
    return 0;
}
int rehab_arm_lift_start(rehab_arm_lift_t *g,const rehab_arm_lift_config_t *c,
                        const rehab_arm_lift_feedback_t *f,uint32_t now)
{
    rehab_arm_lift_t next;
    unsigned i;
    int r;
    if(!g || !config_valid(c)) return LIFT_INVALID;
    if(g->state!=LIFT_IDLE && g->state!=LIFT_DONE) return LIFT_INVALID;
    r=feedback_valid(c,f,now);if(r) return r;
    for(i=0;i<3;i++) if(fabsf(f->axis[i].velocity)>=0.10f) return LIFT_LIMIT;
    if(fabsf(f->axis[0].position-c->raised_raw)<=2*c->tolerance) return LIFT_INVALID;
    memset(&next,0,sizeof(next));next.config=*c;
    for(i=0;i<3;i++) {next.origin[i]=f->axis[i].position;next.last_tick[i]=f->axis[i].timestamp_ms;}
    next.last_arrival_tick=f->axis[0].timestamp_ms;
    r=plan(&next,next.origin[0],c->raised_raw,now,LIFT_RAISE);
    if(r) return r;
    *g=next;return 0;
}
void rehab_arm_lift_step(rehab_arm_lift_t *g,const rehab_arm_lift_feedback_t *f,
                        uint32_t now,unsigned stop_requested,rehab_arm_lift_output_t *out)
{
    int r=0;
    unsigned i;
    rehab_scurve_sample_t sample;
    if(!out) return;
    memset(out,0,sizeof(*out));
    if(!g) {out->stop_all=1;out->result=LIFT_INVALID;return;}
    if(g->state==LIFT_IDLE) {out->state=g->state;return;}
    if(g->state==LIFT_DONE || g->state==LIFT_FAULT) goto output;
    if(stop_requested) {r=LIFT_CANCELLED;goto fail;}
    r=feedback_valid(&g->config,f,now);if(r) goto fail;
    for(i=0;i<3;i++) {
        if((int32_t)(f->axis[i].timestamp_ms-g->last_tick[i])<0) {r=LIFT_FEEDBACK;goto fail;}
        g->last_tick[i]=f->axis[i].timestamp_ms;
        if(i && fabsf(f->axis[i].position-g->origin[i])>g->config.hold_error) {
            r=LIFT_LIMIT;goto fail;
        }
    }
    if(g->state==LIFT_RAISE || g->state==LIFT_RETURN) {
        uint32_t elapsed=now-g->phase_at;
        if(elapsed>=g->config.segment_timeout_ms) {r=LIFT_TIMEOUT;goto fail;}
        if(rehab_scurve_sample(&g->segment,elapsed,&sample) || !isfinite(sample.position_rad)) {
            r=LIFT_INVALID;goto fail;
        }
        out->write_targets=1;out->target_raw[0]=sample.position_rad;
        if(f->axis[0].timestamp_ms!=g->last_arrival_tick) {
            g->last_arrival_tick=f->axis[0].timestamp_ms;
            if(elapsed>=g->segment.duration_ms &&
               fabsf(f->axis[0].position-g->target)<=g->config.tolerance)
                g->samples++;
            else g->samples=0;
        }
        if(g->samples>=g->config.arrival_samples) {
            g->state=g->state==LIFT_RAISE ? LIFT_DWELL_HIGH:LIFT_DWELL_HOME;
            g->phase_at=now;
        }
    } else {
        if(fabsf(f->axis[0].position-g->target)>g->config.tolerance) {r=LIFT_LIMIT;goto fail;}
        out->write_targets=1;out->target_raw[0]=g->target;
        if((uint32_t)(now-g->phase_at)>=g->config.dwell_ms) {
            if(g->state==LIFT_DWELL_HOME) {
                g->cycles++;
                if(!g->config.repeat) {g->state=LIFT_DONE;goto output;}
                r=plan(g,f->axis[0].position,g->config.raised_raw,now,LIFT_RAISE);
            } else r=plan(g,f->axis[0].position,g->origin[0],now,LIFT_RETURN);
            if(r) goto fail;
        }
    }
    out->target_raw[1]=g->origin[1];out->target_raw[2]=g->origin[2];
    goto output;
fail:
    g->result=r;g->state=LIFT_FAULT;
output:
    out->state=g->state;out->cycles=g->cycles;out->result=g->result;
    if(g->state==LIFT_DONE || g->state==LIFT_FAULT) {out->write_targets=0;out->stop_all=1;}
}

int rehab_arm_lift_session_start(rehab_arm_lift_session_t *s,
                                const rehab_arm_lift_config_t *c,uint32_t now)
{
    if(!s || !config_valid(c) || s->active) return LIFT_INVALID;
    /* 上次失败不能靠再次启动清除；须控制层显式重新验收/初始化。 */
    if(s->result && s->result!=LIFT_CANCELLED) return LIFT_INVALID;
    if(s->stop_result) return LIFT_INVALID;
    memset(s,0,sizeof(*s));s->config=*c;s->active=1;s->preparing=1;s->prepared_at=now;
    return 0;
}
static void session_stop(rehab_arm_lift_session_t *s,int result,uint32_t now)
{
    if(!s->stopping) {
        s->stopping=1;s->preparing=0;s->result=result;s->stop_at=now-50U;
    }
}
void rehab_arm_lift_session_step(rehab_arm_lift_session_t *s,
    const rehab_arm_lift_io_t *io,void *context,uint32_t now,unsigned stop_requested)
{
    rehab_arm_lift_feedback_t feedback;
    rehab_arm_lift_output_t out;
    int r;
    unsigned i;
    if(!s || !s->active) return;
    if(!io || !io->prepare || !io->feedback || !io->target_raw || !io->stop) {
        /* 无有效STOP适配器无法保证电机停止，明确报告失败，不称已停止。 */
        s->result=LIFT_INVALID;s->stop_result=LIFT_INVALID;s->active=0;return;
    }
    if(stop_requested) session_stop(s,LIFT_CANCELLED,now);
    if(!s->stopping && s->preparing) {
        if((uint32_t)(now-s->prepared_at)>=10000U) session_stop(s,LIFT_TIMEOUT,now);
        else {
            r=io->prepare(context);
            if(r<0) session_stop(s,LIFT_FEEDBACK,now);
            else if(r>0) {
                memset(&feedback,0,sizeof(feedback));
                r=io->feedback(context,&feedback);
                if(r) session_stop(s,LIFT_FEEDBACK,now);
                else {
                    r=rehab_arm_lift_start(&s->motion,&s->config,&feedback,now);
                    if(r) session_stop(s,r,now);
                    else s->preparing=0;
                }
            }
        }
    }
    if(!s->stopping && !s->preparing) {
        memset(&feedback,0,sizeof(feedback));
        if(io->feedback(context,&feedback)) session_stop(s,LIFT_FEEDBACK,now);
        else {
            rehab_arm_lift_step(&s->motion,&feedback,now,0,&out);
            if(out.stop_all) session_stop(s,out.result,now);
            else if(out.write_targets) {
                /* 先刷新保持轴，再发布4号抬臂目标；任一发送失败不再继续发布。 */
                static const unsigned order[3]={1U,2U,0U};
                for(i=0;i<3;i++) {
                    unsigned axis=order[i];
                    if(io->target_raw(context,axis+4U,out.target_raw[axis])) {
                        session_stop(s,LIFT_FEEDBACK,now);break;
                    }
                }
            }
        }
    }
    if(s->stopping && (uint32_t)(now-s->stop_at)>=50U) {
        s->stop_at=now;
        /* 三轴独立最多三次，某轴失败仍停止其他轴；STOP发送不等于物理静止。 */
        for(i=0;i<3;i++) if(!(s->stop_done&(1U<<i))) {
            r=io->stop(context,4U+i);s->stop_attempts[i]++;
            if(!r || s->stop_attempts[i]>=3U) {
                s->stop_done|=1U<<i;
                if(r && !s->stop_result) s->stop_result=r;
            }
        }
        if(s->stop_done==7U) {s->active=0;if(!s->result && s->stop_result) s->result=LIFT_FEEDBACK;}
    }
}
