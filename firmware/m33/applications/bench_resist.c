#include "bench_resist.h"
#include "control/control_layer_cfg.h"
#include <math.h>
#include <string.h>

static void enter(br_guard_t *g,unsigned phase,uint32_t now)
{
    g->phase=phase;g->phase_at=now;g->requested=g->written=0;
}
static int fail(br_guard_t *g,const b5_io_t *io,int result)
{
    /* Best-effort zero before the owner performs bounded STOP on all three axes. */
    if(g->enabled) (void)io->write(0x7006,0.0f);
    g->current=0;g->failure_phase=g->phase;g->phase=BR_FAULT;g->result=result;
    rehab_resist_strategy_reset(&g->strategy);
    return result;
}
void br_start(br_guard_t *g,uint32_t now)
{
    memset(g,0,sizeof(*g));g->started=now;
    g->params.resist_direction=1.0f; /* Raw feedback and raw iq_ref have the same motor sign. */
    g->params.resist_max_current_a=CONTROL_REHAB_RESIST_LIMIT_CUR_A;
    g->params.resist_current_gain_a_per_rad_s=CONTROL_REHAB_RESIST_CURRENT_GAIN_A_PER_RAD_S;
    rehab_resist_strategy_reset(&g->strategy);enter(g,BR_STOP,now);
}
static int read_value(br_guard_t *g,const b5_io_t *io,uint16_t index,float *value,uint32_t now)
{
    if(!g->requested) {
        g->last_index=index;
        if(io->read(index)) return B5_IO;
        g->requested=1;g->request_at=now;return 0;
    }
    if((uint32_t)(now-g->request_at)>=250U) return B5_TIMEOUT;
    int r=io->reply(index,g->request_at,value);
    return r<0 ? B5_IO:r>0 && !isfinite(*value) ? B5_FEEDBACK:r;
}
static int checked(br_guard_t *g,const b5_io_t *io,uint16_t index,float value,uint32_t now)
{
    float actual=0;
    if(!g->written) {
        g->last_index=index;
        if(io->write(index,value)) return B5_IO;
        g->written=1;return 0;
    }
    int r=read_value(g,io,index,&actual,now);
    if(r>0 && fabsf(actual-value)>0.001f) return B5_FEEDBACK;
    return r;
}
int br_tick(br_guard_t *g,const b5_io_t *io,uint32_t now)
{
    int r=0;float value=0,position=0,velocity=0;
    if(g->phase==BR_FAULT) return g->result;
    if(g->phase==BR_IDLE) return B5_LOCKED;
    if(g->phase!=BR_RUN && (uint32_t)(now-g->started)>=10000U) return fail(g,io,B5_TIMEOUT);
    switch(g->phase) {
    case BR_STOP:
        if(io->stop()) return fail(g,io,B5_IO);
        enter(g,BR_VERIFY_STOP,now);break;
    case BR_VERIFY_STOP:
        r=io->status(0U);
        if(r<0 || (!r && (uint32_t)(now-g->phase_at)>=250U)) return fail(g,io,B5_FEEDBACK);
        if(r) enter(g,BR_POSITION,now);
        break;
    case BR_POSITION:
        r=read_value(g,io,0x7019,&value,now);
        if(r==1) {
            if(value<CONTROL_MOTOR_P_MIN_RAD || value>CONTROL_MOTOR_P_MAX_RAD ||
               (g->samples && fabsf(value-g->origin)>0.002f)) return fail(g,io,B5_LIMIT);
            if(!g->samples) g->origin=value;
            else g->travel+=fabsf(value-g->previous_position);
            if(g->travel>0.004f) return fail(g,io,B5_LIMIT);
            g->position=g->previous_position=value;enter(g,BR_VELOCITY,now);
        }
        break;
    case BR_VELOCITY:
        r=read_value(g,io,0x701b,&value,now);
        if(r==1) {
            if(fabsf(value)>0.10f) return fail(g,io,B5_LIMIT);
            g->velocity_sum+=fabsf(value);
            if(++g->samples>=5U) {
                if(g->velocity_sum/5.0f>0.03f) return fail(g,io,B5_LIMIT);
                enter(g,BR_MODE,now);
            } else enter(g,BR_POSITION,now);
        }
        break;
    case BR_MODE:
        r=checked(g,io,0x7005,3.0f,now);
        if(r==1) enter(g,BR_LIMIT,now);
        break;
    case BR_LIMIT:
        r=checked(g,io,0x7018,g->params.resist_max_current_a,now);
        if(r==1) enter(g,BR_ZERO,now);
        break;
    case BR_ZERO:
        r=checked(g,io,0x7006,0.0f,now);
        if(r==1) enter(g,BR_ENABLE,now);
        break;
    case BR_ENABLE:
        /* Check a new stopped sample after all parameter writes, before enabling current mode. */
        r=read_value(g,io,0x7019,&value,now);
        if(r==1) {
            if(fabsf(value-g->origin)>0.002f) return fail(g,io,B5_LIMIT);
            if(io->enable()) return fail(g,io,B5_IO);
            g->enabled=1;g->enabled_at=now;enter(g,BR_VERIFY_ENABLE,now);
        }
        break;
    case BR_VERIFY_ENABLE:
        r=io->feedback(g->enabled_at,&position,&velocity);
        if(r<0 || (!r && (uint32_t)(now-g->enabled_at)>=250U)) return fail(g,io,B5_FEEDBACK);
        if(r && (fabsf(position-g->origin)>0.035f || fabsf(velocity)>0.10f)) return fail(g,io,B5_LIMIT);
        if(r) {g->running_at=now;g->update_at=now;enter(g,BR_RUN,now);}
        break;
    case BR_RUN: {
        if((uint32_t)(now-g->running_at)>=120000U) {
            if(control_motor_current_setpoint(5U,0.0f)) return fail(g,io,B5_IO);
            g->current=0;return 2;
        }
        r=io->feedback(g->enabled_at,&position,&velocity);
        if(r<=0) return fail(g,io,B5_FEEDBACK);
        g->position=position;g->velocity=velocity;
        if(!isfinite(position) || !isfinite(velocity) || fabsf(velocity)>1.0f ||
           position<CONTROL_MOTOR_P_MIN_RAD || position>CONTROL_MOTOR_P_MAX_RAD ||
           position<g->origin-0.15f || position>g->origin+1.65f) return fail(g,io,B5_LIMIT);
        /* Drop the previous direction immediately, including between scheduled 20ms updates. */
        if(g->current*velocity>0) {
            rehab_resist_strategy_reset(&g->strategy);g->current=0;
            if(control_motor_current_setpoint(5U,0.0f)) return fail(g,io,B5_IO);
        }
        if((uint32_t)(now-g->update_at)<CONTROL_REHAB_SERVICE_PERIOD_MS) return 1;
        g->update_at=now;
        /* Old slew state must never produce assistance immediately after a measured reversal. */
        if(g->strategy.last_current_a*velocity>0) rehab_resist_strategy_reset(&g->strategy);
        control_motor_feedback_t feedback;rehab_strategy_output_t output;
        memset(&feedback,0,sizeof(feedback));memset(&output,0,sizeof(output));
        feedback.vel_rad_s=velocity;
        rehab_resist_strategy_step(&g->strategy,&g->params,&feedback,&output);
        g->current=output.type==REHAB_STRATEGY_OUTPUT_CURRENT ? output.current_a:0;
        if(!isfinite(g->current) || fabsf(g->current)>g->params.resist_max_current_a ||
           g->current*velocity>0) return fail(g,io,B5_LIMIT);
        if(control_motor_current_setpoint(5U,g->current)) return fail(g,io,B5_IO);
        return 1;
    }
    default: return fail(g,io,B5_LOCKED);
    }
    return r<0 ? fail(g,io,r):0;
}
