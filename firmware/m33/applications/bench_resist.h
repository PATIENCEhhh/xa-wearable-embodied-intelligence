#ifndef BENCH_RESIST_H
#define BENCH_RESIST_H
#include "bench5_guard.h"
#include "control/rehab_resist_strategy.h"
enum { BR_IDLE,BR_STOP,BR_VERIFY_STOP,BR_POSITION,BR_VELOCITY,
       BR_MODE,BR_LIMIT,BR_ZERO,BR_ENABLE,BR_VERIFY_ENABLE,BR_RUN,BR_FAULT };
typedef struct {
    unsigned phase,requested,written,samples,enabled,failure_phase,last_index;
    uint32_t started,phase_at,request_at,enabled_at,update_at,running_at;
    float origin,position,velocity,current,travel,previous_position,velocity_sum;
    int result;
    rehab_resist_strategy_state_t strategy;
    rehab_strategy_params_t params;
} br_guard_t;
void br_start(br_guard_t *g,uint32_t now);
/* 0 preparing, 1 resisting, 2 time limit complete; negative is a group-stop fault. No position targets. */
int br_tick(br_guard_t *g,const b5_io_t *io,uint32_t now);
#endif
