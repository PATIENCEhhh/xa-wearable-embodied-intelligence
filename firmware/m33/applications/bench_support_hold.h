#ifndef BENCH_SUPPORT_HOLD_H
#define BENCH_SUPPORT_HOLD_H
#include <stdint.h>

/* 4/6轴只保持本轮采集的原始位置，不接受网络目标，不使用历史关节零偏。
 * 参数请求串行处理；运行监控只读各轴反馈缓存，不与5号争用参数回复。 */
enum { BH_IDLE, BH_SAMPLE_POS, BH_SAMPLE_VEL, BH_STOP, BH_STOPPED,
       BH_MODE, BH_CURRENT, BH_SPEED, BH_TARGET, BH_RECHECK,
       BH_ENABLE, BH_ENABLED, BH_READY, BH_FAULT };
/* 可组合原因位；首次监控故障锁存，不用停止后的缓存冒充触发样本。 */
enum { BH_TRIP_POSITION=1U, BH_TRIP_SPEED=2U, BH_TRIP_NONFINITE=4U,
       BH_TRIP_FEEDBACK=8U, BH_TRIP_STALE=16U };
typedef struct {
    unsigned reason, enabled_mask;
    float position, velocity, origin;
} bh_trip_t;
typedef struct {
    int (*read)(unsigned joint, uint16_t index);
    int (*reply)(unsigned joint, uint16_t index, float *value);
    int (*write)(unsigned joint, uint16_t index, float value);
    int (*enable)(unsigned joint);
    int (*stop)(unsigned joint);
    /* 必须是原始电机rad/rad/s；返回0表示无新鲜反馈，负值表示故障。 */
    int (*feedback)(unsigned joint, unsigned mode, uint32_t since,
                    float *position, float *velocity);
} bh_io_t;
typedef struct {
    unsigned phase, axis, samples, requested, written, enabled_mask;
    unsigned failed_joint, failure_phase;
    int result;
    uint32_t started, phase_at, request_at, enabled_at[2];
    float origin[2];
    /* 准备期使用速度窗口，并独立检查逐段位移和累计路程，防止正负抵消。 */
    uint32_t sample_started, sample_at;
    float sample_position, sample_travel, velocity_sum, velocity_peak;
    bh_trip_t trip;
    /* 默认仍为旧4/6保持；新增动作只替换实际轴列表和驱动参数。 */
    unsigned joints[2], count, moving;
    float speed, current;
} bh_guard_t;
void bh_start(bh_guard_t *g, uint32_t now);
void bh_start_axes(bh_guard_t *g,uint32_t now,unsigned first,unsigned second,
                   unsigned count,float speed,float current,unsigned moving);
/* 0准备中、1两轴已保持、负值故障。只有返回1才允许启动5号。 */
int bh_tick(bh_guard_t *g, const bh_io_t *io, uint32_t now);
#endif
