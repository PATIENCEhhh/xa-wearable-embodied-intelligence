#ifndef REHAB_ARM_LIFT_H
#define REHAB_ARM_LIFT_H
#include "rehab_scurve.h"

/* 单位始终为电机协议原始rad，4号外部减速比不得重复套用。
 * 本模块只规划目标；驱动适配器必须先完成三轴停用、参数读回和保持准备。
 * 没有实测上举端点时calibrated必须为0，不能仅凭初始位置开放运动。 */
enum { LIFT_IDLE, LIFT_RAISE, LIFT_DWELL_HIGH, LIFT_RETURN,
       LIFT_DWELL_HOME, LIFT_DONE, LIFT_FAULT };
enum { LIFT_OK, LIFT_INVALID=-1, LIFT_FEEDBACK=-2, LIFT_LIMIT=-3,
       LIFT_TIMEOUT=-4, LIFT_CANCELLED=-5 };
typedef struct {
    unsigned calibrated, repeat;
    float min_raw, max_raw, raised_raw;
    float speed, accel, jerk, feedback_speed, hold_error, tolerance;
    unsigned arrival_samples;
    uint32_t dwell_ms, segment_timeout_ms;
} rehab_arm_lift_config_t;
typedef struct {
    /* 仅传入新收到的一帧/轴；timestamp_ms是接收时刻，不能用当前时刻伪造。 */
    unsigned valid, fault;
    uint32_t timestamp_ms;
    float position, velocity;
} rehab_arm_lift_axis_t;
typedef struct { rehab_arm_lift_axis_t axis[3]; } rehab_arm_lift_feedback_t;
typedef struct {
    unsigned state, samples, cycles;
    int result;
    uint32_t phase_at, last_tick[3], last_arrival_tick;
    float origin[3], target;
    rehab_arm_lift_config_t config;
    rehab_scurve_profile_t segment;
} rehab_arm_lift_t;
typedef struct {
    unsigned write_targets, stop_all, state, cycles;
    int result;
    float target_raw[3]; /* 数组0/1/2对应电机4/5/6。 */
} rehab_arm_lift_output_t;
/* 硬件执行适配：prepare负责串行预置三轴当前位置、限流/限速、模式并读回，
 * 随后使能及确认运行反馈。适配层须和现有控制所有者互斥，禁止绕过CAN出口。
 * feedback只能返回真实新鲜反馈；target_raw接受原始坐标。任一IO失败整组停止。
 * 非阻塞prepare返回0等待、1成功、负值失败；没有prepare就拒绝启动。 */
typedef struct {
    int (*prepare)(void *context);
    int (*feedback)(void *context,rehab_arm_lift_feedback_t *feedback);
    int (*target_raw)(void *context,unsigned motor_id,float raw_rad);
    int (*stop)(void *context,unsigned motor_id);
} rehab_arm_lift_io_t;
typedef struct {
    rehab_arm_lift_t motion;
    rehab_arm_lift_config_t config;
    unsigned active, preparing, stopping, stop_attempts[3], stop_done;
    int result, stop_result;
    uint32_t prepared_at, stop_at;
} rehab_arm_lift_session_t;
int rehab_arm_lift_session_start(rehab_arm_lift_session_t *session,
                                const rehab_arm_lift_config_t *config,uint32_t now);
void rehab_arm_lift_session_step(rehab_arm_lift_session_t *session,
    const rehab_arm_lift_io_t *io,void *context,uint32_t now,unsigned stop_requested);
int rehab_arm_lift_start(rehab_arm_lift_t *g,const rehab_arm_lift_config_t *config,
                        const rehab_arm_lift_feedback_t *feedback,uint32_t now);
void rehab_arm_lift_step(rehab_arm_lift_t *g,const rehab_arm_lift_feedback_t *feedback,
                        uint32_t now,unsigned stop_requested,rehab_arm_lift_output_t *out);
#endif
