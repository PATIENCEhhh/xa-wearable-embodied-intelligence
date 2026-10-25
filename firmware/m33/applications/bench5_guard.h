#ifndef BENCH5_GUARD_H
#define BENCH5_GUARD_H
#include <stdint.h>
#include "control/rehab_curl_planner.h"

/* 独立于 RT-Thread 的状态机，便于在主机上注入丢包、错误反馈和时钟回绕。
 * 仅操作 5 号；不采用旧关节零偏。所有位置都是电机协议原始 rad。
 * 屈曲方向已通过手动姿态核对；首次电动试动仍需现场支撑及停止验收。 */
enum { B5_IDLE, B5_READ_POS, B5_READ_VEL, B5_STOP_FIRST, B5_MODE,
       B5_CURRENT, B5_SPEED, B5_HOLD, B5_ENABLE, B5_TARGET,
       B5_MONITOR_POS, B5_MONITOR_VEL, B5_STOPPING, B5_DONE, B5_FAULT,
       B5_VERIFY_STOP, B5_VERIFY_ENABLE };
enum { B5_OK=0, B5_LOCKED=-20, B5_BUSY=-21, B5_IO=-22,
       B5_TIMEOUT=-23, B5_FEEDBACK=-24, B5_LIMIT=-25, B5_CANCELLED=-26,
       B5_NO_PROGRESS=-27 };
typedef struct {
    int (*read)(uint16_t index);
    /* 仅返回指定索引、严格晚于 after_ms 的新回复；0 表示未到，负值表示错误。 */
    int (*reply)(uint16_t index, uint32_t after_ms, float *value);
    int (*write)(uint16_t index, float value);
    int (*enable)(void);
    int (*stop)(void);
    /* 新鲜的运行状态反馈；mode=0停用、2运行。无回复为0，故障为负。 */
    int (*status)(unsigned mode);
    /* 旧主动反馈缓存：仅返回晚于after_ms的新样本，位置/速度来自同一帧。
     * 0表示暂无新样本，负值表示故障；缺省NULL保留历史参数查询测试路径。 */
    int (*feedback)(uint32_t after_ms,float *position,float *velocity);
} b5_io_t;
typedef struct {
    int locked, direction;
    float current_a;
} b5_profile_t;
typedef struct {
    unsigned state, step, requested, attempts, used;
    int result, stop_result;
    uint32_t started, phase_at, request_at, enabled_at, last_pos_at;
    float origin, position, target;
    float velocities[5], previous_position;
    unsigned samples;
    uint32_t previous_at;
    /* STOP请求不会擦除首次失败，便于区分准备失败与已经发送使能/运动目标。 */
    unsigned failure_state, last_index, enable_attempts, target_attempts;
    int first_error;
    float expected, actual;
    uint32_t travel_at, progress_at;
    float travel, progress_position, peak_velocity, velocity_mean;
    /* 演示入口复用旧弯举规划器；历史5°入口保留为离线回归，不再由网络启动。 */
    unsigned legacy_curl;
    /* 仅供本地LVGL会话；原网络入口保持单次。每完整往返重置单周期时限。 */
    unsigned repeat_curl, completed_cycles;
    /* Fixed-action opt-in: bind the existing planner endpoints once to fresh session origin. */
    unsigned relative_curl;
    rehab_curl_planner_t curl;
} b5_guard_t;
int b5_start(b5_guard_t *g, const b5_profile_t *p, uint32_t now);
int b5_start_curl(b5_guard_t *g, const b5_profile_t *p,
                  const rehab_curl_config_t *config, uint32_t now);
void b5_cancel(b5_guard_t *g, uint32_t now);
void b5_tick(b5_guard_t *g, const b5_profile_t *p, const b5_io_t *io, uint32_t now);
int b5_active(const b5_guard_t *g);
#endif
