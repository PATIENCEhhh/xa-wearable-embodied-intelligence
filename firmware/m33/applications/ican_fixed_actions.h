#ifndef ICAN_FIXED_ACTIONS_H
#define ICAN_FIXED_ACTIONS_H
#include "control/rehab_curl_planner.h"
#include "control/control_layer_cfg.h"

/* 这些旧端点作为动作幅度/驻留模板；实际固定动作在新鲜采样后绑定本轮起点。
 * 原电流、速度、反馈新鲜度、单段/使能/整组时限保留；不接收任意角度。
 * 6为完整弯举，12/13/14为小幅、中幅及停留动作。 */
static int ican_fixed_action_config(unsigned opcode, rehab_curl_config_t *config)
{
    const rehab_curl_config_t base = {
        CONTROL_REHAB_CURL_HARD_MIN_RAW_RAD, CONTROL_REHAB_CURL_HARD_MAX_RAW_RAD,
        CONTROL_REHAB_CURL_TOP_TARGET_RAW_RAD, CONTROL_REHAB_CURL_BOTTOM_TARGET_RAW_RAD,
        CONTROL_REHAB_CURL_POSITION_TOLERANCE_RAD, CONTROL_REHAB_CURL_MAX_FEEDBACK_SPEED_RAD_S,
        CONTROL_REHAB_CURL_DWELL_MS, CONTROL_REHAB_CURL_SEGMENT_TIMEOUT_MS,
        CONTROL_REHAB_CURL_COMMAND_REFRESH_MS, CONTROL_REHAB_CURL_ARRIVAL_SAMPLES
    };
    if (!config || !(opcode == 6U || (opcode >= 12U && opcode <= 14U))) return 0;
    *config = base;
    if (opcode == 12U) {
        /* 约30°电机角：相对当前垂直伸直参考，不是30°精密关节标定。 */
        config->bottom_target_pos_rad = CONTROL_MOTOR_JOINT5_ZERO_OFFSET_RAD + 0.523598776f;
    } else if (opcode == 13U || opcode == 14U) {
        config->bottom_target_pos_rad = CONTROL_MOTOR_JOINT5_ZERO_OFFSET_RAD + 1.047197551f;
        if (opcode == 14U) config->dwell_ms = 2000U;
    }
    return 1;
}
#endif
