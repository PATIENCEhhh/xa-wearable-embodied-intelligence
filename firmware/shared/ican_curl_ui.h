#ifndef ICAN_CURL_UI_H
#define ICAN_CURL_UI_H
#include <stdint.h>
/* 两条独占缓存行：M55只写请求行，M33只写响应行，避免跨核缓存回写覆盖。
 * 本地触屏命令不经过HTTP邮箱；心跳仅由LVGL定时器推进，界面卡死即停止。
 * start=1、stop=2。响应确认只代表软件状态，不证明物理停止。 */
#define ICAN_UI_MAGIC 0x43554931U
enum { ICAN_UI_START_CURL=1U, ICAN_UI_STOP=2U, ICAN_UI_START_LIFT=3U,
       ICAN_UI_START_SMALL=4U, ICAN_UI_START_MEDIUM=5U, ICAN_UI_START_PAUSE=6U,
       ICAN_UI_START_ROTATE=7U, ICAN_UI_RECORD=8U, ICAN_UI_PLAY=9U,
       ICAN_UI_RESIST=10U };
/* 本地屏幕命令与HTTP操作码分开命名，STOP=2永远不参与动作映射。 */
static inline unsigned ican_ui_action_opcode(unsigned command)
{
    switch(command) {
    case ICAN_UI_START_CURL: return 6U;
    case ICAN_UI_START_SMALL: return 12U;
    case ICAN_UI_START_MEDIUM: return 13U;
    case ICAN_UI_START_PAUSE: return 14U;
    default: return 0U;
    }
}
/* 保留旧版诊断码；当前抬臂/内旋使用独立动作命令，不能降级为屈肘。 */
#define ICAN_UI_LIFT_NOT_READY (-1001)
#define ICAN_UI_NO_RECORD (-1002)
/* phase低16位保持动作阶段，高16位为本次内存轨迹帧数。 */
typedef struct {
    volatile uint32_t seq, command, heartbeat, reserved[5];
    volatile uint32_t magic, done, result, state, active, cycles, phase, fault;
} ican_curl_ui_t;
#endif
