#ifndef M33_DAP_DIAG_H
#define M33_DAP_DIAG_H
#include <rtthread.h>
/* DAP诊断允许槽位4/5/6的Get-ID及参数读取。
 * op5/6/7仅限5号管理入口，分别是三轴停止、4/6保持+5弯举请求和快照；不允许任意参数写入。
 * op8为肘部细节，op9为保持轴基线/失败阶段，op10为肘部首次失败记录，均只读。
 * 只读op11：index=触发原因位，value_bits/uid_lo/uid_hi=位置/速度/基线，
 * reply_tick=触发时已进入使能流程的掩码（bit0=4号、bit1=6号）。不是使能成功证明。
 * op12/13/14：约30°、60°、60°停留2秒固定屈伸，共用op6单次门禁与整组STOP。
 * op>=5时 index=状态、value_bits=门禁标志、uid_lo=运行结果、uid_hi=停止发送结果。
 * 主机先写opcode/joint，最后提交递增request_seq；done_seq确认前不得覆盖请求。
 * 所有字段固定32位，真实结果只能由主循环写入。state=1等待，2完成；result=0才是匹配回复。
 * value_bits保留电机参数原始float位型，不冒充SDK关节角；超时不证明电机离线。 */
typedef struct {
    rt_uint32_t magic, request_seq, opcode, joint;
    rt_uint32_t done_seq, state, result, start_tick;
    rt_uint32_t end_tick, motor_id, index, value_bits;
    rt_uint32_t uid_lo, uid_hi, reply_tick, polls;
} m33_dap_diag_t;
extern volatile m33_dap_diag_t g_m33_dap_diag;
void m33_dap_diag_poll(void);
#endif
