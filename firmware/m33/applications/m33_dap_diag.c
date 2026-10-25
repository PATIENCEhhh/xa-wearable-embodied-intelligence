#include "m33_dap_diag.h"
#include "m33_bench5.h"
#include "control/control_layer.h"
#include <string.h>

/* 独占完整缓存行，供 M55 无线诊断访问，避免回写相邻业务变量。 */
__attribute__((aligned(32))) volatile m33_dap_diag_t g_m33_dap_diag = {0x44415031U};
/* 只读 ABI 描述符置于双核都可读的 SRAM；M55 不跨权限读取 M33 代码区。 */
__attribute__((aligned(32))) volatile rt_uint32_t g_m33_wifi_binding[8] = {
    0x57444231U, 11U, (rt_uint32_t)&g_m33_dap_diag, sizeof(m33_dap_diag_t),
    0xe7c19011U, 0x326a9d41U, 0x5f21ab88U, 0x9d643207U
};
static rt_uint32_t active_seq, active_opcode, active_joint, active_index;
static rt_tick_t active_start;
static rt_bool_t pending;

static void complete(rt_err_t result, rt_tick_t now)
{
    g_m33_dap_diag.result = (rt_uint32_t)result;
    g_m33_dap_diag.end_tick = now;
    g_m33_dap_diag.state = 2U;
    __asm volatile ("dmb" ::: "memory");
    g_m33_dap_diag.done_seq = active_seq;
    pending = RT_FALSE;
}

void m33_dap_diag_poll(void)
{
    rt_tick_t now = rt_tick_get();
    rt_err_t result;
    rt_uint32_t seq;
    control_motor_probe_report_t probe;
    control_motor_param_report_t param;
    if (g_m33_wifi_binding[0] != 0x57444231U) return;
    g_m33_dap_diag.polls++;
    if (!pending)
    {
        seq = g_m33_dap_diag.request_seq;
        if (!seq || seq == g_m33_dap_diag.done_seq) return;
        __asm volatile ("dmb" ::: "memory");
        active_seq = seq;
        active_opcode = g_m33_dap_diag.opcode;
        active_joint = g_m33_dap_diag.joint;
        if (seq != g_m33_dap_diag.request_seq) return;
        g_m33_dap_diag.state = 1U;
        g_m33_dap_diag.start_tick = now;
        g_m33_dap_diag.motor_id = 0;
        g_m33_dap_diag.index = 0;
        g_m33_dap_diag.value_bits = 0;
        g_m33_dap_diag.uid_lo = g_m33_dap_diag.uid_hi = 0;
        g_m33_dap_diag.reply_tick = 0;
        if (active_joint < 4U || active_joint > 6U || active_opcode < 1U || active_opcode > 14U ||
            (active_opcode>=5U && active_joint!=5U))
        {
            complete(-RT_EINVAL, now);
            return;
        }
        if (active_opcode>=5U) {
            rt_uint32_t state, flags;
            rt_int32_t run_result, stop_result;
            if (active_opcode==11U) {
                float position,velocity,origin;
                rt_uint32_t reason,enabled_mask;
                m33_bench5_trip(&position,&velocity,&origin,&reason,&enabled_mask);
                memcpy((void *)&g_m33_dap_diag.value_bits,&position,4);
                memcpy((void *)&g_m33_dap_diag.uid_lo,&velocity,4);
                memcpy((void *)&g_m33_dap_diag.uid_hi,&origin,4);
                g_m33_dap_diag.index=reason;
                g_m33_dap_diag.reply_tick=enabled_mask;
                complete(RT_EOK,now); return;
            }
            if (active_opcode==9U) {
                float origin4,origin6;
                rt_uint32_t phase,failed_joint,failed_phase;
                m33_bench5_support(&origin4,&origin6,&phase,&failed_joint,&failed_phase);
                memcpy((void *)&g_m33_dap_diag.value_bits,&origin4,4);
                memcpy((void *)&g_m33_dap_diag.uid_lo,&origin6,4);
                g_m33_dap_diag.index=phase;
                g_m33_dap_diag.uid_hi=failed_joint;
                g_m33_dap_diag.reply_tick=failed_phase;
                complete(RT_EOK,now); return;
            }
            if (active_opcode==10U) {
                m33_bench5_failure(&g_m33_dap_diag.index,&g_m33_dap_diag.value_bits,
                    &g_m33_dap_diag.uid_lo,&g_m33_dap_diag.uid_hi,&g_m33_dap_diag.reply_tick);
                complete(RT_EOK,now); return;
            }
            if (active_opcode==8U) {
                float origin,position,target;
                rt_uint32_t attempts;
                m33_bench5_detail(&origin,&position,&target,&attempts);
                memcpy((void *)&g_m33_dap_diag.value_bits,&origin,4);
                memcpy((void *)&g_m33_dap_diag.uid_lo,&position,4);
                memcpy((void *)&g_m33_dap_diag.uid_hi,&target,4);
                g_m33_dap_diag.reply_tick=attempts;
                complete(RT_EOK,now); return;
            }
            result=m33_bench5_command(active_opcode);
            m33_bench5_snapshot(&state,&flags,&run_result,&stop_result);
            g_m33_dap_diag.motor_id=5;
            g_m33_dap_diag.index=state;
            g_m33_dap_diag.value_bits=flags;
            g_m33_dap_diag.uid_lo=(rt_uint32_t)run_result;
            g_m33_dap_diag.uid_hi=(rt_uint32_t)stop_result;
            complete(result,now);
            return;
        }
        /* 试动占用参数回复缓存时拒绝其他诊断；状态与STOP仍可使用。 */
        if (m33_bench5_busy()) { complete(-RT_EBUSY,now); return; }
        /* 白名单固定在固件内；不允许主机指定寄存器、函数地址或任意CAN帧。 */
        active_index = active_opcode == 2U ? 0x7019U : active_opcode == 3U ? 0x701BU : 0x7005U;
        active_start = now;
        if (active_opcode == 1U) result = control_motor_probe_id((rt_uint8_t)active_joint);
        else result = control_motor_read_parameter((rt_uint8_t)active_joint, (rt_uint16_t)active_index);
        if (result != RT_EOK) { complete(result, rt_tick_get()); return; }
        pending = RT_TRUE;
        return;
    }
    /* 请求已发出仍不代表设备已回复；仅接受对应ID/索引和本次起点之后的缓存。
     * 原协议没有请求序号，测试期间须排除其他同类诊断请求；每次仅一个在途请求。 */
    if (active_opcode == 1U)
    {
        result = control_get_last_motor_probe(&probe);
        if (result == RT_EOK && probe.valid && probe.motor_id == active_joint &&
            (rt_int32_t)(probe.timestamp - active_start) >= 0)
        {
            g_m33_dap_diag.motor_id = probe.motor_id;
            g_m33_dap_diag.uid_lo = (rt_uint32_t)probe.unique_id;
            g_m33_dap_diag.uid_hi = (rt_uint32_t)(probe.unique_id >> 32);
            g_m33_dap_diag.reply_tick = probe.timestamp;
            complete(RT_EOK, now);
            return;
        }
    }
    else
    {
        result = control_get_last_motor_param(&param);
        if (result == RT_EOK && param.valid && param.motor_id == active_joint &&
            param.index == active_index && (rt_int32_t)(param.timestamp - active_start) >= 0)
        {
            rt_uint32_t bits;
            memcpy(&bits, &param.value_f32, sizeof(bits));
            g_m33_dap_diag.motor_id = param.motor_id;
            g_m33_dap_diag.index = param.index;
            g_m33_dap_diag.value_bits = active_opcode == 4U ? param.raw_u8 : bits;
            g_m33_dap_diag.reply_tick = param.timestamp;
            complete(RT_EOK, now);
            return;
        }
    }
    if ((rt_tick_t)(now - active_start) >= rt_tick_from_millisecond(1000U)) complete(-RT_ETIMEOUT, now);
}
