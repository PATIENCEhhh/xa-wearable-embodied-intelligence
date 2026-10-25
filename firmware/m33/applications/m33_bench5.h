#ifndef M33_BENCH5_H
#define M33_BENCH5_H
#include <rtthread.h>
/* ABI11：op=5请求三轴停止并锁存，6请求4/6保持及5号完整弯举一轮，7只读快照。
 * op12/13/14分别为约30°、60°、60°端点停留2秒的单次固定动作。
 * result 为请求接收结果；运行结果、停止发送结果须另查快照。 */
rt_err_t m33_bench5_command(unsigned op);
void m33_bench5_snapshot(rt_uint32_t *state, rt_uint32_t *flags,
                         rt_int32_t *result, rt_int32_t *stop_result);
rt_bool_t m33_bench5_busy(void);
/* CAN出口串行化。成功后必须leave；不允许其他线程抢占试动或清故障。 */
rt_err_t m33_bench5_tx_enter(rt_uint32_t id, rt_uint8_t ide);
void m33_bench5_tx_leave(void);
void m33_bench5_detail(float *origin,float *position,float *target,rt_uint32_t *attempts);
void m33_bench5_support(float *origin4,float *origin6,rt_uint32_t *phase,
                         rt_uint32_t *failed_joint,rt_uint32_t *failed_phase);
/* op11只读首次保持反馈故障；reason=0表示没有锁存样本。 */
void m33_bench5_trip(float *position,float *velocity,float *origin,
                     rt_uint32_t *reason,rt_uint32_t *enabled_mask);
void m33_bench5_failure(volatile rt_uint32_t *state,volatile rt_uint32_t *error,
    volatile rt_uint32_t *index,volatile rt_uint32_t *enables,volatile rt_uint32_t *targets);
#endif
