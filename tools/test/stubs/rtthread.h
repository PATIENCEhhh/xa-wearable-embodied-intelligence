#ifndef B5_TEST_RTTHREAD_H
#define B5_TEST_RTTHREAD_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define rt_memset memset
typedef int rt_err_t;
typedef int rt_bool_t;
typedef uint8_t rt_uint8_t;
typedef uint16_t rt_uint16_t;
typedef uint32_t rt_uint32_t;
typedef uint64_t rt_uint64_t;
typedef int32_t rt_int32_t;
typedef int16_t rt_int16_t;
typedef uint32_t rt_tick_t;
typedef void *rt_thread_t;
struct rt_mutex { int held; };
#define RT_TRUE 1
#define RT_FALSE 0
#define RT_NULL NULL
#define RT_EOK 0
#define RT_ERROR 1
#define RT_EBUSY 7
#define RT_EINVAL 10
#define RT_ENOMEM 5
#define RT_ENOSYS 6
#define RT_EFULL 8
#define RT_EEMPTY 9
#define RT_WAITING_FOREVER -1
#define RT_IPC_FLAG_PRIO 1
#define RT_TICK_PER_SECOND 1000
#define RT_UNUSED(x) (void)(x)
#define INIT_APP_EXPORT(fn)
rt_tick_t rt_tick_get(void);
rt_thread_t rt_thread_self(void);
int rt_mutex_take(struct rt_mutex *,int);
int rt_mutex_release(struct rt_mutex *);
int rt_mutex_init(struct rt_mutex *,const char *,int);
rt_thread_t rt_thread_create(const char *,void (*)(void *),void *,unsigned,unsigned,unsigned);
int rt_thread_startup(rt_thread_t);
void rt_thread_mdelay(unsigned);
#endif
