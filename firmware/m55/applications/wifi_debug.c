#include "wifi_debug.h"
#include "wifi_debug_binding.h"
#include "bench5_auth.h"
#include "ican_action_routes.h"
#include <board.h>
#include <wlan_mgnt.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* 无线服务暴露固定诊断、5号STOP及经本机DAP授权的旧规划器单轮弯举。
 * 不执行shell、任意内存写入或任意CAN帧；上电不自动授权或运动。
 * 同一时刻仅处理一个 HTTP 客户端；请求提交后必须等主循环完成，禁止覆盖邮箱。
 * 使用无线查询期间，不得同时运行 DAP 邮箱查询工具。 */
typedef struct {
    uint32_t magic, request_seq, opcode, joint;
    uint32_t done_seq, state, result, start_tick;
    uint32_t end_tick, motor_id, index, value_bits;
    uint32_t uid_lo, uid_hi, reply_tick, polls;
} debug_mailbox_t;
typedef char mailbox_size_check[(sizeof(debug_mailbox_t) == 64) ? 1 : -1];
#define MAILBOX ((volatile debug_mailbox_t *)(uintptr_t)WIFI_DEBUG_MAILBOX)
static rt_thread_t debug_thread;
static rt_tick_t last_query;
static rt_bool_t queried;
__attribute__((aligned(32))) volatile char g_wifi_trial_token[64];

/* 仅供首次通过有线调试器临时入网，不经 HTTP 暴露，不写 Flash。
 * 密码由主机写入 RAM，最后发布 request；处理后擦除该临时缓冲区。
 * 与 M33 共用串口时也可完成配置，后续诊断完全走 Wi-Fi。 */
volatile struct {
    uint32_t request, done;
    int32_t result;
    uint32_t ready;
    char ssid[33], password[65];
} g_wifi_debug_setup;

static void setup_entry(void *unused)
{
    RT_UNUSED(unused);
    for (;;) {
        uint32_t seq = g_wifi_debug_setup.request;
        if (seq && seq != g_wifi_debug_setup.done) {
            char ssid[33], password[65];
            size_t i;
            __DMB();
            for (i = 0; i < sizeof(ssid); i++) ssid[i] = g_wifi_debug_setup.ssid[i];
            for (i = 0; i < sizeof(password); i++) password[i] = g_wifi_debug_setup.password[i];
            ssid[32] = 0; password[64] = 0;
            g_wifi_debug_setup.result = rt_wlan_connect(ssid, password);
            /* volatile 擦除，避免优化器消除密码清理；无线驱动自身保留重连所需 RAM。 */
            for (i = 0; i < sizeof(password); i++) ((volatile char *)password)[i] = 0;
            for (i = 0; i < sizeof(g_wifi_debug_setup.password); i++) g_wifi_debug_setup.password[i] = 0;
            g_wifi_debug_setup.ready = rt_wlan_is_ready();
            __DMB();
            g_wifi_debug_setup.done = seq;
        }
        rt_thread_mdelay(100);
    }
}

static void invalidate_mailbox(void)
{
    /* M33 和 M55 不共享一级数据缓存；每次读取前使 M55 的副本失效。 */
    SCB_InvalidateDCache_by_Addr((void *)MAILBOX, 64);
    __DSB();
}

static rt_bool_t binding_valid(void)
{
    const volatile uint32_t *descriptor = (const volatile uint32_t *)(uintptr_t)WIFI_DEBUG_DESCRIPTOR;
    static const uint32_t expected[8] = {0x57444231U, 11U, WIFI_DEBUG_MAILBOX, 64U,
        0xe7c19011U, 0x326a9d41U, 0x5f21ab88U, 0x9d643207U};
    unsigned i;
    invalidate_mailbox();
    if (MAILBOX->magic != 0x44415031U) return RT_FALSE;
    /* 描述符和邮箱地址由配套 ELF 确定。ABI、地址、长度及标识均匹配才放行。 */
    SCB_InvalidateDCache_by_Addr((void *)descriptor, 32); __DSB();
    for (i = 0; i < 8; i++) if (descriptor[i] != expected[i]) return RT_FALSE;
    return RT_TRUE;
}

static int send_all(int fd, const char *data, size_t len)
{
    while (len) {
        int n = send(fd, data, len, 0);
        if (n <= 0) return -1;
        data += n; len -= (size_t)n;
    }
    return 0;
}

static void respond(int fd, int status, const char *type, const char *body)
{
    char header[224];
    const char *reason = status == 200 ? "OK" : status == 404 ? "Not Found" :
                         status == 405 ? "Method Not Allowed" : status == 429 ? "Too Many Requests" :
                         status == 503 ? "Service Unavailable" : status == 504 ? "Gateway Timeout" : "Bad Request";
    int n = snprintf(header, sizeof(header),
        "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
        status, reason, type, (unsigned)strlen(body));
    if (n > 0 && n < (int)sizeof(header) && send_all(fd, header, (size_t)n) == 0)
        (void)send_all(fd, body, strlen(body));
}

static void json_error(int fd, int status, const char *error)
{
    char body[128];
    snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}", error);
    respond(fd, status, "application/json", body);
}

static void motor_query(int fd, uint32_t joint, uint32_t opcode)
{
    uint32_t seq, values[16], i;
    rt_tick_t start, now = rt_tick_get();
    char body[768];
    if (opcode!=5U && queried && (rt_tick_t)(now - last_query) < rt_tick_from_millisecond(350)) {
        json_error(fd, 429, "query_rate_limit"); return;
    }
    if (!binding_valid()) { json_error(fd, 503, "m33_firmware_mismatch"); return; }
    if (MAILBOX->request_seq != MAILBOX->done_seq) {
        json_error(fd, 503, "m33_query_busy"); return;
    }
    if (MAILBOX->request_seq >= 0xfffffffeU) {
        json_error(fd, 503, "sequence_exhausted"); return;
    }
    queried = RT_TRUE; last_query = now;
    seq = MAILBOX->request_seq + 1;
    MAILBOX->opcode = opcode;
    MAILBOX->joint = joint;
    /* 此时请求尚未发布，M33 不会修改结果。先刷参数，再发布序号并刷缓存。
     * 后续只做 invalidate，不把旧的状态和结果回写到 M33。 */
    SCB_CleanDCache_by_Addr((void *)MAILBOX, 32); __DSB();
    MAILBOX->request_seq = seq;
    SCB_CleanDCache_by_Addr((void *)MAILBOX, 32); __DSB();
    start = rt_tick_get();
    do {
        rt_thread_mdelay(20);
        invalidate_mailbox();
        if (MAILBOX->magic != 0x44415031U || MAILBOX->request_seq != seq) {
            json_error(fd, 503, "m33_restarted_or_request_changed"); return;
        }
        if (MAILBOX->done_seq == seq) break;
    } while ((rt_tick_t)(rt_tick_get() - start) < rt_tick_from_millisecond(1800));
    if (MAILBOX->done_seq != seq) { json_error(fd, 504, "m33_loop_timeout"); return; }
    __DMB();
    for (i = 0; i < 16; i++) values[i] = ((volatile uint32_t *)MAILBOX)[i];
    if (MAILBOX->done_seq != seq || MAILBOX->request_seq != seq) {
        json_error(fd, 503, "mailbox_changed"); return;
    }
    if (opcode==11U) {
        float position,velocity,origin;
        char p[48]="null",v[48]="null",o[48]="null";
        unsigned reason=values[10],sampled=reason && !(reason & (8U|16U));
        memcpy(&position,&values[11],4); memcpy(&velocity,&values[12],4); memcpy(&origin,&values[13],4);
        /* 无样本/无回复和非有限值须输出JSON null，不能冒充零或生成非法JSON。 */
        if (sampled && isfinite(position)) snprintf(p,sizeof(p),"%.9g",(double)position);
        if (sampled && isfinite(velocity)) snprintf(v,sizeof(v),"%.9g",(double)velocity);
        if (reason && isfinite(origin)) snprintf(o,sizeof(o),"%.9g",(double)origin);
        snprintf(body,sizeof(body),
            "{\"ok\":%s,\"reason_bits\":%lu,\"sample_available\":%s,\"position_rad\":%s,\"velocity_rad_s\":%s,\"origin_rad\":%s,\"enable_attempt_mask\":%lu}",
            values[6]==0 ? "true":"false",(unsigned long)reason,sampled ? "true":"false",p,v,o,(unsigned long)values[14]);
        respond(fd,200,"application/json",body); return;
    }
    if (opcode==9U) {
        float origin4,origin6;
        char text4[32]="null",text6[32]="null";
        memcpy(&origin4,&values[11],4); memcpy(&origin6,&values[12],4);
        if(isfinite(origin4)) snprintf(text4,sizeof(text4),"%.7f",(double)origin4);
        if(isfinite(origin6)) snprintf(text6,sizeof(text6),"%.7f",(double)origin6);
        snprintf(body,sizeof(body),
            "{\"ok\":true,\"origin4_rad\":%s,\"origin6_rad\":%s,\"phase\":%lu,\"failed_joint\":%lu,\"failure_phase\":%lu}",
            text4,text6,(unsigned long)values[10],
            (unsigned long)values[13],(unsigned long)values[14]);
        respond(fd,200,"application/json",body); return;
    }
    if (opcode==10U) {
        snprintf(body,sizeof(body),
            "{\"ok\":true,\"failure_state\":%lu,\"first_error\":%ld,\"last_index\":%lu,\"enable_attempts\":%lu,\"target_attempts\":%lu}",
            (unsigned long)values[10],(long)(int32_t)values[11],(unsigned long)values[12],
            (unsigned long)values[13],(unsigned long)values[14]);
        respond(fd,200,"application/json",body); return;
    }
    if (opcode==8U) {
        float origin,position,target;
        memcpy(&origin,&values[11],4); memcpy(&position,&values[12],4); memcpy(&target,&values[13],4);
        snprintf(body,sizeof(body),
            "{\"ok\":true,\"origin_rad\":%.7f,\"position_rad\":%.7f,\"target_rad\":%.7f,\"stop_attempts\":%lu}",
            (double)origin,(double)position,(double)target,(unsigned long)values[14]);
        respond(fd,200,"application/json",body); return;
    }
    if (opcode>=5U) {
        snprintf(body,sizeof(body),
            "{\"ok\":%s,\"joint\":5,\"opcode\":%lu,\"seq\":%lu,\"result\":%ld,"
            "\"state\":%lu,\"flags\":%lu,\"trial_locked\":%s,\"active\":%s,"
            "\"run_result\":%ld,\"stop_send_result\":%ld,\"physical_stop_verified\":false}",
            values[6]==0 ? "true":"false",(unsigned long)opcode,(unsigned long)seq,
            (long)(int32_t)values[6],(unsigned long)values[10],(unsigned long)values[11],
            values[11]&1U ? "true":"false",values[11]&4U ? "true":"false",
            (long)(int32_t)values[12],(long)(int32_t)values[13]);
        respond(fd,200,"application/json",body); return;
    }
    /* 即使 CAN 查询失败，也返回真实的错误码；零值不能冒充已测位置。 */
    snprintf(body, sizeof(body),
        "{\"ok\":%s,\"joint\":%lu,\"opcode\":%lu,\"seq\":%lu,\"result\":%ld,"
        "\"motor_id\":%lu,\"index\":%lu,\"value_bits\":%lu,"
        "\"uid\":\"%08lx%08lx\",\"reply_tick\":%lu,\"start_tick\":%lu,"
        "\"end_tick\":%lu,\"raw_motor_units\":true}",
        values[6] == 0 ? "true" : "false", (unsigned long)joint, (unsigned long)opcode,
        (unsigned long)seq, (long)(int32_t)values[6], (unsigned long)values[9],
        (unsigned long)values[10], (unsigned long)values[11], (unsigned long)values[13],
        (unsigned long)values[12], (unsigned long)values[14], (unsigned long)values[7],
        (unsigned long)values[8]);
    respond(fd, 200, "application/json", body);
}

static const char page[] =
    "<!doctype html><html lang=zh-CN><meta charset=utf-8><meta name=viewport content='width=device-width'>"
    "<title>机械臂无线诊断</title><style>body{font:18px sans-serif;max-width:800px;margin:32px auto;padding:16px}"
    "button,select{font:inherit;padding:10px;margin:6px}pre{white-space:pre-wrap;background:#eee;padding:16px}</style>"
    "<h1>机械臂无线诊断</h1><p>4、6号保持本轮当前位置，5号先屈曲后伸展一轮，需本机临时授权。结束或故障会停用三轴，请保持机械支撑；停止请求不等于物理停止已确认。</p>"
    "<button onclick=read('/api/status')>板卡状态</button><select id=j>"
    "<option value=4>4 肩部抬臂</option><option value=5>5 肘部屈伸</option><option value=6>6 上臂旋转</option></select>"
    "<button onclick=m('id')>设备 ID</button><button onclick=m('position')>位置</button>"
    "<button onclick=m('velocity')>速度</button><button onclick=m('mode')>模式</button>"
    "<button onclick=read('/api/bench/5/status')>5号测试状态</button>"
    "<button onclick=read('/api/bench/5/support')>保持轴状态</button>"
    "<button onclick=read('/api/bench/5/failure')>首次失败记录</button>"
    "<button onclick=read('/api/bench/5/trip')>保持保护触发样本</button>"
    "<button style='background:#a00;color:white' onclick=read('/api/bench/5/stop','POST')>停止4/5/6三轴</button><pre id=o>准备就绪</pre>"
    "<script>let busy=false;async function read(p,method='GET'){if(busy)return;busy=true;try{let r=await fetch(p,{method,cache:'no-store'});"
    "let d=await r.json();if(d.ok&&d.raw_motor_units&&(d.opcode===2||d.opcode===3)){let b=new ArrayBuffer(4);"
    "let v=new DataView(b);v.setUint32(0,d.value_bits,true);d.value_raw=v.getFloat32(0,true)}"
    "document.getElementById('o').textContent=JSON.stringify(d,null,2)}catch(e){document.getElementById('o').textContent=String(e)}"
    "finally{busy=false}}function m(op){read('/api/motor/'+document.getElementById('j').value+'/'+op)}read('/api/status')</script></html>";

static void handle_client(int fd)
{
    char request[1024], method[12], path[96], version[16], body[640];
    size_t used = 0;
    rt_tick_t start = rt_tick_get();
    /* DAP写RAM不更新CPU数据缓存；令牌独占两行，失效不会丢弃相邻业务数据。 */
    SCB_InvalidateDCache_by_Addr((void *)g_wifi_trial_token,64); __DSB();
    /* TCP 可拆包；请求头必须完整收到。总截止时间防止慢客户端长期占住服务。 */
    while (used < sizeof(request) - 1) {
        int n = recv(fd, request + used, sizeof(request) - 1 - used, 0);
        if (n <= 0) return;
        used += (size_t)n; request[used] = 0;
        if (strstr(request, "\r\n\r\n")) break;
        if ((rt_tick_t)(rt_tick_get() - start) > rt_tick_from_millisecond(2000)) return;
    }
    if (!strstr(request, "\r\n\r\n")) { json_error(fd, 400, "header_too_large"); return; }
    if (sscanf(request, "%11s %95s %15s", method, path, version) != 3 ||
        (strcmp(version, "HTTP/1.1") && strcmp(version, "HTTP/1.0"))) {
        json_error(fd, 400, "invalid_request"); return;
    }
    unsigned action_opcode = ican_action_opcode(path);
    if (!strcmp(path,"/api/bench/5/stop") || action_opcode) {
        if (strcmp(method,"POST")) { json_error(fd,405,"post_required"); return; }
        if (action_opcode) {
            unsigned i;
            if (!g_wifi_trial_token[0]) { json_error(fd,423,"local_arming_required"); return; }
            if (!bench5_auth(request,g_wifi_trial_token)) { json_error(fd,403,"invalid_bench_key"); return; }
            /* 在提交邮箱前消耗令牌，超时也不重发；M33另有一次性、60秒RAM门禁。 */
            for(i=0;i<64;i++) g_wifi_trial_token[i]=0;
            SCB_CleanDCache_by_Addr((void *)g_wifi_trial_token,64); __DSB();
            motor_query(fd,5,action_opcode); return;
        }
        motor_query(fd,5,5); return;
    }
    if (strcmp(method, "GET")) { json_error(fd, 405, "get_only"); return; }
    if (!strcmp(path,"/api/bench/5/status")) { motor_query(fd,5,7); return; }
    if (!strcmp(path,"/api/bench/5/detail")) { motor_query(fd,5,8); return; }
    if (!strcmp(path,"/api/bench/5/support")) { motor_query(fd,5,9); return; }
    if (!strcmp(path,"/api/bench/5/failure")) { motor_query(fd,5,10); return; }
    if (!strcmp(path,"/api/bench/5/trip")) { motor_query(fd,5,11); return; }
    if (!strcmp(path, "/")) { respond(fd, 200, "text/html; charset=utf-8", page); return; }
    if (!strcmp(path, "/api/status")) {
        rt_bool_t valid = binding_valid();
        snprintf(body, sizeof(body),
            "{\"ok\":true,\"service\":\"arm-wifi-debug-v5\",\"m55_tick\":%lu,"
            "\"wifi_ready\":%s,\"m33_binding\":%s,\"m33_polls\":%lu,"
            "\"request_seq\":%lu,\"done_seq\":%lu,\"read_only\":false,"
            "\"queries_read_only\":true,\"local_arming_required\":true,\"auth_ready\":%s,\"motion_profile\":\"ican-lvgl-resistance-v14\",\"single_cycle\":true,\"hold_joints\":[4,6],\"stop_joints\":[4,5,6]}",
            (unsigned long)rt_tick_get(), rt_wlan_is_ready() ? "true" : "false",
            valid ? "true" : "false", valid ? (unsigned long)MAILBOX->polls : 0UL,
            valid ? (unsigned long)MAILBOX->request_seq : 0UL, valid ? (unsigned long)MAILBOX->done_seq : 0UL,
            g_wifi_trial_token[0] ? "true":"false");
        respond(fd, 200, "application/json", body); return;
    }
    /* 完整字符串白名单：禁止任意关节、任意参数号及路径尾随字符。 */
    if (!strncmp(path, "/api/motor/", 11) && path[11] >= '4' && path[11] <= '6' && path[12] == '/') {
        const char *op = path + 13;
        uint32_t opcode = !strcmp(op, "id") ? 1 : !strcmp(op, "position") ? 2 :
                          !strcmp(op, "velocity") ? 3 : !strcmp(op, "mode") ? 4 : 0;
        if (opcode) { motor_query(fd, (uint32_t)(path[11] - '0'), opcode); return; }
    }
    json_error(fd, 404, "endpoint_not_found");
}

static void debug_entry(void *unused)
{
    struct sockaddr_in addr;
    RT_UNUSED(unused);
    while (1) {
        int listener = socket(AF_INET, SOCK_STREAM, 0), opt = 1;
        if (listener < 0) { rt_thread_mdelay(1000); continue; }
        setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET; addr.sin_port = htons(8088); addr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(listener, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(listener, 2) < 0) {
            close(listener); rt_thread_mdelay(1000); continue;
        }
        rt_kprintf("[wifi_debug] HTTP 8088; bench5 one-shot/local-arm; binding=%d\n", binding_valid());
        while (1) {
            struct timeval timeout = {2, 0};
            int client = accept(listener, RT_NULL, RT_NULL);
            if (client < 0) break;
            setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
            handle_client(client); close(client);
        }
        close(listener); rt_thread_mdelay(1000);
    }
}

rt_err_t wifi_debug_start(void)
{
    rt_thread_t setup;
    if (debug_thread) return RT_EOK;
    setup = rt_thread_create("wifi_set", setup_entry, RT_NULL, 6144, 25, 10);
    if (!setup) return -RT_ENOMEM;
    rt_thread_startup(setup);
    debug_thread = rt_thread_create("wifi_dbg", debug_entry, RT_NULL, 8192, 24, 10);
    if (!debug_thread) return -RT_ENOMEM;
    return rt_thread_startup(debug_thread);
}
