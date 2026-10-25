#ifndef ICAN_ACTION_ROUTES_H
#define ICAN_ACTION_ROUTES_H
#include <string.h>

/* 完整路径白名单；所有新动作共用原DAP授权与单次令牌，不提供任意参数入口。 */
static unsigned ican_action_opcode(const char *path)
{
    if (!strcmp(path, "/api/bench/5/curl")) return 6U;
    if (!strcmp(path, "/api/bench/5/curl-small")) return 12U;
    if (!strcmp(path, "/api/bench/5/curl-medium")) return 13U;
    if (!strcmp(path, "/api/bench/5/curl-pause")) return 14U;
    return 0;
}
#endif
