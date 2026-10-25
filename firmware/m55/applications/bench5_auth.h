#ifndef BENCH5_AUTH_H
#define BENCH5_AUTH_H
#include <stddef.h>
#include <string.h>

/* 随机128位会话令牌由有线DAP写RAM，不写Flash、不放URL、不在网页显示。
 * 只接受一个完整且精确的专用头；重复头、尾随字符、空令牌都失败。
 * 此处是局域网首测访问控制，明文HTTP不提供链路加密。 */
static int bench5_auth(const char *request,const volatile char *token)
{
    const char *p=strstr(request,"\r\n");
    unsigned count=0,diff=0,i;
    if (!p || !token[0] || token[32]) return 0;
    p+=2;
    while (*p && strncmp(p,"\r\n",2)) {
        const char *end=strstr(p,"\r\n");
        if (!end) return 0;
        if (!strncmp(p,"X-Bench-Key:",12)) {
            if (++count!=1 || end-p!=45 || p[12]!=' ') return 0;
            for(i=0;i<32;i++) diff|=(unsigned char)p[13+i]^(unsigned char)token[i];
        }
        p=end+2;
    }
    return count==1 && diff==0;
}
#endif
