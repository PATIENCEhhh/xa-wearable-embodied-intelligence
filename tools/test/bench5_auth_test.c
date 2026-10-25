#include <assert.h>
#include <stdio.h>
#include "bench5_auth.h"
#include "ican_action_routes.h"
int main(void)
{
    const char token[]="0123456789abcdef0123456789abcdef";
    assert(bench5_auth("POST /api/bench/5/trial HTTP/1.1\r\nX-Bench-Key: 0123456789abcdef0123456789abcdef\r\n\r\n",token));
    assert(!bench5_auth("POST / HTTP/1.1\r\n\r\n",token));
    assert(!bench5_auth("POST / HTTP/1.1\r\nX-Bench-Key: 0123456789abcdef0123456789abcdee\r\n\r\n",token));
    assert(!bench5_auth("POST / HTTP/1.1\r\nX-Bench-Key: 0123456789abcdef0123456789abcdef0\r\n\r\n",token));
    assert(!bench5_auth("POST / HTTP/1.1\r\nX-Bench-Key: 0123456789abcdef0123456789abcdef\r\nX-Bench-Key: 0123456789abcdef0123456789abcdef\r\n\r\n",token));
    assert(!bench5_auth("POST / HTTP/1.1\r\nX-Bench-Key: 0123456789abcdef0123456789abcdef\r\n\r\n",""));
    puts("PASS auth: exact header, wrong/missing/duplicate/trailing/empty key rejected");
    assert(ican_action_opcode("/api/bench/5/curl")==6U);
    assert(ican_action_opcode("/api/bench/5/curl-small")==12U);
    assert(ican_action_opcode("/api/bench/5/curl-medium")==13U);
    assert(ican_action_opcode("/api/bench/5/curl-pause")==14U);
    assert(!ican_action_opcode("/api/bench/5/curl-small/extra"));
    assert(!ican_action_opcode("/api/bench/5/curl-small?angle=90"));
    assert(!ican_action_opcode("/api/bench/4/curl-small"));
    assert(!ican_action_opcode("/api/bench/5/curl-repeat"));
    puts("PASS fixed-action exact route whitelist; no arbitrary angle/repeat or trailing path");
    return 0;
}
