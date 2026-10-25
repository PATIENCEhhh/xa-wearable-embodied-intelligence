#ifndef WIFI_DEBUG_BINDING_H
#define WIFI_DEBUG_BINDING_H
/* 来自本次配套 M33 ELF；重新链接 M33 后必须重新核对地址与 ABI 描述符。
 * SRAM 邮箱独占两个 32 字节缓存行，启动魔数及描述符均匹配才允许提交。 */
#define WIFI_DEBUG_MAILBOX 0x240BD680UL
#define WIFI_DEBUG_DESCRIPTOR 0x240BD660UL
/* 由重新链接后的M33 ELF核对；只允许本地LVGL发布循环启停，不经HTTP暴露。 */
#define WIFI_DEBUG_CURL_UI 0x240BD600UL
#endif
