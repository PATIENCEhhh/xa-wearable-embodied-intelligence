"""核对无线诊断绑定与当前 M33 ELF，避免重新链接后误用旧邮箱地址。"""
import re
import struct
from pathlib import Path
from elftools.elf.elffile import ELFFile

root = Path(__file__).resolve().parents[1]
header = (root / "firmware/m55/applications/wifi_debug_binding.h").read_text(encoding="utf-8")
defines = dict(re.findall(r"#define (WIFI_DEBUG_\w+) (0x[0-9A-Fa-f]+|\d+)[UuLl]*", header))
with (root / "firmware/m33/rt-thread.elf").open("rb") as handle:
    elf = ELFFile(handle)
    symtab = elf.get_section_by_name(".symtab")
    mailbox = symtab.get_symbol_by_name("g_m33_dap_diag")[0]
    descriptor = symtab.get_symbol_by_name("g_m33_wifi_binding")[0]
    address = descriptor["st_value"]
    size = descriptor["st_size"]
    data = None
    for segment in elf.iter_segments():
        offset = address - segment["p_vaddr"]
        if 0 <= offset and offset + size <= segment["p_filesz"]:
            data = segment.data()[offset:offset + size]
            break
    assert data is not None and size == 32 and address % 32 == 0, "描述符缺失或对齐错误"
    assert struct.unpack("<8I", data) == (0x57444231, 11, mailbox["st_value"], 64,
                                         0xe7c19011, 0x326a9d41, 0x5f21ab88, 0x9d643207), "ABI 描述符内容错误"
    assert mailbox["st_size"] == 64 and mailbox["st_value"] % 32 == 0, "邮箱大小或缓存行对齐错误"
    expected = {
        "WIFI_DEBUG_MAILBOX": mailbox["st_value"],
        "WIFI_DEBUG_DESCRIPTOR": address,
        "WIFI_DEBUG_CURL_UI": symtab.get_symbol_by_name("g_ican_curl_ui")[0]["st_value"],
    }
    ui = symtab.get_symbol_by_name("g_ican_curl_ui")[0]
    assert ui["st_size"] == 64 and ui["st_value"] % 32 == 0, "UI邮箱大小或对齐错误"
    for name, value in expected.items():
        assert int(defines[name], 0) == value, f"{name} 失配：应为 {value:#x}"
        print(f"{name}={value:#x} 通过")
