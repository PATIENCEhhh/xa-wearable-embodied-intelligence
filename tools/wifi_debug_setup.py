"""首次通过 KitProg3 将 Wi-Fi 凭据临时写入 M55 RAM；不保存密码。"""
import argparse
import getpass
from pathlib import Path
import socket
import struct
import subprocess
import time

from elftools.elf.elffile import ELFFile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ssid", required=True)
    parser.add_argument("--serial", default="1C161868022E2400")
    args = parser.parse_args()
    ssid = args.ssid.encode("utf-8")
    password = getpass.getpass("Wi-Fi password (not saved): ").encode("utf-8")
    if not 1 <= len(ssid) <= 32 or not len(password) <= 64:
        parser.error("SSID 必须为 1..32 字节，密码不超过 64 字节")
    root = Path(__file__).resolve().parents[1]
    with (root / "firmware/m55/rt-thread.elf").open("rb") as handle:
        elf = ELFFile(handle)
        address = elf.get_section_by_name(".symtab").get_symbol_by_name("g_wifi_debug_setup")[0]["st_value"]
    base = root / ".restore-tools/openocd/openocd"
    command = [str(base / "bin/openocd.exe"), "-s", str(base / "scripts"), "-s",
               str(root / "firmware/m33/libs/TARGET_APP_KIT_PSE84_EVAL_EPC2/config/GeneratedSource"),
               "-f", "interface/kitprog3.cfg", "-c", f"adapter serial {args.serial}; transport select swd",
               "-c", "bindto 127.0.0.1; tcl_port 6666; telnet_port disabled; gdb_port disabled",
               "-c", "set ENABLE_ACQUIRE 0; set ENABLE_CM55 1; set DEVICE PSE84xGxS2",
               "-f", "target/infineon/pse84.cfg", "-c", "init; targets cat1d.cm55"]
    # 密码不出现在命令行、临时文件或 OpenOCD 日志中，仅通过本机 RPC 写 RAM。
    process = subprocess.Popen(command, cwd=root, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        connection = None
        for _ in range(100):
            if process.poll() is not None:
                raise RuntimeError("OpenOCD 启动失败；检查 KitProg3 连接或端口占用")
            try:
                connection = socket.create_connection(("127.0.0.1", 6666), timeout=2)
                break
            except OSError:
                time.sleep(0.1)
        if connection is None:
            raise TimeoutError("OpenOCD 未就绪")
        with connection:
            def rpc(command):
                connection.sendall(command.encode("ascii") + b"\x1a")
                result = b""
                while not result.endswith(b"\x1a"):
                    block = connection.recv(4096)
                    if not block:
                        raise ConnectionError("调试器断开")
                    result += block
                return result[:-1].decode("ascii").strip()

            def read(offset):
                value = rpc(f"read_memory {address + offset} 32 1")
                return int(value, 0)

            def write(offset, value):
                response = rpc(f"mww {address + offset} {value}")
                if response:
                    raise RuntimeError("RAM 写入失败（省略含凭据的诊断）")

            request, done = read(0), read(4)
            if request != done:
                raise RuntimeError("已有入网请求在处理，未覆盖")
            payload = (ssid.ljust(33, b"\0") + password.ljust(65, b"\0")).ljust(100, b"\0")
            for offset in range(0, len(payload), 4):
                write(16 + offset, struct.unpack_from("<I", payload, offset)[0])
            del payload, password
            seq = request + 1
            write(0, seq)
            deadline = time.monotonic() + 55
            while time.monotonic() < deadline:
                if read(4) == seq:
                    result = read(8)
                    if result & 0x80000000:
                        result -= 0x100000000
                    cleared = all(int(value, 0) == 0 for value in
                                  rpc(f"read_memory {address + 49} 8 65").split())
                    print(f"join_result={result}; temporary_password_cleared={cleared}")
                    print("DHCP 就绪可能稍后到达；用 Wi-Fi 状态接口继续验收。")
                    return 0 if result == 0 and cleared else 1
                time.sleep(0.5)
            raise TimeoutError("板端入网请求未完成；未重复发布")
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
