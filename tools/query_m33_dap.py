"""通过受限DAP诊断邮箱查询原电机协议；不使能、不写参数、不发送运动目标。"""
import argparse
import re
import struct
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("operation", choices=["id", "position", "velocity", "mode"])
parser.add_argument("joint", type=int, choices=[4, 5, 6])
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
elf = root / "firmware/m33/rt-thread.elf"
symbols = subprocess.check_output(["arm-none-eabi-nm", "--defined-only", str(elf)], text=True)
match = re.search(r"^([0-9a-fA-F]+)\s+\w\s+g_m33_dap_diag$", symbols, re.M)
if not match:
    raise SystemExit("ELF没有诊断入口；先构建并烧录匹配固件。")
address = int(match[1], 16)
opcode = {"id": 1, "position": 2, "velocity": 3, "mode": 4}[args.operation]
home = root / ".restore-tools/openocd/openocd"
generated = root / "firmware/m33/libs/TARGET_APP_KIT_PSE84_EVAL_EPC2/config/GeneratedSource"
# 仅写入审核过的邮箱请求字段；地址由当前ELF确定，并先核对固件魔数。
commands = f"""
init
targets cat1d.cm33
if {{[lindex [read_memory {address} 32 1] 0] != 0x44415031}} {{ error "diagnostic magic mismatch" }}
set prior [lindex [read_memory {address+4} 32 1] 0]
set done [lindex [read_memory {address+16} 32 1] 0]
if {{$prior != $done}} {{ error "previous request still pending" }}
if {{$prior >= 0xFFFFFFFE}} {{ error "sequence exhausted" }}
set seq [expr {{$prior+1}}]
mww {address+8} {opcode}
mww {address+12} {args.joint}
mww {address+4} $seq
set finished 0
for {{set i 0}} {{$i < 25}} {{incr i}} {{
    sleep 100
    if {{[lindex [read_memory {address+16} 32 1] 0] == $seq}} {{ set finished 1; break }}
}}
echo DIAG_RESULT
mdw {address} 16
if {{!$finished}} {{ error "main loop did not complete diagnostic request" }}
shutdown
"""
command = [str(home / "bin/openocd.exe"), "-s", str(home / "scripts"), "-s", str(generated),
           "-f", "interface/kitprog3.cfg", "-c", "transport select swd", "-c",
           "set ENABLE_ACQUIRE 0; set ENABLE_CM55 0; set DEVICE PSE84xGxS2",
           "-f", "target/infineon/pse84.cfg", "-c", commands]
result = subprocess.run(command, capture_output=True, text=True, errors="replace", timeout=30)
output = result.stdout + result.stderr
print(output)
if result.returncode:
    raise SystemExit(result.returncode)
memory = {}
for line in output.splitlines():
    row = re.match(r"0x([0-9a-fA-F]+):\s+((?:[0-9a-fA-F]{8}\s*)+)$", line.strip())
    if row:
        base = int(row[1], 16)
        for i, word in enumerate(row[2].split()):
            memory[base + 4*i] = int(word, 16)
values = [memory[address + 4*i] for i in range(16)]
code = struct.unpack("<i", struct.pack("<I", values[6]))[0]
print(f"查询={args.operation} 槽位={args.joint} 请求={values[1]} 完成={values[4]} 返回={code}")
if code != 0:
    raise SystemExit("未获得匹配回复；超时不代表电机损坏。")
print(f"回复电机ID={values[9]} 回复tick={values[14]}")
if args.operation == "id":
    print(f"设备UID=0x{values[13]:08X}{values[12]:08X}")
elif args.operation == "mode":
    print(f"运行模式原始值={values[11]}")
else:
    value = struct.unpack("<f", struct.pack("<I", values[11]))[0]
    print(f"参数0x{values[10]:04X}原始值={value}（电机协议参数，未换算关节坐标）")
