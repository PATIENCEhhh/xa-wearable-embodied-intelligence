"""编译实际抬臂规划模块及原S曲线算法；不访问硬件。"""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("需要主机GCC")
with tempfile.TemporaryDirectory(prefix="arm-lift-") as tmp:
    binary = Path(tmp) / "arm_lift.exe"
    subprocess.run([gcc, "-std=c99", "-Wall", "-Wextra", "-Werror", "-O2",
                    "-I", str(root / "tools/test/stubs"),
                    "-I", str(root / "firmware/m33/applications"),
                    str(root / "firmware/m33/applications/control/rehab_arm_lift.c"),
                    str(root / "firmware/m33/applications/control/rehab_scurve.c"),
                    str(root / "tools/test/arm_lift_test.c"), "-lm", "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
