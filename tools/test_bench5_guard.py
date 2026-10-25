"""在主机编译实际M33试动状态机并注入故障；不访问开发板。"""
from pathlib import Path
import shutil
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
gcc=shutil.which("gcc")
if not gcc:
    raise SystemExit("需要主机 GCC（不是 arm-none-eabi-gcc）")
with tempfile.TemporaryDirectory(prefix="bench5-test-") as tmp:
    binary=Path(tmp)/"bench5_test.exe"
    subprocess.run([gcc,"-std=c99","-Wall","-Wextra","-Werror","-O2",
                    "-I",str(root/"tools/test/stubs"),
                    "-I",str(root/"firmware/m33/applications"),
                    str(root/"firmware/m33/applications/bench5_guard.c"),
                    str(root/"firmware/m33/applications/control/rehab_curl_planner.c"),
                    str(root/"tools/test/bench5_guard_test.c"),"-lm","-o",str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
    auth=Path(tmp)/"bench5_auth.exe"
    subprocess.run([gcc,"-std=c99","-Wall","-Wextra","-Werror","-O2",
                    "-I",str(root/"firmware/m55/applications"),
                    str(root/"tools/test/bench5_auth_test.c"),"-o",str(auth)],check=True)
    subprocess.run([str(auth)],check=True)
    owner=Path(tmp)/"bench5_owner.exe"
    subprocess.run([gcc,"-std=c99","-Wall","-Wextra","-Werror","-O2",
                    "-I",str(root/"tools/test/stubs"),
                    "-I",str(root/"firmware/m33/applications"),
                    str(root/"tools/test/bench_three_teach_test.c"),
                    str(root/"firmware/m33/applications/control/rehab_arm_lift.c"),
                    str(root/"firmware/m33/applications/control/rehab_trajectory_bank.c"),
                    str(root/"firmware/m33/applications/control/rehab_scurve.c"),
                    str(root/"firmware/m33/applications/control/rehab_curl_planner.c"),
                    str(root/"firmware/m33/applications/bench5_guard.c"),
                    str(root/"firmware/m33/applications/bench_support_hold.c"),
                    str(root/"firmware/m33/applications/bench_resist.c"),
                    str(root/"firmware/m33/applications/control/rehab_resist_strategy.c"),
                    str(root/"firmware/m33/applications/control/rehab_adaptive_pid.c"),
                    str(root/"firmware/m33/applications/control/rehab_adrc.c"),"-lm","-o",str(owner)],check=True)
    subprocess.run([str(owner)],check=True)
    support=Path(tmp)/"bench_support_hold.exe"
    subprocess.run([gcc,"-std=c99","-Wall","-Wextra","-Werror","-O2",
                    "-I",str(root/"tools/test/stubs"),
                    "-I",str(root/"firmware/m33/applications"),
                    str(root/"tools/test/bench_support_hold_test.c"),
                    str(root/"firmware/m33/applications/bench_support_hold.c"),"-lm","-o",str(support)],check=True)
    subprocess.run([str(support)],check=True)
