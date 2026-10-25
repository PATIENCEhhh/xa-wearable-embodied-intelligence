"""iCAN三轴演示接口：复用已验收的板端流程和DAP单次授权工具。"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from wifi_arm_debug import query
from ican_fixed_actions import ACTIONS, supported_actions


class DemoError(RuntimeError):
    """携带运行记录的接口错误；异常不代表实物已经停止。"""

    def __init__(self, message, report=None):
        super().__init__(message)
        self.report = report


class ArmDemo:
    """同步接口，仅支持4/6保持、5号先屈曲再伸展一轮。

    创建对象和查询均不使能电机。start_action/start_curl需要明确现场准备确认，
    并委托原工具通过有线DAP写一次性RAM授权；不复位、不烧录、不重试动作。
    同一开发板只能由一个客户端控制，运行中不要另发电机参数查询。
    """

    def __init__(self, host="192.168.5.182", serial="1C161868022E2400"):
        self.host = host
        self.serial = serial

    def _query(self, operation):
        result = query(self.host, operation, None)
        if not result.get("ok"):
            raise DemoError(f"板端{operation}查询失败", result)
        return result

    def status(self):
        """只读查询。可启动仅指软件条件，还需要现场准备和DAP连接。"""
        service = self._query("status")
        time.sleep(0.55)  # 诊断邮箱必须串行使用，不并发发送请求。
        bench = self._query("bench")
        compatible = (
            service.get("service") == "arm-wifi-debug-v5"
            and service.get("m33_binding") is True
            and bool(supported_actions(service.get("motion_profile")))
        )
        used = bool(bench.get("flags", 2) & 2)
        active = bench.get("active") is True
        idle = bench.get("state") == 0 and bench.get("active") is False
        return {
            "compatible": compatible,
            "supported_actions": supported_actions(service.get("motion_profile")) if compatible else [],
            "active": active,
            "cycle_consumed": used,
            "can_request_start": bool(compatible and idle and not used),
            "requires_power_off_reset": bool(not active and (used or not idle)),
            "physical_stop_verified": False,
            "service": service,
            "bench": bench,
        }

    def stop(self):
        """请求三轴STOP。返回接收结果，实物静止仍须现场确认。

        STOP会锁存并消耗本次启动机会，解除主动保持；它不是暂停/继续。
        """
        return self._query("stop")

    def start_curl(self, *, supported_and_clear=False, log_path=None):
        """阻塞执行一轮并保存完成/清理结果；不接受角度、电流或循环次数。

        supported_and_clear代表现场已确认未穿戴、有机械支撑、行程无阻挡、
        电机已上电。软件无法替代现场确认。失败时不重试、不改保护参数。
        """
        return self.start_action("elbow-full", supported_and_clear=supported_and_clear, log_path=log_path)

    def start_action(self, action, *, supported_and_clear=False, log_path=None):
        """选择固化动作ID；不会把未支持的动作退化成另一动作。"""
        if action not in ACTIONS:
            raise DemoError("未知固定动作ID")
        if supported_and_clear is not True:
            raise DemoError("先确认未穿戴、机械支撑、行程无阻挡及电机上电")
        before = self.status()
        if not before["can_request_start"]:
            raise DemoError("当前不满足单次启动条件；已使用时须断动力后复位", before)
        available = before.get("supported_actions", [])
        if action not in available:
            raise DemoError("当前固件未提供该动作，需先安装配套版本", before)

        # 先建立日志，避免动作之后才发现输出路径不可写；禁止覆盖历史证据。
        handle = None
        if log_path is not None:
            handle = Path(log_path).open("x", encoding="utf-8")
        report = {"preflight": before, "action": action, "completed_cycle": None,
                  "physical_stop_verified": False, "log_path": str(log_path) if log_path else None}
        command = [sys.executable, "-X", "utf8", "-u",
                   str(Path(__file__).with_name("bench5_trial.py")),
                   "--host", self.host, "--serial", self.serial, "--action", action, "--supported-and-clear"]
        try:
            try:
                run = subprocess.run(command, capture_output=True, text=True,
                                     encoding="utf-8", errors="replace", timeout=85)
                output = run.stdout + run.stderr
                report["tool_exit_code"] = run.returncode
            except subprocess.TimeoutExpired as error:
                # 子进程超时绝不再发启动。板端也有独立时限；尽力另发STOP。
                report["host_timeout"] = True
                if handle:
                    for partial in (error.stdout, error.stderr):
                        if partial:
                            handle.write(partial.decode("utf-8", errors="replace")
                                         if isinstance(partial, bytes) else partial)
                    handle.flush()
                try:
                    report["timeout_stop_request"] = self.stop()
                except Exception as stop_error:
                    report["timeout_stop_error"] = str(stop_error)
                raise DemoError("主机等待超时；托稳机构并在现场关闭电机动力", report) from error
            except KeyboardInterrupt:
                # Ctrl+C中断等待也尽力停止，不依赖子进程是否完成finally清理。
                try:
                    self.stop()
                except Exception:
                    pass
                raise DemoError("主机等待已中断；托稳机构并在现场关闭电机动力", report)
            if handle:
                handle.write(output)
                handle.flush()
            # 单独保存DONE，不能让工具末尾追加STOP产生的取消码覆盖完成证据。
            fields = {"trial_request": "accepted_request", "trial_result": "completed_cycle",
                      "trial_detail": "detail", "support": "support", "failure": "failure",
                      "trip": "trip", "final_stop_request": "cleanup_stop_request"}
            for line in output.splitlines():
                name, separator, payload = line.partition(": ")
                if separator and name in fields:
                    try:
                        report[fields[name]] = json.loads(payload)
                    except ValueError as error:
                        raise DemoError("原试动工具输出无法解析；请检查运行日志", report) from error
            # STOP接收与处理有时差，有限等待只读状态；不补发动作、不复位。
            after = None
            for _ in range(4):
                time.sleep(0.55)
                after = self.status()
                if not after["active"] and after["service"].get("auth_ready") is False:
                    break
            report["postcheck"] = after
            done = report["completed_cycle"] or {}
            report["completed"] = bool(
                run.returncode == 0 and done.get("state") == 13
                and done.get("run_result") == 0 and done.get("active") is False
                and done.get("stop_send_result") == 0
                and not after["active"] and after["service"].get("auth_ready") is False)
            if not report["completed"]:
                raise DemoError("本轮未通过软件完成核对；检查日志和实物，不自动重试", report)
            return report
        finally:
            if handle:
                handle.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=["list", "status", "start", "stop"])
    parser.add_argument("--host", default="192.168.5.182")
    parser.add_argument("--serial", default="1C161868022E2400")
    parser.add_argument("--supported-and-clear", action="store_true")
    parser.add_argument("--log", help="启动运行日志路径，必须不存在")
    parser.add_argument("--action", choices=list(ACTIONS), default="elbow-full")
    args = parser.parse_args()
    if args.operation == "start" and not args.supported_and_clear:
        parser.error("start需要--supported-and-clear现场准备确认")
    if args.operation != "start" and (args.log or args.supported_and_clear):
        parser.error("--log和--supported-and-clear只用于start")
    client = ArmDemo(args.host, args.serial)
    try:
        if args.operation == "list":
            result = {"actions": ACTIONS, "note": "离线目录；实际支持以status为准，新动作待实物验收"}
        elif args.operation == "start":
            if not args.log:
                parser.error("start需要--log保存本轮独立运行证据")
            result = client.start_action(args.action, supported_and_clear=True, log_path=args.log)
        else:
            result = getattr(client, args.operation)()
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (DemoError, OSError, ValueError) as error:
        print(json.dumps({"ok": False, "error": str(error),
                          "report": getattr(error, "report", None)},
                         ensure_ascii=False, indent=2, allow_nan=False))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
