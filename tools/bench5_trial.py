"""通过本机DAP临时授权，4/6保持本轮位置，5号先屈曲后伸展一轮；不自动重试。"""
import argparse
import json
from pathlib import Path
import secrets
import socket
import struct
import subprocess
import time
import urllib.error
import urllib.request

from elftools.elf.elffile import ELFFile
from ican_fixed_actions import ACTIONS, supported_actions, binding_words

ROOT = Path(__file__).resolve().parents[1]


def symbol(core, name):
    with (ROOT / f"firmware/{core}/rt-thread.elf").open("rb") as handle:
        result = ELFFile(handle).get_section_by_name(".symtab").get_symbol_by_name(name)
        if not result:
            raise RuntimeError(f"配套ELF缺少符号：{name}")
        return result[0]["st_value"]


def arm_session(token, serial, motion_profile):
    """令牌仅驻留进程和板卡RAM，DAP不暂停、不复位、不烧录运行中的板卡。"""
    arm = symbol("m33", "g_bench5_arm")
    descriptor = symbol("m33", "g_m33_wifi_binding")
    mailbox = symbol("m33", "g_m33_dap_diag")
    expected = binding_words(motion_profile, mailbox)
    # 新ELF不能授权仍在运行的旧镜像；先核对本地描述符，避免按错地址写RAM。
    with (ROOT / "firmware/m33/rt-thread.elf").open("rb") as handle:
        elf = ELFFile(handle)
        local = None
        for segment in elf.iter_segments():
            offset = descriptor - segment["p_vaddr"]
            if 0 <= offset and offset + 32 <= segment["p_filesz"]:
                local = list(struct.unpack("<8I", segment.data()[offset:offset+32]))
                break
        if local != expected:
            raise RuntimeError("本地ELF与运行中的动作配置不匹配；未授权、未发送动作")
    token_address = symbol("m55", "g_wifi_trial_token")
    base = ROOT / ".restore-tools/openocd/openocd"
    command = [str(base / "bin/openocd.exe"), "-s", str(base / "scripts"), "-s",
               str(ROOT / "firmware/m33/libs/TARGET_APP_KIT_PSE84_EVAL_EPC2/config/GeneratedSource"),
               "-f", "interface/kitprog3.cfg", "-c", f"adapter serial {serial}; transport select swd",
               "-c", "bindto 127.0.0.1; tcl_port 6666; telnet_port disabled; gdb_port disabled",
               "-c", "set ENABLE_ACQUIRE 0; set ENABLE_CM55 1; set DEVICE PSE84xGxS2",
               "-f", "target/infineon/pse84.cfg", "-c", "init; targets cat1d.cm33"]
    # 本次令牌不进入OpenOCD命令行或日志；只在本机RPC连接中传输。
    process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    connection = None
    rpc = None
    try:
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("DAP未连接，或本机调试端口被占用；没有发送试动")
            try:
                connection = socket.create_connection(("127.0.0.1", 6666), timeout=2)
                break
            except OSError:
                time.sleep(0.1)
        if connection is None:
            raise TimeoutError("本机DAP服务未就绪")

        def rpc(command):
            connection.sendall(command.encode("ascii") + b"\x1a")
            result = b""
            while not result.endswith(b"\x1a"):
                block = connection.recv(4096)
                if not block:
                    raise ConnectionError("DAP断开")
                result += block
            return result[:-1].decode("ascii").strip()

        def write(address, value):
            if rpc(f"mww {address} {value}"):
                raise RuntimeError("临时授权RAM写入失败（省略令牌）")

        actual = [int(x, 0) for x in rpc(f"read_memory {descriptor} 32 8").split()]
        if actual != expected:
            raise RuntimeError("运行中M33与配套ELF不匹配；没有授权")
        write(arm, 0)
        rpc("targets cat1d.cm55")
        payload = token.encode("ascii").ljust(64, b"\0")
        for offset in range(0, 64, 4):
            write(token_address + offset, struct.unpack_from("<I", payload, offset)[0])
        rpc("targets cat1d.cm33")
        write(arm, 0x42354131)
    except BaseException:
        # 部分写入后失败也尽力撤销；不重复发布动作。
        if connection and rpc:
            try:
                rpc("targets cat1d.cm33")
                rpc(f"mww {arm} 0")
            except Exception:
                pass
        raise
    finally:
        if connection:
            connection.close()
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.5.182")
    parser.add_argument("--serial", default="1C161868022E2400")
    parser.add_argument("--action", choices=list(ACTIONS), default="elbow-full")
    gate = parser.add_mutually_exclusive_group(required=True)
    gate.add_argument("--supported-and-clear", action="store_true",
                      help="三轴协同弯举；确认未穿戴、上臂/前臂有机械支撑、行程无阻挡、三轴已上电")
    gate.add_argument("--motors-off-test", action="store_true",
                      help="电机动力关闭，仅验收授权、重放拒绝、无回复时退出；会消耗本次试动")
    args = parser.parse_args()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(path, method="GET", key=None):
        headers = {"X-Bench-Key": key} if key else {}
        req = urllib.request.Request(f"http://{args.host}:8088{path}", method=method, headers=headers)
        try:
            response = opener.open(req, timeout=5)
        except urllib.error.HTTPError as error:
            response = error
        with response:
            return response.code, json.load(response)

    code, service = request("/api/status")
    motion_profile = service.get("motion_profile")
    if code != 200 or service.get("service") != "arm-wifi-debug-v5" or not service.get("m33_binding") or args.action not in supported_actions(motion_profile):
        raise RuntimeError(f"配套固件/网络未就绪：{service}")
    action_path = ACTIONS[args.action]["path"]
    print("selected_action:", json.dumps({"id": args.action, **ACTIONS[args.action]}, ensure_ascii=False))
    time.sleep(0.5)
    _, state = request("/api/bench/5/status")
    if not state.get("ok") or state.get("active") or state.get("flags", 2) & 2 or state.get("state") != 0:
        raise RuntimeError(f"不是本次启动的空闲状态，请断动力后复位准备：{state}")
    token = secrets.token_hex(16)
    arm_session(token, args.serial, motion_profile)
    time.sleep(0.5)
    # DAP写入只代表允许一次请求；发错令牌必须在HTTP层拒绝，不能推进M33邮箱。
    _, before = request("/api/status")
    code, data = request(action_path, "POST", "0" * 32 if token != "0" * 32 else "1" * 32)
    _, after = request("/api/status")
    if code != 403 or before["request_seq"] != after["request_seq"]:
        raise RuntimeError("错误令牌拒绝验收失败，没有发送有效试动请求")
    print("PASS invalid token rejected without M33 request")
    sent = False
    try:
        sent = True  # 即使网络超时也不重发；M33本地时限负责退出。
        code, accepted = request(action_path, "POST", token)
        print("trial_request:", json.dumps(accepted, ensure_ascii=False))
        if code != 200 or not accepted.get("ok"):
            raise RuntimeError("试动未被接收；不会自动重试")
        deadline = time.monotonic() + 58
        while time.monotonic() < deadline:
            time.sleep(0.5)
            _, state = request("/api/bench/5/status")
            if state.get("ok") and not state.get("active"):
                break
        else:
            raise TimeoutError("试动未在预期时间结束")
        print("trial_result:", json.dumps(state, ensure_ascii=False))
        time.sleep(0.5)
        _, detail = request("/api/bench/5/detail")
        print("trial_detail:", json.dumps(detail, ensure_ascii=False))
        for endpoint in ("support","failure","trip"):
            time.sleep(0.5)
            _, diagnostic=request("/api/bench/5/"+endpoint)
            print(endpoint+":",json.dumps(diagnostic,ensure_ascii=False))
        _, before = request("/api/status")
        code, _ = request(action_path, "POST", token)
        _, after = request("/api/status")
        if code != 423 or before["request_seq"] != after["request_seq"]:
            raise RuntimeError("重复请求拒绝验收失败")
        print("PASS consumed token cannot replay or advance M33 mailbox")
        if args.motors_off_test:
            assert state["state"] == 14 and state["run_result"] in (-22, -23), state
            assert state["stop_send_result"] != 0 and detail["stop_attempts"] == 3, (state, detail)
            assert detail["origin_rad"] == detail["target_rad"] == 0, detail
            print("PASS motors-off preflight fails before enable/target; bounded STOP failure reported")
        elif state["state"] != 13 or state["run_result"] != 0:
            raise RuntimeError("试动未完成；保持机械支撑，检查结果，不自动加大电流或行程")
        return 0
    finally:
        if sent:
            # 额外STOP不等于物理制动确认；网络失败时现场仍必须能够切断动力。
            try:
                print("final_stop_request:", json.dumps(request("/api/bench/5/stop", "POST")[1]))
            except Exception as error:
                print("STOP request failed:", type(error).__name__, "; close motor power locally")


if __name__ == "__main__":
    raise SystemExit(main())
