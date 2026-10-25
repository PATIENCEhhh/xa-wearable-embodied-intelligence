"""通过Wi-Fi查询，或请求4/5/6三轴停止；查询默认只读，不依赖DAP、串口或ROS。"""
import argparse
import json
import struct
import time
import urllib.error
import urllib.request


def query(host, operation, joint):
    path = "/api/status" if operation == "status" else (
        f"/api/bench/5/{'status' if operation=='bench' else operation}"
        if operation in ("bench","stop","detail","support","failure","trip") else f"/api/motor/{joint}/{operation}")
    # 局域网直连，不使用电脑上的 HTTP 代理；错误响应也保留板端具体原因。
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        request=urllib.request.Request(f"http://{host}:8088{path}",method="POST" if operation=="stop" else "GET")
        with opener.open(request, timeout=5) as response:
            result = json.load(response)
    except urllib.error.HTTPError as error:
        result = json.loads(error.read())
        result["http_status"] = error.code
    if result.get("ok") and operation in ("position", "velocity"):
        result["value_raw"] = struct.unpack("<f", struct.pack("<I", result["value_bits"]))[0]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", nargs="?", default="status",
                        choices=["status", "id", "position", "velocity", "mode", "bench", "detail", "support", "failure", "trip", "stop"])
    parser.add_argument("joint", nargs="?", type=int, choices=[4, 5, 6])
    parser.add_argument("--host", default="192.168.5.182", help="开发板 IPv4 地址")
    parser.add_argument("--count", type=int, default=1, help="连续读取次数")
    parser.add_argument("--interval", type=float, default=1.0, help="读取间隔秒数，至少 0.5 秒")
    args = parser.parse_args()
    if args.operation not in ("status","bench","detail","support","failure","trip") and args.joint is None:
        parser.error("电机查询需要指定槽位 4、5 或 6")
    if args.operation in ("bench","stop","detail","support","failure","trip") and args.joint not in (None,5):
        parser.error("三轴演示通过5号入口统一管理，入口槽位只能为5")
    if args.operation=="stop" and args.count!=1:
        parser.error("停止不自动重复；先查bench状态")
    if args.count < 1 or args.interval < 0.5:
        parser.error("次数必须为正数，间隔至少 0.5 秒")
    failed = False
    for index in range(args.count):
        try:
            result = query(args.host, args.operation, args.joint)
        except (OSError, ValueError) as error:
            result = {"ok": False, "error": str(error)}
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        failed |= not result.get("ok", False)
        if index + 1 < args.count:
            time.sleep(args.interval)
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
