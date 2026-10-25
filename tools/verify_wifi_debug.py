"""实机验收 Wi-Fi 诊断；仅发送白名单查询和无效请求，不使能电机。"""
import argparse
import json
import socket
import time
import urllib.error
import urllib.request
from ican_fixed_actions import ACTIONS, supported_actions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.5.182")
    parser.add_argument("--motors-off", action="store_true", help="要求电机断电查询均返回失败")
    args = parser.parse_args()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    base = f"http://{args.host}:8088"

    def request(path, method="GET"):
        try:
            response = opener.open(urllib.request.Request(base + path, method=method), timeout=6)
        except urllib.error.HTTPError as error:
            response = error
        with response:
            return response.code, response.read()

    def status():
        code, body = request("/api/status")
        assert code == 200
        data = json.loads(body)
        assert data.get("service")=="arm-wifi-debug-v5" and supported_actions(data.get("motion_profile"))
        assert data["ok"] and data["wifi_ready"] and data["m33_binding"] and data["queries_read_only"] and data["local_arming_required"] and not data["auth_ready"], data
        return data

    def raw(parts, delay=0.0):
        with socket.create_connection((args.host, 8088), timeout=6) as connection:
            for part in parts:
                connection.sendall(part)
                if delay:
                    time.sleep(delay)
            data = b""
            while True:
                try:
                    block = connection.recv(4096)
                except ConnectionResetError:
                    # 超长头被拒绝时，关闭含未读数据的 TCP 连接可能收到 RST。
                    return data
                if not block:
                    return data
                data += block

    first = status()
    time.sleep(1.2)
    second = status()
    assert second["m55_tick"] > first["m55_tick"] and second["m33_polls"] > first["m33_polls"]
    print("PASS 双核运行计数递增:", json.dumps(second))
    code, page = request("/")
    assert code == 200 and "机械臂无线诊断" in page.decode("utf-8")
    print("PASS 浏览器页面")
    for path, method, expected in [("/api/motor/3/id", "GET", 404),
                                    ("/api/motor/4/enable", "GET", 404),
                                    ("/api/motor/4/id/extra", "GET", 404),
                                    ("/api/status", "POST", 405),
                                    ("/api/bench/5/curl", "POST", 423),
                                    ("/api/bench/5/trial", "POST", 405),
                                    ("/api/bench/5/curl", "GET", 405),
                                    ("/api/bench/5/stop", "GET", 405),
                                    ("/api/bench/4/status", "GET", 404)]:
        assert request(path, method)[0] == expected, (path, method)
    for action in supported_actions(second["motion_profile"]):
        path = ACTIONS[action]["path"]
        assert request(path, "POST")[0] == 423, path
        assert request(path, "GET")[0] == 405, path
        assert request(path + "/extra")[0] == 404, path
    fragmented = raw([b"GET /api/sta", b"tus HTTP/1.1\r\nHost: board\r\n", b"\r\n"], 0.05)
    assert fragmented.startswith(b"HTTP/1.1 200")
    rejected = raw([b"GET / HTTP/1.1\r\nX:" + b"a" * 1100])
    assert not rejected or rejected.startswith(b"HTTP/1.1 400")
    with socket.create_connection((args.host, 8088), timeout=6) as connection:
        connection.sendall(b"GET /api/status HTTP/1.1\r\n")
        started = time.monotonic()
        assert connection.recv(1024) == b""
        assert time.monotonic() - started < 4.5
    after_invalid = status()
    assert after_invalid["request_seq"] == second["request_seq"]
    print("PASS 拆包、超长头、慢客户端、非法请求；未提交电机请求")
    previous = after_invalid["done_seq"]
    for joint in (4, 5, 6):
        for operation in ("id", "position", "velocity", "mode"):
            code, body = request(f"/api/motor/{joint}/{operation}")
            data = json.loads(body)
            assert code == 200 and data["seq"] == previous + 1, data
            assert data["joint"] == joint and data["ok"] == (data["result"] == 0), data
            if args.motors_off:
                assert not data["ok"], data
            previous = data["seq"]
            print("PASS 查询完成:", joint, operation, json.dumps(data))
            time.sleep(0.5)
    final = status()
    assert final["request_seq"] == final["done_seq"] == previous
    assert final["m33_polls"] > second["m33_polls"]
    print("PASS 全部验收完成:", json.dumps(final))
    for endpoint,fields in (("support",("origin4_rad","origin6_rad","phase","failed_joint","failure_phase")),
                            ("failure",("failure_state","first_error","last_index","enable_attempts","target_attempts")),
                            ("trip",("reason_bits","sample_available","position_rad","velocity_rad_s","origin_rad","enable_attempt_mask"))):
        time.sleep(0.5)
        code,body=request("/api/bench/5/"+endpoint)
        data=json.loads(body)
        assert code==200 and data.get("ok") and all(key in data for key in fields),data
        if endpoint=="trip":
            assert data["reason_bits"]==0 and not data["sample_available"] and data["enable_attempt_mask"]==0,data
            assert data["position_rad"] is None and data["velocity_rad_s"] is None and data["origin_rad"] is None,data
        print("PASS 三轴只读诊断:",endpoint,json.dumps(data))


if __name__ == "__main__":
    main()
