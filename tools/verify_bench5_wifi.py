"""断动力验收三轴门禁与整组STOP失败反馈；不会发送试动或使能。"""
import argparse
import json
import time
import urllib.error
import urllib.request
from ican_fixed_actions import ACTIONS, supported_actions

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--host",default="192.168.5.182")
parser.add_argument("--motors-off",action="store_true",required=True,
                    help="现场已关闭电机动力；会发送4/5/6三轴STOP")
args=parser.parse_args()
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))

def request(path,method="GET"):
    try:
        response=opener.open(urllib.request.Request(f"http://{args.host}:8088{path}",method=method),timeout=5)
    except urllib.error.HTTPError as error:
        response=error
    with response:
        return response.code,json.load(response)

_,before=request("/api/status")
assert before.get("service")=="arm-wifi-debug-v5" and supported_actions(before.get("motion_profile"))
for path,method,expected in [
    ("/api/bench/5/curl","POST",423),
    ("/api/bench/5/trial","POST",405),
    ("/api/bench/5/trial","GET",404),
    ("/api/bench/5/stop","GET",405),
    ("/api/bench/4/stop","POST",405),
    ("/api/bench/6/status","GET",404),
    ("/api/bench/5/stop/extra","POST",405),
]:
    code,data=request(path,method)
    assert code==expected,(path,code,data)
for action in supported_actions(before["motion_profile"]):
    path=ACTIONS[action]["path"]
    assert request(path,"POST")[0]==423,path
    assert request(path,"GET")[0]==405,path
_,after=request("/api/status")
assert before["request_seq"]==after["request_seq"],(before,after)
print("PASS HTTP trial locked and invalid requests do not reach M33")
time.sleep(0.4)
code,data=request("/api/bench/5/status")
assert code==200 and data["ok"] and data["trial_locked"] and not data["active"],data
assert data["state"]==0,data
print("PASS initial bench snapshot:",json.dumps(data))
code,data=request("/api/bench/5/stop","POST")
assert code==200 and data["ok"],data
print("PASS STOP accepted (not physical-stop confirmation):",json.dumps(data))
deadline=time.monotonic()+4
while time.monotonic()<deadline:
    time.sleep(0.5)
    code,data=request("/api/bench/5/status")
    assert code==200 and data["ok"] and data["trial_locked"],data
    if not data["active"]:
        break
else:
    raise AssertionError("STOP did not finish within bounded retries")
assert data["state"]==14 and data["run_result"]==-26,data
assert data["stop_send_result"]!=0 and not data["physical_stop_verified"],data
print("PASS powered-off STOP failure reported after bounded retries:",json.dumps(data))
print("PASS Wi-Fi reachable after STOP failure:",json.dumps(request("/api/status")[1]))
