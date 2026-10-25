# iCAN三轴演示功能接口

日期：2026-10-16。

后续扩展：已准备四个固定动作的v6双核镜像及`start_action(action, ...)`选择入口，目录和验收状态见[iCAN固定动作菜单-v6.md](iCAN固定动作菜单-v6.md)。新版尚未烧录时，实际固件仅提供原`elbow-full`动作；以下`start_curl()`接口继续表示完整弯举。

第一项封装为已完成实物运行的三轴配合弯举：4、6号保持本轮启动位置，5号先屈曲再伸展。实现位于`tools/ican_arm_demo.py`，在现有Wi-Fi接口和`bench5_trial.py`外提供统一Python调用方式；使用当前v5固件，无需新增烧录。后续模式可在功能层继续扩展，本版只提供这一项已跑通动作。

## 调用接口

| Python接口 | 作用 | 返回与限制 |
|---|---|---|
| `ArmDemo(host, serial)` | 建立客户端对象 | 无硬件操作 |
| `status()` | 只读获取服务和动作状态 | 含兼容性、活动状态、单次机会是否消耗、软件启动条件 |
| `start_curl(supported_and_clear=True, log_path=...)` | 同步执行一轮 | 返回独立的完成记录、保持轴诊断、清理STOP和后查状态；失败抛出`DemoError` |
| `stop()` | 请求4/5/6三轴STOP | 返回请求接收结果；锁存停止，解除主动保持，不是暂停 |

调用代码可将仓库`tools`加入Python模块搜索路径，然后导入：

```python
from ican_arm_demo import ArmDemo, DemoError

arm = ArmDemo()
state = arm.status()
print(state["can_request_start"])

# 仅在现场完成准备、需要实际执行时调用，日志文件必须不存在。
try:
    result = arm.start_curl(
        supported_and_clear=True,
        log_path="ican-curl-run-001.log",
    )
    print(result["completed"], result["completed_cycle"])
except DemoError as error:
    print(str(error), error.report)
```

`supported_and_clear=True`表示调用方已完成现场确认：机构未穿戴、上臂/前臂有可靠机械支撑、行程无阻挡、电机已上电。仅程序赋值不能证明这些物理条件。返回`completed=True`表示板端报告完成且后查已结束、无授权；`physical_stop_verified`固定为false，实物静止及姿态仍由现场判断。

## 命令行

以下从仓库根目录执行。状态查询只读，无需DAP：

```powershell
& .restore-tools/Scripts/python.exe -X utf8 tools/ican_arm_demo.py status
```

启动需要当前有线DAP、Wi-Fi在线及本次开发板启动尚未使用的单次机会；仅在已完成现场准备并要执行动作时运行：

```powershell
& .restore-tools/Scripts/python.exe -X utf8 tools/ican_arm_demo.py start --supported-and-clear --log ican-curl-run-001.log
```

停止请求：

```powershell
& .restore-tools/Scripts/python.exe -X utf8 tools/ican_arm_demo.py stop
```

停止请求不是机械制动，STOP后主动保持会解除。停止锁存或一轮完成后，本次启动机会不能重放；需要现场关闭动力后复位，再重新确认上电。接口不会复位开发板或清除单次使用标记。

## 实现与验收范围

启动仍完整委托原`bench5_trial.py`：核对固件、通过DAP写一次性RAM授权、拒绝错误令牌、只发一次有效启动、等待板端结束、诊断、验证重放拒绝及追加STOP。封装层不接收新的角度、电流、速度或循环次数，不调整已跑通参数。主机等待超时或中断时尽力发送STOP，不重试启动。

完成与清理分别保存：`completed_cycle`中的state13/run_result0才是原动作完成记录，`cleanup_stop_request`和`postcheck`可能包含追加STOP产生的取消状态-26。该清理状态不覆盖原DONE，也不能把STOP接收误当物理静止。

单个开发板同一时刻由一个客户端控制。运行中仅由原工具监控流程，不额外并发查询电机参数。`status()`是先后读取的两个状态快照，`can_request_start`表示当时的软件条件，实际启动仍由原工具和固件再次准入检查。

4/6初始参考详见`4和6号电机初始位置参考.md`及`calibration/ican-j46-initial-reference-2026-10-16.json`。用户接受复测6号约2°初始姿态差，原始两组值均保留；本接口仍采用实时启动位置保持，没有读取JSON后自动回零，没有改变运行漂移保护。

离线验收包含接口门禁、固件不匹配拒绝、已用启动机会拒绝、单次委托、DONE与清理分离、日志防覆盖、超时/中断STOP及无自动重试；另运行原M33状态机、三轴保持及原主动反馈回归。新接口只做只读联机核对，未借本次封装再次启动电机；现有动作实物运行结果见`三轴保持与肘部弯举-v5.md`。
