# iCAN固定动作菜单 v6

日期：2026-10-16。目标：优先展示未穿戴机构的固定动作，复用原工程控制，不依赖ROS。

当前状态：四个动作已实现，新版双核镜像已烧录并校验，离线及断动力验收通过。约30°小幅屈伸已执行一轮，板端DONE成功，无保持故障，流程结束且无授权；现场动作观察、静止及断动力仍待确认。60°、停留方案以及本版完整屈伸尚待实物验收。原完整弯举此前在v5报告三轮板端完成，不能视作v6新动作已经实物通过。

## 当前可选动作设计

| ID | 展示名称 | 顺序 | 屈曲端相对伸直参考的原始角度 | 两端驻留 |
|---|---|---|---:|---:|
| `elbow-small` | 小幅屈伸 | 屈曲→驻留→伸展→驻留 | 约30° | 各0.5秒 |
| `elbow-medium` | 中幅屈伸 | 屈曲→驻留→伸展→驻留 | 约60° | 各0.5秒 |
| `elbow-full` | 完整屈伸 | 屈曲→驻留→伸展→驻留 | 约84.14°，沿用已跑通目标 | 各0.5秒 |
| `elbow-pause` | 屈曲停留后伸展 | 屈曲→驻留→伸展→驻留 | 约60° | 各2秒 |

这是同一肘屈伸自由度的四个固定方案，不能宣传为四种不同关节动作。所有方案4、6号都保持各自在本轮使能前采集的位置，5号执行动作；每次仅一轮，成功、失败或停止都进入三轴STOP。没有新增自动循环串烧。

新30°/60°目标分别为伸直参考4.971363544加0.523598776/1.047197551rad；伸展端仍为5.021363544rad。原始角度是电机坐标，显示角度不能代替精确关节角验证。全部目标保持在已核对的肘部姿态包络内，保留0.12rad/s指令速度、原限流、250ms新鲜反馈、4/6漂移保护和原时间上限。

## 原工程固定动作核查

已查看`firmware/m33/applications/control/rehab_fixed_action.c`、原服务执行代码和配置：

| 原动作 | 源码状态 | 原运动轴 | 恢复处理 |
|---|---|---|---|
| `ELBOW_FLEX_EXTEND` | enabled=true | 5 | 复用原弯举规划与CSP接口，使用现在的两点标定 |
| `SHOULDER_PLANAR` | enabled=true | 6 | 保留恢复项；旧safe范围3.689～4.944rad与当前约1.365rad基线不一致，不直接发布旧绝对目标 |
| `COORDINATED` | enabled=true | 5＋6 | 保留恢复项；先验证6号范围、方向和独立动作，再做同步配合 |
| `SHOULDER_FORE_AFT` | enabled=false | 4 | 原配置无有效目标范围；需要补充当前机构的实测运动端点 |

上述enabled只是源码配置，不能据此称原动作在本台机构上已验证。原肘部safe范围6.226～8.038rad也不适用于现在4.971rad伸直参考，因此未启用旧固定表的绝对目标。旧工程还存在ROS名称与物理关节命名不一致的历史，后续以电机ID和现场映射验收为准。

复用范围包括原弯举规划器、旧CSP目标入口、原主动上报解码及缓存、4/6保持准备与整组STOP。新增层只选择固化目标和驻留时间，不重新实现电机协议。

## 调用

离线查看目录，不访问板卡：

```powershell
& .restore-tools/Scripts/python.exe -X utf8 tools/ican_arm_demo.py list
```

查询实际固件支持的动作：

```powershell
& .restore-tools/Scripts/python.exe -X utf8 tools/ican_arm_demo.py status
```

只有在更新配套双核镜像、断动力验收通过，并重新确认机构未穿戴、有支撑、行程无阻挡及上电后，才调用实际动作：

```powershell
& .restore-tools/Scripts/python.exe -X utf8 tools/ican_arm_demo.py start --action elbow-small --supported-and-clear --log ican-elbow-small-001.log
```

Python调用为`ArmDemo().start_action("elbow-small", supported_and_clear=True, log_path="...")`。旧`start_curl()`继续表示完整屈伸。旧固件只报告`elbow-full`，新动作不会偷偷退化成完整屈伸。新版本地ELF与旧运行配置不一致时，DAP授权前即拒绝，不按新地址写旧镜像RAM。

## 协议与版本

- 新配置`ican-fixed-j5-hold-j46-v6`，描述符ABI10，邮箱仍64字节，描述符0x240BD600、邮箱0x240BD620。
- 原POST `/api/bench/5/curl`保留op6完整屈伸。新增`curl-small`/`curl-medium`/`curl-pause`分别对应op12/13/14。
- 所有启动路径都要求原有线DAP授权和精确`X-Bench-Key`令牌，提交前消耗令牌；M33仍独立检查单次机会、STOP锁存和排队状态。
- GET不能启动，路径不接受尾随字符或任意角度参数；STOP仍为`/api/bench/5/stop`，停止4/5/6。
- 本版不提供网络复位或网络重新授权，一轮后重新准备仍须断动力复位。STOP会解除主动保持，机械支撑不能撤掉。

## 验收与待执行

主机测试编译实际M33包装器和状态机，模拟电机IO；新增动作覆盖完整往返、端点驻留、无授权、排队时更换动作、已用周期重放、反馈丢失、取消及保持偏移，均检查整组停止。另验证路径白名单与12项Python接口测试。模拟不等于实物验收。

M33/M55编译和M33组合镜像打包通过，既有未使用代码及RWX链接告警保留。证据：`original-ican-fixed-actions-m33-build.log`、`original-ican-fixed-actions-m55-build.log`、`original-ican-fixed-actions-package.log`，镜像哈希详见`original-ican-fixed-actions-prepared.json`。

更新前已保存原v5配套ELF、已校验镜像和绑定文件，路径`.restore-tools/ican-v5-known-good-20261016-220832/`，各文件复制后SHA256核对一致。实际写入及供电状态更正见下节。上电实物验收建议顺序：小幅→中幅→完整→停留方案，每项单独确认现场结果。

## 双核更新与供电状态更正

用户先回复“电机动力已关闭，USB和DAP保持连接”，据此核对配套镜像SHA256与KitProg3连接后，先写M55并校验2129920字节，再执行M33完整写入/启动流程：原始镜像校验933888字节、XIP映射校验925676字节通过。日志为`original-ican-fixed-actions-m55-flash.log`、`original-ican-fixed-actions-m33-flash.log`。Wi-Fi恢复，配置v6、ABI10双核绑定正常，流程state0、active=false、auth_ready=false。

严格断动力检查在4号ID查询获得有效回复时退出，记录`original-ican-fixed-actions-wifi-acceptance.log`，该项不得写成通过。用户随后明确更正“其实我现在并未断电”，实际供电状态与先前确认不一致。此后没有再次复位/写入，也没有发布动作授权、使能或运动目标。

不假定断动力的只读检查通过，日志`original-ican-fixed-actions-wifi-readonly-acceptance.log`：页面、双核运行计数、HTTP异常拒绝、四动作无授权请求拒绝、三轴12项参数/ID查询及只读故障诊断均正常。使能尝试、目标发送、保持准备phase、保护原因均为0，记录`original-ican-fixed-actions-postflash-state.json`。下一步仍需实际关闭动力并确认，再完成断动力验收；之后重新确认现场准备及上电才可启动新动作。

## 实际断动力验收完成

用户随后确认“现在已实际关闭电机动力，USB和DAP保持连接”。首次严格检查遇到一次HTTP超时，保存`original-ican-fixed-actions-actual-off-wifi-acceptance.log`；仅重试只读/未授权拒绝检查，没有重试运动。重试日志`original-ican-fixed-actions-actual-off-wifi-acceptance-retry.log`通过：三轴12项查询均无有效设备回复，四动作无授权拒绝、异常HTTP和诊断检查通过。

`original-ican-fixed-actions-actual-off-stop-acceptance.log`验证断动力STOP：请求接收后有界退出，active=false，run_result=-26，stop_send_result=-255，未将发送失败伪装成成功或物理停止。该STOP锁存随后在保持断动力的条件下经DAP复位清除，日志`original-ican-fixed-actions-actual-off-final-reset.log`。

最终Wi-Fi恢复、v6绑定正常、state0、flags1、active=false、auth_ready=false；保持准备phase0，使能/目标次数、保护原因和使能掩码均0。证据`original-ican-fixed-actions-final-idle.json`。没有发布新动作授权、没有使能或发送运动目标。下一步由现场确认未穿戴、支撑/行程正常并重新上电后，先验收约30°小幅屈伸一轮。

## 约30°小幅屈伸首次上电测试

用户明确确认已上电、支撑和行程正常，允许约30°小幅屈伸。通过统一`ArmDemo.start_action`接口调用`elbow-small`；固件匹配且单次机会尚未使用，原工具仅提交一次op12有效动作，request seq7。完成快照seq23：state13、active=false、run_result0、stop_send_result0，工具退出码0。

5号起点5.0233269rad，最后位置5.0209026rad，伸展目标5.0213637rad；enable_attempts1、target_attempts43，failure_state0、first_error0。4/6保持基线分别为1.1467652rad、1.3655839rad，phase12，无失败轴及保护触发。板端完成含两端到位及驻留；本次未保存连续反馈轨迹，不据此声明精密30°机械角度验收通过。

追加STOP后seq29确认active=false、auth_ready=false。清理产生的state14/run_result-26不覆盖原DONE记录，统一接口completed=true。证据`original-ican-elbow-small-20261016-222830-trial.log`、`original-ican-elbow-small-20261016-222830-report.json`。最后确认电机动力为上电；现场是否完整屈伸、有无侧偏/抖动或结束下落、物理静止及关闭动力均待反馈。没有自动启动下一项动作。


## 约30°小幅屈伸再次观察运行

用户确认实际断动力后，经DAP复位并核对空闲。随后用户要求启动。第一次工具调用在授权前的只读状态读取超时，未写授权、未提交动作，日志`original-ican-elbow-small-repeat-20261016-224026-trial.log`；后查仍state0、单次机会未用。确认此前无运动提交后重新准备，仅提交一次有效op12请求seq7。

本轮完成快照seq21为state13、run_result0、active=false、stop_send_result0；4/6保持phase12，无失败轴或trip，首次错误0，使能尝试1、目标发送43。5号起点5.0229816rad，最终5.0209026rad，伸展目标5.0213637rad。追加STOP后seq27确认流程不活动、无授权；清理取消码-26不覆盖DONE成功。统一接口completed=true。

证据：`original-ican-elbow-small-observe-20261016-224119-trial.log`和同名`-report.json`。现场观察和静止/断动力待用户反馈，未自动启动下一项。

用户随后反馈“正常了”，并提出改为LVGL启动/停止、完整屈肘动作连续循环。该反馈记录为上一轮现场观察正常；没有将其解释为动力已关闭。新需求的实施与验收另见`LVGL屈肘循环演示-v7.md`。
