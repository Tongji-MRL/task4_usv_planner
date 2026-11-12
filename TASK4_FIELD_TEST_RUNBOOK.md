# RobotX Task4 实船快速测试手册

适用环境：Ubuntu 测试电脑使用 VS Code Remote SSH 连接 USV Orin。

- Orin：`192.168.206.66`
- 地面端：`192.168.206.200`
- 移动端：`192.168.206.202`
- Orin 正式工作区：`/home/robotx/robotx_ws`

目标是在约 30-45 分钟内确认 Task4 当前真正具备的功能，不重复采集 LiDAR、相机或其他任务
数据，不临时修改源码，不尝试未经确认的实船控制命令。

## 1. 当前能测什么

当前 Orin 的 `usv_task4_executor` 已实现：

- `AssistanceRequest -> IncidentAck -> ReadinessReport -> ReadinessConfirm`；
- `IDLE -> TASK4_ACK -> TRANSIT -> TASK4_READY -> FINISHED`；
- GPS 目标转本地 NED；
- checkpoint 和简化 progress 上报；
- `/task4_core/setpoint` 目标输出；
- 可选的 LaserScan 反应式避障决策 `/task4_core/avoidance`；
- OCS 内部 JSON 命令和 `/tongji_mrl/twave/vehicle_report` 上报。

当前仍未确认：

- `/task4_core/setpoint` 是否接入真实实船控制器/goto；
- 真实控制反馈、唯一控制发布者和自主停止能力；
- 正式 checkpoint 来源以及 `team_id`、`run_id`；
- OCS、总状态机、地面端和移动端的完整端到端链路。
- 实船 LaserScan 的 frame/角度/单位，以及避障决策是否接入唯一真实控制边界。

仓库里的 `task4_controller_node` 只用于 VRX WAM-V 仿真，发布
`/wamv/thrusters/*`。**严禁在实船上启动它。** 仓库虽然已有 LaserScan 反应式避障
监视器，但在实船 frame 和真实控制接入确认前只能只读观察，不得据此放开自主运动。
Task4 没有泵、夹爪或喷射执行机构。

## 2. 最高效测试顺序

| 优先级 | 阶段 | 预计时间 | 成功标准 |
| --- | --- | --- | --- |
| P0 | 网络、版本、非控制测试 | 5-10 分钟 | SSH 正常、正式包正确、全部测试通过 |
| P0 | 真实 `/fix` 和 NED | 5 分钟 | 数据有效、格式正确、静止不明显漂移 |
| P0 | 隔离 topic 的完整 Core 流程 | 10 分钟 | ACK、READY、Confirm、FINISHED 全部出现 |
| P1 | 默认 topic + OCS/地面/移动端 | 10-15 分钟 | 双向命令和结果可见，checkpoint 非 UNKNOWN |
| P2 | 控制接入判定 | 2 分钟 | 明确“已接入可测”或“未接入，停止” |
| 可选 | 人工遥控 1-2 m 定位方向检查 | 3 分钟 | north/east 方向正确 |

时间不足时做到前三项即可。控制接口没有确认时，不做自主运动。

## 3. 安全门

开始前必须确认：

- 遥控手全程在船边，遥控器可立即切回人工并上锁；
- P0/P1 阶段推进器不上锁或保持物理断能；
- 桨叶附近无人、无线缆和漂浮物；
- 非 Task4 执行机构保持断电；
- 未确认唯一控制发布者前禁止运动；
- 网络中断、定位跳变、多个控制发布者或船不能停止时立即终止。

统一停止动作：

```text
人工接管 -> 回零/上锁 -> 危险执行机构断电 -> 停止软件节点 -> 保留终端错误 -> 不自动重试
```

`Ctrl+C` 不能代替遥控接管和物理断电。

## 4. Ubuntu 和 VS Code 连接

### 4.1 Ubuntu 本机网络检查

位置：Ubuntu 本地 Terminal。以下都是一次性命令：

```bash
ip -br address
ip route
ping -c 5 192.168.206.66
ping -c 5 192.168.206.200
ping -c 5 192.168.206.202
nc -vz -w 3 192.168.206.66 22
```

如果没有 `nc`，直接执行：

```bash
ssh -o ConnectTimeout=5 robotx@192.168.206.66
```

网络不稳定或连续丢包时，停止 P1/P2，只做离线测试。

### 4.2 VS Code Remote SSH

1. VS Code 安装 `Remote - SSH` 扩展；
2. `Ctrl+Shift+P`；
3. 选择 `Remote-SSH: Connect to Host...`；
4. 输入 `robotx@192.168.206.66`；
5. 连接后打开 `/home/robotx/robotx_ws`。

建议在 Ubuntu 的 `~/.ssh/config` 中加入：

```sshconfig
Host robotx-orin
    HostName 192.168.206.66
    User robotx
    ServerAliveInterval 15
    ServerAliveCountMax 3
```

以后可直接在 VS Code 连接 `robotx-orin`。

SSH 中断后重新连接。重新 launch 前先执行 `ros2 node list`，避免启动第二份相同节点。

## 5. VS Code 终端布局

在 VS Code 中打开 4 个远程终端：

| 终端 | 用途 | 类型 |
| --- | --- | --- |
| R0 | 查询、测试和一次性命令 | 可复用 |
| R1 | 共享位置 | 常驻 |
| R2 | Task4 executor | 常驻 |
| R3 | state/progress/report 监视，可拆成多个分屏 | 监视 |

每个新终端都只能执行下面两行环境加载：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/robotx_ws/install/setup.bash
```

禁止 source 旧 workspace、staging、独立 MAVROS 或 Livox 工作区。

## 6. P0-1：版本、包和离线测试

位置：R0。以下都是一次性命令：

```bash
git -C /home/robotx/robotx_workspace log -1 --oneline
ros2 pkg prefix usv_task4_executor
ros2 pkg executables usv_task4_executor
colcon list --base-paths /home/robotx/robotx_ws/src | awk '{print $1}' | sort | uniq -d
```

期望：

- 管理仓库包含 `e262463` 或后续明确更新；
- `usv_task4_executor` 位于 `/home/robotx/robotx_ws/install/`；
- 存在 `task4_executor` 和 `task4_core_node`；
- 重复包检查无输出。

运行当前非控制测试：

```bash
cd /home/robotx/robotx_ws/src/task4
PYTHONPATH=. python3 -m unittest discover -s test -v
```

期望全部测试为 `OK`；测试数量以当前仓库为准，不使用旧的固定数量判断版本。

只有源码刚更新、包找不到或安装版本不一致时才重建，平时不要浪费现场时间：

```bash
cd /home/robotx/robotx_ws
colcon build --packages-select usv_task4_executor --symlink-install --executor sequential
source /home/robotx/robotx_ws/install/setup.bash
```

构建失败不要删除工作区，保留第一处错误并停止后续测试。

## 7. P0-2：真实定位输入

### 7.1 先检查已有数据

位置：R0，一次性：

```bash
timeout 5s ros2 topic echo --once /mavros/state
timeout 5s ros2 topic echo --once /fix
timeout 5s ros2 topic echo --once /tongji_mrl/twave/ned_position
ros2 topic info /fix --verbose
ros2 topic info /tongji_mrl/twave/ned_position --verbose
```

如果 NED 已正常输出，不要重复启动共享位置，R1 保持空闲即可。

如果 NED 没有输出，位置：R1，常驻命令：

```bash
ros2 launch usv_shared_position shared_position.launch.py
```

启动后切回 R0 重做上面的 NED 检查。

### 7.2 必须确认

`/fix` 必须是 `sensor_msgs/msg/NavSatFix`，经纬度为有限数。

`/tongji_mrl/twave/ned_position` 必须是 `std_msgs/msg/String`，内容至少类似：

```json
{
  "position_ned_m": {
    "north": 0.0,
    "east": 0.0
  },
  "heading_rad": 0.0
}
```

- north/east/heading 必须是有限数；
- heading 单位是 rad；
- 船静止约 10 秒时，N/E 不应明显跳变或持续漂移；
- 漂移超过约 3 m、方向不明或 JSON 格式不符时，停止控制测试。

不要启动 Task3 节点。Task4 自有避障节点只有在确认 LaserScan topic、frame、0 rad 前向、
左右角度方向和米制距离后才能单独启动；P0 Core 流程不依赖它，也不重复采集点云。

## 8. P0-3：隔离 topic 的无控制完整流程

本阶段使用真实 GPS/NED，但所有 Task4 输出都放到 `/task4_test/*`，不会接入正式 OCS。

### 8.1 启动 executor

位置：R2，常驻命令：

```bash
ros2 run usv_task4_executor task4_executor --ros-args \
  -p task_cmd_topic:=/task4_test/task_cmd \
  -p checkpoint_topic:=/task4_test/checkpoint \
  -p progress_topic:=/task4_test/progress \
  -p setpoint_topic:=/task4_test/setpoint \
  -p state_topic:=/task4_test/state \
  -p report_topic:=/task4_test/task4_report \
  -p vehicle_report_topic:=/task4_test/vehicle_report \
  -p checkpoint_task:=TASK1 \
  -p checkpoint_progress:=FIELD_TEST \
  -p arrival_radius_m:=1000.0
```

`arrival_radius_m=1000.0` 只用于无控制协议测试，禁止用于正式航行。

### 8.2 确认没有控制订阅者

位置：R0，一次性：

```bash
ros2 topic info /task4_test/setpoint --verbose
```

subscription count 必须为 0。否则立即停止 R2，并查明未知订阅节点。

### 8.3 监视输出

把 R3 拆成四个分屏，每个先 source 正式环境，然后分别运行：

```bash
ros2 topic echo /task4_test/state
```

```bash
ros2 topic echo /task4_test/progress
```

```bash
ros2 topic echo /task4_test/task4_report
```

```bash
ros2 topic echo /task4_test/vehicle_report
```

这些都是监视命令，保持运行，结束时按 `Ctrl+C`。

### 8.4 发送当前位置目标

位置：R0。先读取当前位置：

```bash
ros2 topic echo --once /fix
```

把下一条命令中的 `LATITUDE` 和 `LONGITUDE` 替换为刚看到的数字：

```bash
ros2 topic pub --once /task4_test/task_cmd std_msgs/msg/String \
  "{data: '{\"team_id\":\"FIELD_TEST\",\"seq\":91001,\"vehicle_id\":\"T-Wave\",\"body\":{\"type\":\"assistance_request\",\"position\":{\"latitude\":LATITUDE,\"longitude\":LONGITUDE},\"vehicle_type\":\"TYPE_USV\"}}'}"
```

10 秒内应看到：

1. `incident_ack`；
2. checkpoint 为 `TASK1/FIELD_TEST`；
3. progress 依次出现 `TASK4_ACK`、`TASK4_TRANSIT` 和最终的 `TASK4_FINISH`；
4. state 经过 `TASK4_ACK -> TRANSIT -> TASK4_READY`；
5. task4_report 出现 `readiness`。

记住 readiness 消息中的实际 `seq`。首次通常为 2，但不要直接猜。

### 8.5 发送 ReadinessConfirm

把 `REPORT_SEQ` 换成上一步的实际值：

```bash
ros2 topic pub --once /task4_test/task_cmd std_msgs/msg/String \
  "{data: '{\"seq\":91002,\"body\":{\"type\":\"readiness_confirm\",\"report_seq\":REPORT_SEQ,\"vehicle_id\":\"T-Wave\"}}'}"
```

期望 state 进入 `FINISHED`，恢复字段为 `TASK1/FIELD_TEST`。

这一项通过后，P0 完成。按 `Ctrl+C` 停止 R2 和 R3。状态机没有 reset 命令，每轮新测试都
应重启 executor，并使用新的 command seq。

## 9. P1：默认 topic 和 OCS 联调

只有 P0 全部通过才进行。船仍保持不上动力。

### 9.1 启动正式 executor

位置：R2，常驻：

```bash
ros2 launch usv_task4_executor task4_executor.launch.py
```

位置：R0，先做安全检查：

```bash
ros2 topic info /task4_core/setpoint --verbose
```

若出现未知订阅者，先停止，不发正式命令。

### 9.2 监视正式 topic

在 R3 分屏中监视：

```bash
ros2 topic echo /tongji_mrl/twave/task_cmd
```

```bash
ros2 topic echo /tongji_mrl/twave/vehicle_report
```

```bash
ros2 topic echo /tongji_mrl/twave/progress
```

```bash
ros2 topic echo /task4_core/state
```

### 9.3 端到端步骤

1. 地面端/OCS 向 `T-Wave` 发送目标为**当前 GPS 位置**的 AssistanceRequest；
2. Orin `/task_cmd` 必须收到命令；
3. Orin 必须发布 ACK、checkpoint、READY 和 readiness report；
4. 地面端和移动端必须能看到 ACK/READY；
5. 地面端/OCS 使用实际 `report_seq` 发送 ReadinessConfirm；
6. Orin state 必须进入 `FINISHED`；
7. 地面端和移动端必须看到最终状态。

现场人员不要在默认 `/task_cmd` 上手写模拟正式命令。若没有 OCS，就把本阶段标记为“未测”，
不要用 P0 结果冒充端到端结果。

正式 checkpoint 不能是 `UNKNOWN/UNKNOWN`。如果是 UNKNOWN，只能说明通信链路可达，不能说明
恢复流程完成。

若团队要求总状态机参与，先确认没有现有实例，再在新的 VS Code 终端启动：

```bash
ros2 launch usv_orin_state_machine usv_state_machine.launch.py
```

出现重复 ACK、重复 publisher 或命令被吞时，立即停止本轮，不现场改源码。

## 10. P2：控制和最小实船动作

先在 R0 查看：

```bash
ros2 topic info /task4_core/setpoint --verbose
```

- subscription count 为 0：实船控制尚未接入，测试到此结束；
- 有订阅者：记录节点名，再执行 `ros2 node info NODE_NAME`；
- 只有平台负责人确认它是真实控制器、坐标和单位一致、具备超时零输出、反馈和人工接管后，
  才能另行批准自主动作。

本手册不提供未经确认的真实控制 launch、速度、推力、service request 或 action goal。

如果只需要快速确认定位方向，可选择人工遥控测试：

1. 停止 Task4 executor 和所有自动控制节点；
2. 遥控手切人工模式；
3. 最低速度移动 1-2 m，最长 3 秒；
4. 向北移动应主要增加 north，向东移动应主要增加 east；
5. 立即回零并上锁。

方向错误、数值跳变或网络中断时，立即人工接管并断执行器。这只是定位检查，不是 Task4
自主航行成功证据。

## 11. 快速故障处理

| 问题 | 处理 |
| --- | --- |
| VS Code 连不上 Orin | 本机先 `ping` 和 `nc`；普通 SSH 也失败时在 Orin 本地检查 `sudo systemctl status ssh` |
| SSH/VS Code 中断 | 人工接管；重连后先 `ros2 node list`，不要重复 launch |
| 找不到 ROS 包 | 只 source 正式环境，检查 `ros2 pkg prefix usv_task4_executor` |
| 有重复包/节点 | 停止重复实例；不要删除未知目录，不要 source 旧工作区 |
| `/fix` 无输出 | 查 MAVROS/GNSS；禁止继续 Task4 和运动 |
| NED 无输出 | 启动或检查 `usv_shared_position`；确认 JSON 是 north/east 对象 |
| 有 IncidentAck 但没有 `TASK4_ACK` progress | 通常是 `/fix` 或 NED 尚未就绪，检查 executor 日志 |
| 卡在 TRANSIT | 检查当前 NED、目标和原点；不要靠扩大正式到达半径掩盖问题 |
| Confirm 被拒绝 | 使用实际 readiness `seq`，不要猜 report seq |
| 重复命令被拒绝 | 重启 executor 或使用递增 command seq |
| 多个 setpoint/控制发布者 | 禁止解锁，先确定唯一控制责任节点 |
| 船反向、振荡或不能停止 | 人工接管、回零、上锁、物理断电，不自动重试 |
| ACK/结果不上地面端 | ping 只是网络可达；继续检查 `/task_cmd`、`vehicle_report` 和 OCS 路由 |

未知异常统一执行：

```text
人工接管 -> 零输出 -> 危险执行机构断电 -> 保留错误终端 -> 不自动重试 -> 等负责人判断
```

## 12. 关闭顺序

1. 遥控手人工接管、回零、上锁；
2. 若启动过真实控制器，确认零输出后先停止控制器；
3. R2 按 `Ctrl+C` 停止 Task4 executor；
4. 停止总状态机和通信测试节点；
5. R3 各分屏按 `Ctrl+C`；
6. 只有在船已安全回收后，才停止 R1 的共享位置；
7. 执行 `ros2 node list`，确认没有遗留测试节点；
8. 关闭 VS Code Remote SSH，最后给危险硬件断电。

## 13. 最终只需要回答五个问题

- [ ] Orin 网络、SSH、正式包和当前全部非控制测试是否通过？
- [ ] `/fix` 和共享 NED 是否真实、稳定、格式正确？
- [ ] 隔离 Core 是否完成 ACK -> READY -> Confirm -> FINISHED？
- [ ] OCS、地面端、移动端是否完成双向闭环，checkpoint 是否有效？
- [ ] `/task4_core/setpoint` 是否存在已确认的唯一实船控制订阅者？

前四项通过即可确认 Task4 Core 与通信流程；第五项未通过时，结论必须写“规划输出完成，实船
控制未接入”，不能写“实船自主航行完成”。
