# RobotX Task4 实船分阶段测试手册

> 适用范围：RobotX 2026 Task 4 Core tier 的 USV 子流程。现场执行人即使没有参与开发，
> 也应能按本手册完成安全检查、无控制联调和有限的实船验证。
>
> 本手册基于当前 `task4_usv_planner` 仓库、已部署到 Orin 的
> `usv_task4_executor` 固定版本和现有完成清单编写。任何现场发现若与本文不一致，
> 以“停止、只读确认、联系负责人”为准，不猜接口、不临时改源码。

## 0. 一页结论

当前已经完成并验证：

- Core 子状态机：`IDLE -> TASK4_ACK -> TRANSIT -> TASK4_READY -> RESUMED`；
- GPS 目标转本地 NED、直达 setpoint、5 m 默认到达半径、1000 m 默认最大距离；
- checkpoint、progress、14-bit 状态/错误编码；
- 官方 Core 通信链：`AssistanceRequest -> IncidentAck -> ReadinessReport -> ReadinessConfirm`；
- OCS 内部 JSON 信封适配和 `/tongji_mrl/twave/vehicle_report` 上报；
- VRX 中的 GPS/IMU 位姿适配和左右推进器闭环；
- 纯 Python 测试 18/18、ROS 侧测试 7/7、包构建以及 Orin 单包构建均曾通过。

尚未完成或不能据此宣称完成：

- **当前 Orin 的 `usv_task4_executor` 只发布 `/task4_core/setpoint`，没有已确认的实船
  控制器/goto 接口。** 仿真的 `task4_controller_node` 只面向 WAM-V 的
  `/wamv/thrusters/*`，严禁在实船上当作控制器使用；
- 尚未完成实船自主航行、真实推进器闭环和真实控制反馈验证；
- 尚未完成 OCS、地面端、移动端、总状态机和 Task4 executor 的四端端到端验证；
- 当前 launch 只固定 `vehicle_id=T-Wave` 和共享 NED topic，`team_id`、`run_id`、
  `checkpoint_task`、`checkpoint_progress` 仍需现场确认。默认 checkpoint 为
  `UNKNOWN/UNKNOWN`，不能作为正式恢复依据；
- 节点没有实现等待 GPS、等待 NED、航行和等待确认的自动超时；现场必须使用人工计时和
  停止规则；
- 状态机没有运行时 reset 命令。每轮独立测试应停止并重新启动 executor；
- Task4 Core 没有目标检测、LiDAR 感知、泵、夹爪或喷射执行机构。现场不要为 Task4
  重复采集已完成的数据，也不要启动 Task3 感知或控制链。

因此，明天的成功标准分三层：

| 优先级 | 必须完成的结果 | 允许的动作 |
| --- | --- | --- |
| P0 | 网络、版本、构建/测试、真实 GPS/NED、隔离 topic 下完整 Core 协议链、安全退出 | 船不上动力或保持硬件断能，不启动任何控制器 |
| P1 | 默认 topic 下总状态机/OCS/地面端/移动端双向 ACK、状态和结果闭环 | 仍不允许自主运动；目标使用当前位置 |
| P2 | 人工遥控最小位移验证定位方向；仅在真实控制接口全部确认后评审自主单步 | 最低速度、最小距离、一次一条命令；任一门槛不满足即跳过 |

时间不足时只做 P0，再做 P1；P2 可以不做。**没有真实控制接口不是现场人员应临时修复的
问题，而是明确阻塞。**

## 1. 已确认接口与边界

### 1.1 Orin 正式包

- ROS 包：`usv_task4_executor`
- launch：`task4_executor.launch.py`
- 可执行程序：`task4_executor`、`task4_core_node`
- 正式工作区：`/home/robotx/robotx_ws`
- 已部署固定版本：`robotx_task_execution` commit
  `f124daa8ae816e76b2d8977b4b5cd871a4868b85`
- 工作区管理分支已固定该版本：`feature/usv-orin-unified-workspace` commit `e262463`

### 1.2 Orin Task4 topic

| 方向 | Topic | 类型 | 当前代码用途 |
| --- | --- | --- | --- |
| 输入 | `/tongji_mrl/twave/task_cmd` | `std_msgs/msg/String` | OCS 路由命令或内部 `TASK4_BEGIN/resume` JSON |
| 输入 | `/fix` | `sensor_msgs/msg/NavSatFix` | 第一条有效 fix 建立 Task4 GPS 参考原点 |
| 输入 | `/tongji_mrl/twave/ned_position` | `std_msgs/msg/String` | 实船共享 NED 位置 |
| 兼容输入 | `/task4_planner_node/current_pose` | `geometry_msgs/msg/Pose2D` | 兼容位姿输入；不是 Orin launch 的独立适配节点 |
| 输出 | `/tongji_mrl/twave/checkpoint` | `std_msgs/msg/String` | 被中断任务 checkpoint |
| 输出 | `/tongji_mrl/twave/progress` | `std_msgs/msg/String` | ACK、READY、RESUMED 和结构化错误 |
| 输出 | `/tongji_mrl/twave/task4_report` | `std_msgs/msg/String` | 官方形状的 IncidentAck/ReadinessReport |
| 输出 | `/tongji_mrl/twave/vehicle_report` | `std_msgs/msg/String` | OCS vehicle-link 的 ACK/task_report 信封 |
| 输出 | `/task4_core/state` | `std_msgs/msg/String` | Core 状态、原因、状态码和恢复字段 |
| 输出 | `/task4_core/setpoint` | `geometry_msgs/msg/Pose2D` | `x=north`、`y=east`、`theta=heading` 的直达目标 |

`/tongji_mrl/twave/ned_position` 当前解析器要求 JSON 至少包含：

```json
{
  "position_ned_m": {
    "north": 0.0,
    "east": 0.0
  },
  "heading_rad": 0.0
}
```

`heading_rad` 可缺省，代码会沿用上一次 heading 或初始 `0.0`；但实船运动测试前必须确认
heading 有效。`down` 当前不参与二维到达判断。

### 1.3 坐标系

- Task4 Core 使用局部 NED：`x=north`、`y=east`；
- `theta/heading_rad` 从北向东为正，单位 rad；
- 目标命令是 WGS84 纬经度，Core 相对其启动后收到的第一条 `/fix` 转为 NED；
- 共享位置和 Task4 GPS 参考必须代表同一船体参考点，并且原点关系必须一致；
- 默认到达半径 5 m，默认最大直达距离 1000 m。

### 1.4 service、action、感知和执行机构

当前 Task4 包**没有声明 service 或 action**。Core 感知输入只有 GPS/NED 位姿；不存在
Task4 LiDAR、目标检测或物体感知节点。当前 Orin Task4 包也没有真实推进器、goto、泵、夹爪
或喷射执行器节点。现场若看到其他包提供这些接口，只能先只读确认归属、类型、单位、frame、
唯一发布者和安全责任人，不能按名字猜测后调用。

## 2. 人员和安全责任

至少三人，推荐四人：

| 角色 | 责任 | 不得兼任的情况 |
| --- | --- | --- |
| 测试指挥 | 唯一发令、记录阶段结论、决定继续/停止 | 进行 P2 时不得同时敲命令 |
| 遥控/急停手 | 全程握遥控器，确认模式、接管、零油门、解锁/上锁 | 不得离船或低头操作电脑 |
| 电脑操作员 | Windows、SSH、tmux、ROS 只读检查和测试命令 | 不得自行决定开始运动 |
| 观察员 | 看船、水域、人员、缆绳和障碍，复述停止口令 | 场地视线不完整时必须单独设置 |

上电前必须现场确认：

- 遥控接管开关和急停真实有效，遥控电量充足；
- 推进器危险区无人、船体固定可靠；
- 实船运动前解除固定绳的方式不会缠桨；
- 低层飞控 failsafe 已由平台负责人确认；
- 泵、夹爪、喷射等非 Task4 硬件默认断电或保险关闭；
- P0/P1 阶段推进器不解锁；
- 控制 topic 只有唯一发布者。无法证明唯一时禁止运动。

统一停止口令：**“停止 Task4，人工接管，断执行器！”**

任何人喊出口令后，按固定顺序执行：

```text
遥控手立即人工接管并回零
-> 确认船体停止/受控
-> 危险执行机构物理断电或上锁
-> 电脑操作员停止控制节点
-> 保留所有终端和日志
-> 不自动重试
-> 测试指挥确认原因后才允许恢复
```

软件 `Ctrl+C` 不能代替遥控接管、飞控上锁或物理断电。

## 3. 终端分类和布局

### 3.1 三类命令

- **常驻**：launch、节点、控制器、rosbag。运行后不能继续输入，下一步必须新开终端或
  tmux 窗口/分屏；关闭时按 `Ctrl+C`，等待进程退出。
- **监视**：持续 ping、`topic echo`、`topic hz`、`watch`。需要保持观察；结束时按
  `Ctrl+C`。下一步必须使用另一个终端/窗口/分屏。
- **一次性**：执行完成会返回提示符，可以复用当前终端。

### 3.2 Windows 窗口

| 编号 | 位置 | 类型 | 命令/用途 | 保持到何时 |
| --- | --- | --- | --- | --- |
| W0 | Windows PowerShell | 常驻 | `ssh robotx@192.168.206.66`，登录后进入 `tmux` | 全部 Orin 测试结束 |
| W1 | Windows CMD | 监视 | `ping -t 192.168.206.66` | 全部测试结束 |
| W2 | Windows CMD | 监视 | `ping -t 192.168.206.200` | P1 通信结束 |
| W3 | Windows CMD | 监视 | `ping -t 192.168.206.202` | P1 通信结束 |

W0 登录后执行以下**一次性**命令：

```bash
tmux new -s task4_field
```

若提示 session 已存在：

```bash
tmux attach -t task4_field
```

SSH 中断后重新打开 W0，重新 SSH，再执行 `tmux attach -t task4_field`。不要重启已经在
tmux 中运行的节点。

### 3.3 Orin tmux 窗口

只建立当前 Task4 确实需要的窗口：

| 编号/名称 | 类型 | 用途 |
| --- | --- | --- |
| T0-health | 监视 | 系统时间、节点和基本健康状态 |
| T1-position | 常驻/监视 | MAVROS 已由平台负责人启动时只读检查；启动共享位置并监视 `/fix`、NED |
| T2-task4 | 常驻 | 启动隔离 dry-run 或正式 Task4 executor |
| T3-task4-io | 监视/常驻 | state、progress、report、setpoint 监视和最小证据 rosbag |
| T4-comm | 监视 | P1 默认 topic、ACK、结果及应用层通信观察 |
| T5-query | 一次性 | 版本、topic、类型、发布者/订阅者、构建和命令注入 |

tmux 操作：`Ctrl+B` 后按 `c` 新建窗口，按 `,` 重命名，按 `%` 左右分屏，按 `"` 上下
分屏，按数字切换窗口。每个新窗口或分屏都必须先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/robotx/robotx_ws/install/setup.bash
```

禁止 source 旧 workspace、staging、独立 MAVROS 或 Livox 工作区。

### 3.4 阶段窗口总表

| 进入阶段 | 新启动窗口 | 必须继续保持的窗口 | 可以关闭/复用的窗口 |
| --- | --- | --- | --- |
| 网络检查 | W0、W1、W2、W3 | W1、W2、W3 | Windows 一次性网络检查窗口 |
| Orin 健康/版本 | T0、T5 | W1-W3、T0 | T5 可反复复用 |
| 定位检查 | T1 | W1-W3、T0、T1 | T5 可复用 |
| 隔离 dry-run | T2、T3 | W1-W3、T0、T1、T2、T3 | T5 用于一次性发布 |
| 默认 topic/四端 P1 | 重新启动 T2，启用 T4 | W1-W3、T0、T1、T2、T3、T4 | T5 可复用 |
| 人工最小位移 P2 | 不新增控制窗口 | W1、T0、T1、T3；遥控手在位 | W2/W3 可按通信需要保留 |
| 已确认实船控制 P2 | **仅接口负责人确认后**新建 T6-control | 所有安全监视窗口和遥控手 | 未确认接口时不得创建/启动 T6 |
| 关闭 | 无 | 按第 17 节逐一关闭 | 最后关闭 W1-W3、W0 |

## 4. 阶段 A：上电前物理检查

执行地点：船边；不需要终端。类型：人工检查。全部勾选前禁止给推进器上动力。

- [ ] 船体、推进器、天线、网线、电源线和防水接头无松动/进水；
- [ ] 桨叶周围无人员、工具、缆绳和漂浮物；
- [ ] 电池电压和固定方式正常；
- [ ] Orin、路由器、地面端、移动端地址标签一致；
- [ ] 遥控器已绑定，人工/自动模式清晰，接管和急停已做无桨或断能验证；
- [ ] 非 Task4 执行机构默认断电；
- [ ] 测试边界、最近障碍、岸边距离和回收路线已口头确认；
- [ ] 所有人知道统一停止口令；
- [ ] P0/P1 阶段船体固定，推进器不上锁/不解锁。

任一项失败：停止，不用软件绕过。

## 5. 阶段 B：网络和四端连通

### 5.1 Windows 一次性检查

位置：Windows。终端：新的 PowerShell/CMD。类型：一次性。执行完可关闭或复用。

```powershell
ipconfig
ping -n 30 192.168.206.66
ping -n 30 192.168.206.200
ping -n 30 192.168.206.202
Test-NetConnection 192.168.206.66 -Port 22
```

记录测试电脑的 IPv4 为 `TEST_PC_IP`。不要记录 VPN、虚拟网卡或无关接口地址。

随后启动并保持 W1、W2、W3 三个**监视**窗口：

```bat
ping -t 192.168.206.66
```

```bat
ping -t 192.168.206.200
```

```bat
ping -t 192.168.206.202
```

下一步必须使用 W0 或其他窗口，不能占用 W1-W3。

### 5.2 SSH 和反向 ping

位置：Windows W0。类型：常驻交互会话。

```powershell
ssh robotx@192.168.206.66
```

进入 tmux 的 T5 后，位置：Orin。类型：一次性：

```bash
ping -c 30 TEST_PC_IP
ping -c 30 192.168.206.200
ping -c 30 192.168.206.202
```

必须把 `TEST_PC_IP` 替换为 5.1 记录的实际地址。若地面端和移动端支持终端，也分别反向
ping `192.168.206.66` 和 `TEST_PC_IP`。

通过建议：0% 丢包最好；任何连续丢包、时延突增或超过 2% 丢包先视为不稳定。P0 可以在
确认原因后做纯离线测试，但 P1/P2 不得继续。

### 5.3 应用层双向验证原则

ping 只说明 IP 可达。Task4 应用层必须至少看到：

1. 地面/OCS 发出一条目标为 `T-Wave` 的 Task4 命令；
2. Orin 的 `/tongji_mrl/twave/task_cmd` 收到该 JSON；
3. Orin 发布 IncidentAck 或 vehicle ACK；
4. 地面端/移动端显示或记录 ACK；
5. READY 后地面/OCS 发回匹配 `report_seq` 的 ReadinessConfirm；
6. Orin 状态进入 `RESUMED`，结果在地面端/移动端可见。

当前 Task4 executor 本身不发布 heartbeat；若公共通信模块声称有 heartbeat，必须先用以下
**一次性只读**命令发现真实 topic 和类型，再由通信负责人确认：

```bash
ros2 topic list -t | grep -Ei 'heartbeat|task4|vehicle_report|ack|progress'
```

没有发现真实 heartbeat 时，不得编造 heartbeat 名称；使用 ACK、状态和结果消息完成应用层
双向验证。

## 6. 阶段 C：版本、环境和系统健康

### 6.1 T0 健康监视

位置：Orin T0-health。先 source 正式环境。类型：监视。保持至关闭阶段：

```bash
watch -n 2 'date -Is; echo ROS_NODES; ros2 node list | sort; echo DISK; df -h /home/robotx'
```

下一步必须切到 T5。

### 6.2 T5 一次性检查

位置：Orin T5-query。类型：一次性：

```bash
date -Is
timedatectl status
df -h /home/robotx
free -h
git -C /home/robotx/robotx_workspace status --short --branch
git -C /home/robotx/robotx_workspace log -1 --oneline
ros2 pkg prefix usv_task4_executor
ros2 pkg executables usv_task4_executor
colcon list --base-paths /home/robotx/robotx_ws/src | awk '{print $1}' | sort | uniq -d
```

期望：

- 工作区管理仓库位于 `feature/usv-orin-unified-workspace`，包含 `e262463`；
- `usv_task4_executor` prefix 位于 `/home/robotx/robotx_ws/install/`；
- 至少显示 `task4_executor` 和 `task4_core_node`；
- 重复包检查无输出；
- 系统时间与测试电脑/地面端一致，磁盘空间足够记录短时日志。

再做类型和接口发现，位置 T5，类型：一次性：

```bash
ros2 topic list -t | sort
ros2 service list -t | sort
ros2 action list -t | sort
ros2 interface show std_msgs/msg/String
ros2 interface show geometry_msgs/msg/Pose2D
ros2 interface show sensor_msgs/msg/NavSatFix
```

若发现同名包来自其他 prefix、重复包、时间相差超过 1 s 或磁盘接近满：停止 P1/P2，先由
负责人修复。不要临时 source 其他工作区覆盖问题。

## 7. 阶段 D：Task4 离线测试和单包构建

位置：Orin T5-query。类型：一次性。船保持断能，不需要启动 MAVROS。

先运行当前纯 Python 非控制测试：

```bash
cd /home/robotx/robotx_ws/src/task4
PYTHONPATH=. python3 -m unittest discover -s test -v
```

期望 18 项通过。然后执行单包构建和包测试：

```bash
cd /home/robotx/robotx_ws
colcon build --packages-select usv_task4_executor --symlink-install --executor sequential
source /home/robotx/robotx_ws/install/setup.bash
colcon test --packages-select usv_task4_executor --event-handlers console_direct+
colcon test-result --verbose
```

构建或测试失败时不要删除 `build/install/log`，记录第一处错误并停止后续阶段。现场禁止为通过
测试而修改业务源码。

## 8. 阶段 E：真实定位输入和坐标系检查

### 8.1 飞控和共享位置

MAVROS 的实际启动参数由平台负责人提供，本手册不猜测。若 MAVROS 已启动，在 T5 做以下
**一次性只读**检查：

```bash
ros2 topic echo --once /mavros/state
ros2 topic echo --once /mavros/global_position/global
ros2 topic echo --once /fix
ros2 topic info /fix --verbose
```

若 `/fix` 不存在但 `/mavros/global_position/global` 为有效
`sensor_msgs/msg/NavSatFix`，先记录为接口不一致；只能由负责人决定修改 launch 参数，现场人员
不能自行假设等价。

位置：Orin T1-position。类型：常驻：

```bash
ros2 launch usv_shared_position shared_position.launch.py
```

该窗口运行后不能再输入。若提示同名节点已存在，不要启动第二份；按 `Ctrl+C` 退出新启动的
重复实例，并在新分屏中只读检查现有节点。

在 T1 新分屏中先 source 环境，执行**监视**：

```bash
ros2 topic echo /tongji_mrl/twave/ned_position std_msgs/msg/String
```

在另一个 T1 分屏执行**监视**：

```bash
ros2 topic hz /tongji_mrl/twave/ned_position
```

保持 20 s，记录频率和静止漂移后按 `Ctrl+C` 结束 `hz`；NED echo 可以继续保持。

### 8.2 只读验收点

- `/fix` 类型必须是 `sensor_msgs/msg/NavSatFix`，纬度、经度和高度为有限数；
- `ned_position` 必须是 `std_msgs/msg/String`，JSON 中的
  `position_ned_m.north/east` 为有限数；
- `heading_rad` 若存在，应为有限 rad；船头方向由平台负责人现场核对；
- 静止 20 s 内 N/E 不应持续单向发散。场地小，漂移超过约 1.5 m 先暂停；超过 3 m 禁止
  P2；
- 时间戳、frame 或原点若消息中存在，必须稳定且与平台约定一致；
- Task4 GPS 参考是 executor 启动后的第一条 `/fix`。executor 启动时船必须静止。

Task4 Core 没有其他感知输入。不要启动 LiDAR、Task3 几何感知或 VRX
`task4_pose_adapter_node` 来“补齐”实船测试。

## 9. 阶段 F：无控制隔离 dry-run

目标：使用真实 `/fix` 和真实共享 NED，但把 Task4 命令、状态和 setpoint 全部放到
`/task4_field/*`，不接入任何控制器或 OCS。

### 9.1 安全门

位置：T5。类型：一次性：

```bash
ros2 topic info /task4_field/setpoint --verbose
ros2 topic info /task4_core/setpoint --verbose
```

`/task4_field/setpoint` 必须没有订阅者。船保持人工模式/上锁/断执行器。若发现任何未知控制
订阅者，停止并查明节点：

```bash
ros2 node list
ros2 node info NODE_NAME
```

### 9.2 启动隔离 executor

位置：Orin T2-task4。类型：常驻。运行后必须去 T3/T5：

```bash
ros2 run usv_task4_executor task4_executor --ros-args \
  -p task_cmd_topic:=/task4_field/task_cmd \
  -p checkpoint_topic:=/task4_field/checkpoint \
  -p progress_topic:=/task4_field/progress \
  -p setpoint_topic:=/task4_field/setpoint \
  -p state_topic:=/task4_field/state \
  -p report_topic:=/task4_field/task4_report \
  -p vehicle_report_topic:=/task4_field/vehicle_report \
  -p checkpoint_task:=TASK1 \
  -p checkpoint_progress:=FIELD_DRY_RUN \
  -p arrival_radius_m:=1000.0
```

`arrival_radius_m=1000.0` 仅用于本次无控制协议测试，使当前位置目标快速进入 READY。禁止把该
参数用于正式航行。

### 9.3 启动监视

在 T3-task4-io 建三个分屏，每个先 source 正式环境。

T3a，类型：监视，保持到 dry-run 结束：

```bash
ros2 topic echo /task4_field/state std_msgs/msg/String
```

T3b，类型：监视：

```bash
ros2 topic echo /task4_field/progress std_msgs/msg/String
```

T3c，类型：监视：

```bash
ros2 topic echo /task4_field/task4_report std_msgs/msg/String
```

可再开 T3d 做最小证据记录，类型：常驻；不要记录 LiDAR/图像等已完成数据：

```bash
ros2 bag record -o /home/robotx/robotx_logs/task4_field_p0 \
  /fix /tongji_mrl/twave/ned_position \
  /task4_field/checkpoint /task4_field/progress \
  /task4_field/state /task4_field/setpoint \
  /task4_field/task4_report /task4_field/vehicle_report
```

### 9.4 发布隔离 AssistanceRequest

先在 T5 执行**一次性**命令并抄下当前位置纬度、经度：

```bash
ros2 topic echo --once /fix sensor_msgs/msg/NavSatFix
```

将下一条命令中的 `LATITUDE`、`LONGITUDE` 替换为刚读到的纯数字。位置：T5。类型：一次性。
该命令只发往隔离 topic；安全门确认无订阅者后不会驱动实船：

```bash
ros2 topic pub --once /task4_field/task_cmd std_msgs/msg/String \
  "{data: '{\"team_id\":\"FIELD_TEST\",\"seq\":91001,\"vehicle_id\":\"T-Wave\",\"body\":{\"type\":\"assistance_request\",\"position\":{\"latitude\":LATITUDE,\"longitude\":LONGITUDE},\"vehicle_type\":\"TYPE_USV\"}}'}"
```

10 s 内应观察到：

1. `/task4_field/task4_report`：`incident_ack`；
2. `/task4_field/checkpoint`：`TASK1/FIELD_DRY_RUN`；
3. `/task4_field/progress`：`TASK4_ACK`，随后 `TASK4_READY`；
4. `/task4_field/state`：`TASK4_ACK -> TRANSIT -> TASK4_READY`；
5. `/task4_field/vehicle_report`：ACK 和 task_report；
6. `/task4_field/task4_report`：`readiness`，记录其中的 `seq`，通常首次运行为 2，
   但必须以实际输出为准。

如果错过 volatile 消息：不要盲目重发相同 seq；停止 T2，重新启动 executor 和全部监视，
再使用新的 command seq。

### 9.5 发布隔离 ReadinessConfirm

把 `REPORT_SEQ` 替换为上一步 readiness 的实际 `seq`。位置：T5。类型：一次性：

```bash
ros2 topic pub --once /task4_field/task_cmd std_msgs/msg/String \
  "{data: '{\"seq\":91002,\"body\":{\"type\":\"readiness_confirm\",\"report_seq\":REPORT_SEQ,\"vehicle_id\":\"T-Wave\"}}'}"
```

期望 `/task4_field/state` 进入 `RESUMED`，`resume_task=TASK1`，
`resume_progress=FIELD_DRY_RUN`。本轮通过后先按 `Ctrl+C` 停止 T2 executor，再停止 T3
监视和 rosbag。状态机没有 reset 命令，下一轮必须重启。

### 9.6 安全错误上报检查

重新启动 9.2 和 9.3 后，可在 T5 发送一条不会产生运动的非法命令，类型：一次性：

```bash
ros2 topic pub --once /task4_field/task_cmd std_msgs/msg/String \
  "{data: '{\"vehicle\":\"usv\",\"command\":\"unsupported_field_test\"}'}"
```

期望 `/task4_field/progress` 发布 `TASK4_ERROR`，包含 `status_code`、`status_bits`、
错误名称、`retryable`、`recovery_action`、`detail_code` 和 detail。只做一次，记录后重启
executor。不要对正式 topic 注入错误命令。

## 10. 阶段 G：状态机、超时和控制反馈只读检查

### 10.1 状态机验收

P0 必须确认以下对应关系：

| 输入/条件 | 期望状态/输出 |
| --- | --- |
| AssistanceRequest 有效 | 立即 IncidentAck；GPS/NED 就绪后 checkpoint、TASK4_ACK |
| ACK 后定时 tick | `TRANSIT`，持续发布直达 setpoint |
| 距目标不大于到达半径 | `TASK4_READY` 和 ReadinessReport |
| ReadinessConfirm 的 `report_seq` 匹配 | `RESUMED`，记录 checkpoint 的 task/progress |
| 重复/旧 command seq | `TASK4_ERROR`，拒绝重复命令 |
| ReadinessConfirm seq 不匹配 | `TASK4_ERROR`，保持不恢复 |
| 目标距离超过 1000 m | `TASK4_TARGET_UNREACHABLE`，不接受该目标 |

### 10.2 当前超时限制

代码中虽定义了 TIMEOUT 类错误，但 executor 当前没有对等待 GPS、NED、TRANSIT 或
ReadinessConfirm 启动自动计时器。现场人工门限：

- 命令发出 5 s 内无 IncidentAck：停止应用层测试；
- GPS/NED 已确认有效后 10 s 内无 checkpoint/TASK4_ACK：停止并查 topic/消息；
- 当前位置 dry-run 10 s 内无 READY：停止并查 NED 原点/JSON；
- P1 30 s 内无 ReadinessConfirm：保持人工安全状态，停止本轮，不自动重试；
- 任何运动阶段 1 s 内丢失位置/控制反馈：立即人工接管和断执行器。

这些是现场停止规则，不代表软件已实现对应超时。

### 10.3 控制链发现

位置：T5。类型：一次性，只读：

```bash
ros2 topic info /task4_core/setpoint --verbose
ros2 topic type /task4_core/setpoint
ros2 topic echo --once /task4_core/setpoint geometry_msgs/msg/Pose2D
```

若 subscription count 为 0：结论是“Task4 规划输出正常，但实船控制未接入”，禁止自主运动。

若存在订阅者，记录节点名后：

```bash
ros2 node info NODE_NAME
```

只有控制负责人确认以下全部信息，才允许建立 T6-control：

- 节点确为实船控制器/goto adapter，不是 VRX 节点；
- 输入是 NED `Pose2D`，单位和 heading 定义一致；
- 有明确的速度/推力限制、超时零输出、人工接管和停止反馈；
- 实际执行器 topic 只有一个控制发布者；
- 有只读状态/到达/故障反馈，并明确消息类型；
- launch 名称和参数来自当前实船代码或负责人书面确认。

在这些信息未确认前，本手册不提供任何真实控制 launch、service request、action goal、速度、
推力或舵角命令。

## 11. 阶段 H：最小低风险实船动作

本阶段只验证真实定位方向，不验证自主控制。必须完成 P0，且测试指挥、遥控手、观察员全部
在位。若场地太小、定位漂移大、网络不稳定或无法物理断电，跳过。

1. Task4 executor 和所有未知控制节点保持停止；
2. T1 持续观察 `/fix` 和 NED，T3 可只记录 `/fix`、NED；
3. 遥控手选择人工模式，确认随时可回零/上锁；
4. 测试指挥口头指定一个无遮挡方向，计划位移 **1-2 m**；
5. 观察员确认水域和缆绳安全后，遥控手用最低可控输入单步移动；
6. 达到 1-2 m 或持续 3 s（先到者为准）立即回零；
7. 检查 NED：向北移动主要增加 north，向东移动主要增加 east；数值方向错误或跳变立即停止；
8. 人工驶回或回收，不做连续往返，不做速度调参。

这是人工定位验证，不是 Task4 自主任务成功证据。不得用仿真
`task4_controller_node` 驱动实船。

## 12. 阶段 I：Task4 实际执行机构和自主控制

Task4 Core 当前没有任务专用执行机构；唯一可能的物理动作是 USV 航行。当前 Orin 部署未确认
从 `/task4_core/setpoint` 到实船控制器的闭环，因此本阶段默认状态是 **BLOCKED**。

若现场控制负责人完成第 10.3 节全部确认，可另建 T6-control，并先进行以下不运动验收：

1. 船体固定、推进器断能，启动已确认的实船控制节点；
2. 只读确认节点订阅 `/task4_core/setpoint`；
3. 只读确认控制状态、超时和零输出反馈；
4. 确认实际执行器命令只有一个 publisher；
5. 停止控制节点后确认硬件仍保持零输出；
6. 由平台负责人单独批准后，才可制定一条“最短距离、最低速度、单目标”的真实自主命令。

任何 service request、action goal、执行器 topic、单位、frame 或停止方式仍有未知时，本阶段结束
并记录阻塞，不临时试错。

## 13. 阶段 J：总状态机、地面端和移动端通信

先停止隔离 dry-run。保持 W1-W3、T0、T1。船仍不上动力。

### 13.1 默认 Task4 executor

位置：T2-task4。类型：常驻：

```bash
ros2 launch usv_task4_executor task4_executor.launch.py
```

期望日志显示 `vehicle_id=T-Wave`、NED topic 为
`/tongji_mrl/twave/ned_position`。下一步必须转到 T4/T5。

### 13.2 默认 topic 监视

T4-comm 建分屏并分别运行，全部为**监视**：

```bash
ros2 topic echo /tongji_mrl/twave/task_cmd std_msgs/msg/String
```

```bash
ros2 topic echo /tongji_mrl/twave/vehicle_report std_msgs/msg/String
```

```bash
ros2 topic echo /tongji_mrl/twave/task4_report std_msgs/msg/String
```

```bash
ros2 topic echo /tongji_mrl/twave/progress std_msgs/msg/String
```

T3 继续监视 `/task4_core/state` 和 `/task4_core/setpoint`。在 T5 做**一次性**发布者检查：

```bash
ros2 topic info /tongji_mrl/twave/task_cmd --verbose
ros2 topic info /tongji_mrl/twave/vehicle_report --verbose
ros2 topic info /task4_core/setpoint --verbose
```

### 13.3 四端端到端步骤

1. 地面端/OCS 使用其当前正式界面向 `T-Wave` 发送目标为**当前 GPS 位置**的
   AssistanceRequest；现场人员不得在默认 topic 手写模拟正式命令；
2. T4 确认 `/task_cmd` 收到，T4/T3 确认 IncidentAck、checkpoint、TASK4_ACK、READY；
3. 地面端和移动端确认显示同一 command seq、ACK 和 READY；
4. 地面端/OCS 用实际 readiness `report_seq` 发送 ReadinessConfirm；
5. Orin `/task4_core/state` 进入 `RESUMED`；
6. 地面端和移动端确认最终状态；
7. 保存最小 rosbag、OCS 日志时间点和三端截图，不重复采集 LiDAR、相机或其他任务数据。

若总状态机需要启动，必须使用团队文档已确认的：

```bash
ros2 launch usv_orin_state_machine usv_state_machine.launch.py
```

它是**常驻**命令，需新开独立 tmux 窗口。启动前先用 `ros2 node list` 确认没有现有实例。
当前 Task4 与总状态机的 topic 门控、checkpoint 来源和恢复责任尚未实船验证；若启动后出现重复
ACK、重复 publisher 或命令被吞，立即停止本轮并记录为集成阻塞，不改源码。

### 13.4 checkpoint 门槛

正式 P1 不能接受 `UNKNOWN/UNKNOWN` checkpoint。开始前必须由总状态机/任务负责人确认
`checkpoint_file` 或 `checkpoint_task/checkpoint_progress` 的真实来源。否则只可把本阶段记为
“通信可达、正式恢复阻塞”，不能判定 Task4 完成。

## 14. 可选分段任务测试

时间允许时按以下顺序，每轮都重启 executor，command seq 递增，且只测试一个变量：

| 分段 | 条件 | 目标 | 运动 |
| --- | --- | --- | --- |
| J1 | P0 通过 | 官方 AssistanceRequest -> IncidentAck | 无 |
| J2 | J1 通过 | checkpoint + TASK4_ACK | 无 |
| J3 | J2 通过 | 当前位置目标 -> READY/ReadinessReport | 无 |
| J4 | J3 通过 | 匹配 ReadinessConfirm -> RESUMED | 无 |
| J5 | J4 通过 | 错误 vehicle/report seq 被拒绝并上报 | 无，只在隔离 topic |
| J6 | 第 10.3 全部确认 | 极短自主目标及控制反馈 | 默认 BLOCKED，需单独批准 |

不要在一次运行中同时改变网络、坐标、控制参数和目标位置，否则失败后无法定位原因。

## 15. 故障处理表

未知异常统一执行：

```text
人工接管
-> 零输出
-> 危险执行机构断电
-> 保留日志
-> 不自动重试
-> 等待负责人判断
```

| 现象 | 常见原因/只读检查 | 立即停止条件 | 处理 |
| --- | --- | --- | --- |
| Windows 找不到 `ssh` | Windows OpenSSH Client 未安装；PowerShell 执行 `Get-Command ssh` | 无法建立受控会话 | 在“可选功能”安装 OpenSSH Client，或使用团队批准的 SSH 客户端；不要关闭现场防火墙代替安装 |
| 能 ping Orin，但 Orin 不能 ping Windows | Windows 防火墙/网络配置文件阻止 ICMP、地址选错 | P1/P2 禁止继续 | 核对 `ipconfig` 和网络类型；只为当前可信局域网放行 ICMP，修复后重测 30 包 |
| 连续丢包/时延突增 | Wi-Fi 干扰、距离、供电、路由器过载 | 任意连续丢包；运动中一次明显链路中断 | 人工接管、断执行器，靠近 AP、检查供电和信道；稳定后从网络阶段重做 |
| ping 正常但 SSH 端口失败 | sshd 未运行、防火墙、连接数过多 | 无可靠维护通道 | Orin 本地屏幕执行 `sudo systemctl status ssh`、`sudo ss -lntp | grep ':22'`、`sudo journalctl -u ssh -n 50 --no-pager`；由管理员恢复 sshd |
| `kex_exchange_identification`/SSH 被关闭 | sshd MaxStartups、临时封禁或服务异常 | 同上 | 停止反复重连；Orin 本地检查 ssh 日志，确认后 `sudo systemctl restart ssh` |
| SSH 中断 | 无线短断 | 若正在运动立即人工接管 | 重连 `ssh robotx@192.168.206.66`，再 `tmux attach -t task4_field`；先看现有进程，不重复 launch |
| ROS 包找不到 | 未 source 正式 install、构建失败 | 不得 source 旧工作区绕过 | 只 source 两条正式环境；查 `ros2 pkg prefix`，必要时重做单包构建 |
| 同名包/节点重复 | 旧源码仍在 src、重复 launch | 多控制发布者或重复任务节点立即停止 | `colcon list ... | uniq -d`、`ros2 node list`；由负责人清理，现场不删除未知目录 |
| launch/依赖/消息类型不一致 | Orin 版本未更新、topic 类型改变 | 任何类型与第 1.2 节不符 | 保存输出，检查固定 commit 和 package prefix；不做临时 topic 强转 |
| `/fix` 无输出/无效 | MAVROS/GNSS 未启动、天线、无 fix | 禁止发送 Task4 正式命令和运动 | 查 `/mavros/state`、GPS topic 类型和状态；由平台负责人恢复 |
| NED 无输出或 JSON 被忽略 | shared_position 未启动；`position_ned_m` 不是 north/east 对象 | 禁止 P1/P2 | 看 executor warning 和原始 String；修正上游接口后重启 executor |
| heading 错、TF/frame/单位不明 | ENU/NED 混用、deg/rad 混用、原点不一致 | 方向与人工观察不一致 | 人工接管；记录 N/E/heading，不进行控制；由定位负责人确认 |
| 时间戳异常 | 系统时钟未同步、消息陈旧 | 数据不再更新或倒退 | 停止任务，检查 `timedatectl` 和频率；不要用旧数据继续 |
| “Task4 感知无输出” | Core 本来没有 LiDAR/目标感知节点 | 无 | 不启动 Task3 感知；只检查 GPS/NED 输入 |
| 状态停在 IDLE/无 ACK | 命令未到、vehicle ID/格式错误 | 10 s 无预期输出 | 查 `/task_cmd`、`vehicle_id=T-Wave`、JSON；隔离测试可重启后用新 seq |
| 有 IncidentAck 但无 TASK4_ACK | GPS 或 NED 未就绪，checkpoint 无效 | 10 s | 查 `/fix`、NED 和 executor 日志；不自动重发 |
| 卡在 TRANSIT/无 READY | NED 原点不一致、目标超出到达半径、位置不更新 | dry-run 10 s；运动时 1 s 无反馈 | 人工接管；比对 setpoint 和 NED；不通过放大正式到达半径掩盖问题 |
| 重复命令被拒绝 | command seq 旧或相同 | 无需运动 | 重启独立测试或使用递增 seq；不得修改代码关闭去重 |
| ReadinessConfirm 被拒绝 | `report_seq` 或 vehicle ID 不匹配 | 保持安全等待状态 | 从实际 readiness report 抄 seq；重启完整一轮，不猜 seq |
| 目标超过 1000 m | 坐标输入错误、GPS/NED 原点差、真实目标过远 | 立即取消该目标 | 核对纬经度和 frame；不临时扩大正式上限 |
| 控制器无响应 | 当前 Orin 本就未确认实船控制器，或订阅/模式错误 | 禁止继续加大命令 | 查 setpoint subscriber 和节点 info；记录为控制接入阻塞 |
| 船反向运动/左右颠倒 | 坐标、heading、推进器映射错误 | 任何反向趋势 | 立即人工接管、零输出、断执行器；不得现场反复试方向 |
| 振荡/持续转圈 | heading 符号、增益、定位噪声 | 第一次明显振荡 | 同上；保留反馈，由控制负责人离线分析 |
| 多个控制发布者 | 重复节点或手动/自动同时输出 | 发现即禁止解锁 | `topic info --verbose` 定位；停止多余节点，必须恢复唯一发布者 |
| 执行机构不能停止 | 控制器/飞控/继电器故障 | 最高级停止 | 遥控接管、飞控上锁、物理断电；人员远离桨叶，禁止软件重试 |
| heartbeat/ACK/结果不上报 | 公共通信未启动、topic/路由/vehicle ID/run ID 错 | P1 失败，P2 禁止 | ping 后检查实际 topic、publisher/subscriber、OCS 日志；没有 heartbeat 时以 ACK/结果闭环为准 |
| 节点崩溃 | 输入异常、依赖或版本问题 | 运动中崩溃立即人工接管 | 保留 traceback/rosbag，停止控制链；只允许在断能状态重启一次验证 |

## 16. 结果记录表

### 16.1 基本信息

| 字段 | 记录 |
| --- | --- |
| 日期/时间 | |
| 场地/天气/水面 | |
| 测试指挥 | |
| 遥控手 | |
| 电脑操作员 | |
| 观察员 | |
| 测试电脑 IPv4 | |
| Orin 工作区 commit | |
| `robotx_task_execution` 固定 commit | |
| ROS_DOMAIN_ID | |
| rosbag/日志路径 | |

### 16.2 分阶段结果

| 阶段 | PASS/FAIL/BLOCKED/SKIP | 关键证据 | 异常与处理 | 负责人签字 |
| --- | --- | --- | --- | --- |
| A 物理安全 | | | | |
| B 网络/反向 ping/SSH | | | | |
| C 版本/环境/时间/磁盘 | | | | |
| D 18 项测试/单包构建 | | | | |
| E `/fix` 和共享 NED | | | | |
| F 隔离 Core 全链 | | | | |
| G 状态机/错误上报 | | | | |
| H 人工最小位移 | | | | |
| I 实船控制闭环 | | | | |
| J OCS/总状态机/地面/移动端 | | | | |

### 16.3 消息序列记录

| command seq | IncidentAck seq | ReadinessReport seq | Confirm report seq | 最终状态 | 备注 |
| --- | --- | --- | --- | --- | --- |
| | | | | | |

### 16.4 最终结论

只能选择一个：

- [ ] P0 通过：Core 在 Orin + 真实定位下可运行，无控制闭环结论；
- [ ] P1 通过：四端协议和状态闭环通过，实船控制仍阻塞；
- [ ] P2 通过：经平台负责人批准的最小实船闭环通过；
- [ ] BLOCKED：存在安全/接口/网络/定位/控制阻塞，禁止继续；
- [ ] FAIL：已知功能不符合预期，已保留日志且未自动重试。

禁止把“topic 有消息”“仿真通过”或“人工遥控移动”写成“Task4 实船自主航行通过”。

## 17. 按终端编号关闭

关闭前由测试指挥下达统一停止口令。按以下顺序：

1. 遥控手人工接管、回零、上锁；观察员确认船体受控；
2. 若存在 T6-control，先确认零输出/物理断执行器，再在 T6 按 `Ctrl+C`；
3. T2-task4 对 executor 按 `Ctrl+C`，等待退出；
4. 若启动总状态机/通信常驻节点，逐个 `Ctrl+C`；
5. T3 停止 rosbag，等待显示写盘完成，再停止 state/progress/report 监视；
6. T4 停止通信监视；
7. 船已回收并断推进器电源后，T1 才能停止 shared_position/MAVROS 相关进程；
8. T0 停止 health 监视；
9. 在 T5 执行一次性确认：

   ```bash
   ros2 node list
   ros2 topic info /task4_core/setpoint --verbose
   ```

10. 确认没有需保留的常驻任务后退出 tmux；仅在所有进程已人工确认停止后执行：

    ```bash
    tmux kill-session -t task4_field
    ```

11. 退出 W0 SSH；最后在 W1、W2、W3 按 `Ctrl+C`；
12. 物理断电并检查推进器停止，复制日志，填写第 16 节结论。

仅关闭 SSH 窗口不会停止 tmux 内的节点；`tmux detach` 也不会停止节点。

## 18. 明天最先执行的三个阶段

1. **阶段 A-B：**物理安全、Windows 三路持续 ping、SSH/tmux 和双向网络；
2. **阶段 C-E：**版本/包/构建测试，以及真实 `/fix`、NED、heading/原点检查；
3. **阶段 F：**隔离 topic、无控制的完整 Core 协议链和安全错误上报。

只有这三段全部通过，才进入 P1 的默认 topic/四端通信。没有实船控制接口确认时，到 P1
结束即为正确结果。
