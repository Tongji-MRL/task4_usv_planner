# Task 4 USV VRX/Gazebo 闭环仿真完成清单

目标：只完成 RobotX 2026 Task 4 Core tier 的 USV 子流程；先验证 checkpoint、ACK、航行、ready 和 resume，不扩展到 Advanced/Disruptive 避碰规划。

规则依据：2026 Team Handbook，Task 4 Dynamic Incident Response，Core Tier。

实船测试入口：[TASK4_FIELD_TEST_RUNBOOK.md](TASK4_FIELD_TEST_RUNBOOK.md)。该手册以当前
Orin `usv_task4_executor` 接口为准，VRX 推进器控制器不得直接用于实船。

## 实船待验证

- [ ] 现场网络、SSH/tmux、版本和正式包来源通过手册 P0 检查
- [ ] 实船 `/fix` 与 `/tongji_mrl/twave/ned_position` 的格式、原点、方向和稳定性确认
- [ ] Orin 上使用隔离 topic 完成无控制 Core 全链和错误上报验证
- [ ] OCS、总状态机、地面端、移动端的 ACK/READY/Confirm/FINISHED 闭环验证
- [ ] 正式 checkpoint 来源、`team_id`、`run_id` 和 vehicle ID 配置确认
- [ ] `/task4_core/setpoint` 到实船控制器/goto 的接口、唯一控制发布者和反馈闭环确认
- [ ] 实船 LaserScan 的 frame、0 rad 前向、左右方向、距离单位和更新率确认
- [ ] `/task4_core/avoidance` 在真实控制边界的 HOLD/REPLAN/超时停车接入确认
- [ ] 经遥控接管和物理急停门控后完成最小实船自主运动验证

最后两项当前仍受“实船控制接口未确认”阻塞；不得用下面已经通过的 VRX 控制结果代替。

## 当前里程碑

- [x] 修复 `ros_gz_sim create` 卡在 `/gazebo/worlds` 的问题
  - 原因：Gazebo Transport 默认广播无线网卡地址，本机回调端口不可达。
  - 修复：`run_vrx.sh` 默认设置 `GZ_IP=127.0.0.1`。
- [x] WAM-V 成功 spawn
  - `ros_gz_sim create` 返回 `OK creation of entity`。
- [x] 确认 WAM-V live ROS 接口
  - `/wamv/pose`：TFMessage 实时数据
  - `/wamv/joint_states`：实时数据
  - `/wamv/sensors/gps/gps/fix`：实时数据
  - `/wamv/sensors/imu/imu/data`：实时数据
  - 左右 thrust / pos bridge 已建立。
- [x] 完成 ENU → NED 位姿适配，并接入 Task4 Core node
  - 新增 `task4_pose_adapter_node`：订阅 `/wamv/pose`，发布
    `/task4_planner_node/current_pose`。
  - VRX 当前 `/wamv/pose` 仅有传感器相对变换，适配器自动使用 GPS + IMU
    备用路径；默认以首个 GPS fix 为 local NED 原点。
  - Core node 订阅 `/task4_planner_node/current_pose` 作为实时位置输入。
  - 位姿适配器测试通过；Core 子状态机、progress 接口与通信测试全部通过。
- [x] 清理 Core 不需要的旧复杂动态避碰演示和安全距离监视器
  - 已移除二维回放脚本、视频、摘要和独立动态安全监视节点。
  - 已移除上一阶段动态 planner、旧 supervisor、moving-object 接口和相关测试。
  - 旧动态目标预测和复杂绕行链路不再参与 Core；近场反应式安全层独立实现。
- [x] 定义 Core 子状态机：IDLE → TASK4_ACK → TRANSIT → TASK4_READY → FINISHED
  - 按 `TASK4.pdf` 实现 checkpoint、ACK、直达目标点、ready 和 resume 记录。
  - Task4 不保存、不恢复原任务航路；主规划器负责原任务生命周期。
  - 纯 Python 子状态机测试已覆盖主状态转移；ROS 测试已覆盖 Task3 风格的简单 progress JSON 契约。
- [x] 接入 Core 任务目标 GPS 点，并转换为本地 NED 目标
  - `GpsReference` 使用 WAM-V 首个 GPS fix 建立局部原点。
  - `TASK4_BEGIN.position.latitude/longitude` 转为 NED 目标。
- [x] 生成“直达目标点”的简单 setpoint
- [x] 到达目标点后发布最终 progress `FINISHED`
- [x] 接收主规划器的 resume 指令并交回控制权
  - `task4_core_node` 已接入 `/tongji_mrl/twave/task_cmd`、`checkpoint`、`progress`。
- [x] 添加最低限度的停止/越界保护
  - 目标超过最大直达距离时拒绝 ACK；默认上限 1000 m，可配置。
- [x] 简化 Task 4 progress 上报
  - 已删除独立状态码、错误码目录和编码模块。
  - 参考 Task3 子状态机边界，progress 使用
    `TASK4_ACK → TASK4_TRANSIT → FINISHED`，最终值固定为 `FINISHED`。
  - 异常上报 `TASK4_ERROR` 和普通错误字符串，不再查错误码目录。
  - 已兼容主状态机的 `ASSISTANCE_REQUEST`、`READINESS_CONFIRM`、`TASK4_BEGIN` 和 `resume`。
- [x] 在 VRX 中完成 Core 闭环验证并记录状态
  - 真实 WAM-V 使用约 8 m 非零距离目标运行，控制器日志显示距离从
    `8.00 m` 降到 `5.61 m`，随后状态进入 `TASK4_READY` 并停止左右推力。
  - 第二次运行 `command_seq=7002` 实时捕获 `TASK4_ACK → TASK4_READY`，
    中间状态通过 `/task4_core/state` 确认为 `TRANSIT`。
- [x] 在 VRX 中完成 Core 协议冒烟验证
  - 真实 WAM-V + GPS/IMU 位姿适配器 + `task4_core_node` 已输出
    `TASK4_ACK`、`TASK4_READY`，并确认 `TRANSIT` 状态。
  - 本次用当前 GPS 位置作为目标，验证的是协议和子状态机，不代表非零距离
    航行控制器已经完成。
- [x] 接入 Core 到 WAM-V 左右推进器的闭环控制器
  - `task4_controller_node` 消费 `/task4_core/setpoint`，发布左右 Float64 推力。
  - 仅在 `TASK4_ACK` / `TRANSIT` 驱动，输入超时或 `TASK4_READY` 后自动停车。
- [x] 增加 Task4 Core 最小反应式避障代码
  - 复用 Task3 已验证的 LaserScan 前向通道检测思路，不复制 Task3 状态机、topic 或 launch。
  - 连续帧确认、释放滞回、紧急停车、锁定左右绕行方向和决策有效期均有纯 Python 测试。
  - `task4_avoidance_node` 只发布 `PROCEED/HOLD/REPLAN`；VRX 控制器在最终推力边界执行。
  - LaserScan 或决策过期后停车；实船 frame 与控制器接入仍属于现场阻塞项。
- [x] 接入官方 Task 4 Core 通信链路
  - `task4_communication.py` 支持 `AssistanceRequest → IncidentAck → ReadinessReport → ReadinessConfirm`。
  - `RxCommand.seq` 作为 `command_seq`，USV 独立维护 `RxReport.seq`；严格校验 USV 类型、vehicle ID、重复命令和确认的 `report_seq`。
  - ROS 节点支持官方形状 JSON 和 OCS 已路由的 `task4_assistance` / `task4_readiness_confirm` 信封。
  - 通过 `/tongji_mrl/twave/vehicle_report` 输出 OCS 可接收的 `ack` 与 `task_report.official_report`。

以下内容暂不属于当前 Core 交付范围：全局绕行路径、移动目标预测、绕行候选点、最小安全距离证明、Advanced keep-out 区域和 Disruptive 级复杂避碰。

## 验收记录

| 项目 | 结果 | 证据 |
| --- | --- | --- |
| Gazebo Transport 回调 | PASS | `GZ_IP=127.0.0.1` 后 world-list 请求成功 |
| WAM-V spawn | PASS | `OK creation of entity` |
| WAM-V live pose | PASS | `/wamv/pose` 有实时 TFMessage |
| WAM-V live sensors | PASS | joint states / GPS / IMU 均可 `ros2 topic echo --once` |
| ENU→NED pose adapter | PASS | GPS/IMU fallback 发布 `Pose2D`，planner subscription count=1 |
| WAM-V 动力控制 | PASS | `task4_controller_node` 已接入左右 thrust，非零距离运行后进入 `TASK4_READY` 并停车 |
| 反应式避障离线回归 | PASS | 连续帧、单点噪声、紧急停车、方向锁定、释放滞回和推力门控测试通过 |
| 官方 Task 4 Core 通信适配 | PASS | 18 项纯 Python 测试通过；支持官方 RxCommand JSON 与 OCS vehicle-link 信封 |
| ROS 包构建与适配器回归 | PASS | `task4_usv_ros` 构建成功，ROS 侧 7 项测试通过 |
