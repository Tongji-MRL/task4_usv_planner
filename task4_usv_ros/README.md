# Task 4 Core ROS adapter

这个 ROS 包当前提供 Task 4 Core 子状态机、WAM-V 位姿输入适配器和最小反应式
避障层。旧的 moving-object 接口、复杂动态 planner 和最小安全距离证明已经移除。

## Core task node

`task4_core_node` 使用 `std_msgs/msg/String` JSON 话题接收
`/tongji_mrl/twave/task_cmd`，并发布 checkpoint、progress、状态和直达
setpoint。它支持官方 Core 的 `AssistanceRequest`、`IncidentAck`、
`ReadinessReport`、`ReadinessConfirm` 链路，同时兼容仿真使用的
`TASK4_BEGIN`/`resume` 命令。默认输出 `/tongji_mrl/twave/task4_report`（官方
形状 JSON）和 `/tongji_mrl/twave/vehicle_report`（OCS vehicle-link 信封）。
正式接入时将 `vehicle_id` 设置为 OCS 声明的 USV ID，并按需设置 `team_id`、
`run_id`。默认使用 WAM-V 首个 GPS fix 作为本地 NED 原点；如果已有
progress 程序，应通过 `checkpoint_file` 参数提供其 JSON checkpoint 文件，
否则使用 `checkpoint_task` 和 `checkpoint_progress` 参数。默认到达半径为
5 m，最大直达距离为 1000 m，可通过 `arrival_radius_m` 和
`max_transit_distance_m` 调整。

```bash
ros2 run task4_usv_ros task4_core_node
```

## VRX 闭环控制器

`task4_controller_node` 消费 `/task4_core/setpoint` 和实时 NED 位姿，向
`/wamv/thrusters/left/thrust`、`/wamv/thrusters/right/thrust` 发布限幅后的
差分推力。只有 Core 处于 `TASK4_ACK` 或 `TRANSIT` 时才驱动；到达停止半径、
输入超时、`TASK4_READY` 或 `FINISHED` 时会把左右推力清零。收到有效避障决策后，
`HOLD` 清零推力，`REPLAN` 降速并按锁定方向转向；避障决策一旦接入，后续超时也会停车。

```bash
ros2 run task4_usv_ros task4_controller_node
```

## 反应式避障监视器

`task4_avoidance_node` 默认订阅 `/mavros/obstacle/send`
(`sensor_msgs/msg/LaserScan`)，只在 `TASK4_ACK` / `TRANSIT` 中发布
`/task4_core/avoidance`。它不直接发布推进器命令。雷达丢失、扫描无效或过期时发布
`HOLD`；稳定障碍进入绕行距离时发布 `REPLAN`；紧急距离内发布 `HOLD`。

```bash
ros2 run task4_usv_ros task4_avoidance_node --ros-args \
  -p scan_topic:=/mavros/obstacle/send

ros2 run task4_usv_ros task4_controller_node --ros-args \
  -p avoidance_required:=true
```

扫描 topic 可以替换，但输入必须是船体前向为 0 rad 的 `LaserScan`；在 frame、角度方向和
距离单位实测确认前禁止启用运动。该节点只做近场反应式安全，不是全局路径规划。

## VRX WAM-V pose adapter

`task4_pose_adapter_node` 订阅：

- `/wamv/pose` (`tf2_msgs/msg/TFMessage`)
- `/wamv/sensors/gps/gps/fix` (`sensor_msgs/msg/NavSatFix`)
- `/wamv/sensors/imu/imu/data` (`sensor_msgs/msg/Imu`)

并发布 `/task4_planner_node/current_pose`
(`geometry_msgs/msg/Pose2D`)。它把 Gazebo ENU 转成 Task 4 使用的本地
NED，默认以首个 GPS fix 作为局部原点。VRX 当前没有发布 WAM-V 底盘的
世界位姿时，适配器会自动使用 GPS + IMU fallback。

```bash
ros2 run task4_usv_ros task4_pose_adapter_node
```

Core 状态机、位姿适配器、反应式避障和 VRX 推进器控制器已经连通；正式系统接入时必须
把避障决策和 setpoint 一起接到唯一实船控制边界，不能只替换 thrust topic 就视为完成。

## Progress 接口

Task4 已移除独立状态码、错误码目录和编码模块。参考 Task3 子状态机的 FSM 边界，
`/tongji_mrl/twave/progress` 只发布：

```json
{"vehicle":"usv","task":"TASK4","progress":"TASK4_TRANSIT","error":null}
```

正常顺序是 `TASK4_ACK → TASK4_TRANSIT → TASK4_FINISH`，最终值固定为
`TASK4_FINISH`。异常发布 `TASK4_ERROR`，`error` 是普通可读字符串，不是编码。
内部状态仍供控制器和避障安全门使用。到达目标后进入 `TASK4_READY` 等待恢复指令，
收到主状态机明确恢复指令后进入最终状态 `FINISHED`，不会自动恢复原任务。

通信适配器支持主状态机的 `ASSISTANCE_REQUEST` / `READINESS_CONFIRM`，并保留
主状态机实际发给 USV 的 `TASK4_BEGIN` / `resume`。通信层仍会拒绝非 USV 目标、
旧/重复 command sequence、错误 vehicle ID 和不匹配的 readiness report sequence。
