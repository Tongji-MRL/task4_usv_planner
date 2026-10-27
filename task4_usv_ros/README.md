# Task 4 Core ROS adapter

这个 ROS 包当前提供 Task 4 Core 子状态机和 WAM-V 位姿输入适配器。旧的
动态避碰 planner、moving-object 接口、复杂 supervisor 和安全距离监视器
已经移除。

## Core task node

`task4_core_node` 使用 `std_msgs/msg/String` JSON 话题接收
`/tongji_mrl/twave/task_cmd`，并发布 checkpoint、progress、状态和直达
setpoint。它支持 PDF 中的 `TASK4_BEGIN`、`TASK4_ACK`、`TASK4_READY` 和
`resume` 流程。默认使用 WAM-V 首个 GPS fix 作为本地 NED 原点；如果已有
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
输入超时、`TASK4_READY` 或 `RESUMED` 时会把左右推力清零。

```bash
ros2 run task4_usv_ros task4_controller_node
```

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

Core 状态机、位姿适配器和 VRX 推进器控制器已经连通；正式系统接入时只需
按现场控制接口替换左右 thrust topic 或控制器实现。

## 状态与错误编码

`task4_usv_planner/task4_status_codec.py` 与 Task 3 使用相同的 14-bit
诊断布局：`vehicle(1) | task(2) | progress(4) | error(4) | command(3)`。
正常 progress 的 `error` 为 `null`；非法命令、目标超距、非法 resume 等会在
`/tongji_mrl/twave/progress` 发布 `TASK4_ERROR`，其中包含 `status_code`、
`status_bits`、错误名称、重试属性、恢复动作、来源、细节和 `command_seq`。
这样主规划器可以按 Task 3 的方式解码，而无需解析日志文本。
