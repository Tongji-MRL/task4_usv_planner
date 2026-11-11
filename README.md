# Task 4 USV Core

当前仓库只以 RobotX 2026 Task 4 的 Core tier 为交付目标，且独立于
Task 3 的 `dock_planner_v0`。

Core 只需要支持：收到指定 GPS 位置的重新任务、读取并上报 checkpoint、
发送 `IncidentAck`、航行到目标点、发送 `ReadinessReport`，再等待
`ReadinessConfirm` 后交回主规划器。Task4 不负责中断或恢复原任务，也不实现
Advanced/Disruptive tier 的移动目标预测、复杂绕行和最小安全距离证明。

规则依据：[RobotX 2026 Team Handbook（官网最新版本）](https://robotx.org/programs/2026/)。
官方链路是 `AssistanceRequest → IncidentAck → ReadinessReport → ReadinessConfirm`；
RoboCommand 只连接 OCS，USV 不直接连接 RoboCommand。

完成进度见 [VRX闭环仿真完成清单.md](VRX闭环仿真完成清单.md)。实船现场测试请严格按
[TASK4_FIELD_TEST_RUNBOOK.md](TASK4_FIELD_TEST_RUNBOOK.md) 分阶段执行；该手册明确区分
Orin Core executor、VRX 专用控制器和当前尚未确认的实船控制接口。

上一阶段面向 Advanced/Disruptive 的动态目标预测和复杂绕行已经移除。当前纯 Python Core 子状态机位于
`task4_usv_planner/core_state_machine.py`，不依赖 ROS 或 RoboCommand。
通信解析与序列校验位于 `task4_usv_planner/task4_communication.py`：支持官方
`RxCommand` JSON 形态，也支持 OCS 已路由的 `task4_assistance` /
`task4_readiness_confirm` 内部信封。ROS 节点会在 `vehicle_report_topic`
发布 OCS 可直接接收的 `ack` 和 `task_report` 信封；`report_topic` 保留官方
形状 JSON，便于仿真和抓包检查。实际 MQTT/protobuf 传输仍由 OCS 负责。

状态和错误上报位于 `task4_usv_planner/task4_status_codec.py`，采用主状态机
直接处理的简单 JSON 表：正常流程只使用 `TASK4_ACK`、`TASK4_TRANSIT`、
`TASK4_READY`、`TASK4_RESUMED`，正常消息的 `error` 为 `null`；错误使用一个
扁平的字符串 `error.code`，不再依赖 Task 3 的 14-bit 拼接状态字。ROS 节点
同时兼容主状态机的 `ASSISTANCE_REQUEST`、`READINESS_CONFIRM`，以及主状态机
发给 USV 的 `TASK4_BEGIN`、`resume`。

| 字段 | 正常值 | 作用 |
| --- | --- | --- |
| `task` | `TASK4` | 主状态机路由任务 |
| `progress` | `TASK4_ACK` / `TASK4_TRANSIT` / `TASK4_READY` / `TASK4_RESUMED` | 主状态机判断流程 |
| `error` | `null` | 正常；异常时为 `{ "code": "...", "detail": "..." }` |
| `command` | `null` | 保留 JSON 接口兼容，不参与进度判断 |

## Core 航行避障

Task4 现在包含一个最小反应式避障层，代码位于
`task4_usv_planner/reactive_avoidance.py`。它参考团队 Task3 分支
[`feature/task3-usv-only-geofence-search`](https://github.com/Tongji-MRL/robotx-dock-planner/tree/feature/task3-usv-only-geofence-search)
中的共享 `usv_reactive_avoidance`，但没有复制 Task3 的状态、launch 或任务 topic。

该层只处理 Core 前往援助点过程中的近距离安全：

- 连续多帧确认前向扫掠通道内的点簇，过滤单点噪声；
- 紧急距离内输出 `HOLD`；
- 可绕行距离内输出带锁定左右方向和降速比例的 `REPLAN`；
- 使用释放滞回，避免障碍边缘反复左右切换；
- 扫描丢失或决策超时后安全停车。

ROS 节点 `task4_avoidance_node` 默认订阅可配置的
`/mavros/obstacle/send` (`sensor_msgs/msg/LaserScan`)，发布 Task4 自有的
`/task4_core/avoidance` JSON 决策。VRX 专用 `task4_controller_node` 已在最终推力边界
消费这个决策；启动避障时应设置 `avoidance_required:=true`。默认值为 `false`，以免没有
LaserScan 的既有 VRX 场景被误锁死。

这不是全局 A*/RRT 路径规划，也不实现 keep-out 区域、移动目标预测或最小安全距离证明，
因此不应作为 Advanced/Disruptive 完成证据。实船控制器仍需单独确认能消费同一决策，
未经确认不得用 VRX 推力节点控制实船。
