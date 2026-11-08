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

上一阶段的动态避碰实现已经移除。当前纯 Python Core 子状态机位于
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
