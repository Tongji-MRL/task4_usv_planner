# Task 4 USV Core

当前仓库只以 RobotX 2026 Task 4 的 Core tier 为交付目标，且独立于
Task 3 的 `dock_planner_v0`。

Core 只需要支持：收到指定 GPS 位置的重新任务、读取并上报 checkpoint、
发送 `TASK4_ACK`、航行到目标点、发送 `TASK4_READY`，再等待主规划器的
resume 指令。Task4 不负责中断或恢复原任务，也不实现 Advanced/Disruptive
tier 的移动目标预测、复杂绕行和最小安全距离证明。

完成进度见 [VRX闭环仿真完成清单.md](VRX闭环仿真完成清单.md)。

上一阶段的动态避碰实现已经移除。当前纯 Python Core 子状态机位于
`task4_usv_planner/core_state_machine.py`，不依赖 ROS 或 RoboCommand。
状态字和错误上报位于 `task4_usv_planner/task4_status_codec.py`，沿用 Task 3
的 14-bit 编码，ROS 节点会把非法命令、目标超距和非法 resume 编码后发布。
