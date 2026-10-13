# Task 4 USV 规划核心

`task4_usv_planner/task4_usv_planner/task4_dynamic.py` 是一个独立于 Task 3 `dock_planner_v0` 的纯 Python 规划核心，不依赖 ROS 2、RoboCommand、protobuf 或执行器。它实现的是 RobotX 2026 Task 4 Disruptive 的 USV 侧第一版几何逻辑，不是最终控制器。

## 输入

- 当前 USV `Pose2D`；
- 当前 Task1/Task3 剩余航点；
- 移动水面目标在 USV NED 中的位置、航向、速度和时间戳；
- 当前时间。

外层以后负责把官方 MovingObjectAlert 的 GPS、heading、speed 转到这个输入契约。当前模块不处理 protobuf、GPS 转换或 ROS topic。

## 规划策略

1. 按匀速直线模型预测目标未来一段时间的位置。
2. 将预测轨迹膨胀成动态禁入走廊：

   ```text
   保护半径 = 10 m 规则 + USV 船体包络 + 定位/预测余量
   ```

3. 如果原任务剩余路径与走廊保持安全距离，返回 `PROCEED`。
4. 如果发生冲突，分别尝试沿预测轨迹左右两侧绕行，并对整条候选路径做连续线段净距检查。
5. 如果简单绕行无法证明安全但当前点仍安全，返回 `HOLD`。
6. 如果当前 USV 已经进入保护走廊，返回 `EMERGENCY_ESCAPE`，优先生成远离目标轨迹的脱离点。
7. 如果既不能安全绕行也不能安全保持，返回 `ABORT`，由上层执行零速/人工接管策略。

当前设计故意偏保守：把整个预测轨迹当成空间走廊检查，因此不会因为“理论上可以抢在目标前面穿过”而放行。后续有真实船动力学、边界和动态障碍输入后，再把它升级成时空规划器。

## 当前边界

- 目标模型是假设匀速直线运动；
- 未接入航道边界、静态障碍和 USV 动力学；
- 左右绕行是可证明安全的少量候选，不是完整全局搜索；
- `PROCEED` 只代表几何上通过当前预测走廊检查，不代表船一定能跟踪；
- 观测过期、坐标不一致或目标未来轨迹无法安全解释时，应由上层停止继续沿用旧路径。

运行测试：

```bash
cd /home/y/robotx_ws/task4_usv_planner
PYTHONPATH=. python3 -m unittest tests.test_task4_dynamic -v
```
