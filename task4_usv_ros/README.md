# Task 4 ROS 2 adapter

`task4_usv_ros` is a ROS 2 Humble `ament_python` wrapper around the pure
Python `task4_usv_planner` package. It only accepts planning inputs and
publishes planner output; it does not send vehicle commands or connect to
Gazebo, VRX, MAVROS, OCS, protobuf, or a controller.

## Topics

The node is named `task4_planner_node`. Its private topics resolve under
`/task4_planner_node`:

| Topic | Type | Contract |
| --- | --- | --- |
| `~/current_pose` | `geometry_msgs/msg/Pose2D` | `x=north_m`, `y=east_m`, `theta=yaw_rad` |
| `~/mission_path` | `nav_msgs/msg/Path` | Each pose uses `position.x=north_m`, `position.y=east_m`; orientation supplies yaw |
| `~/moving_object` | `std_msgs/msg/String` | JSON object: `north_m`, `east_m`, optional `down_m`, `heading_rad`, `speed_mps`, `timestamp_s` |
| `~/plan` | `std_msgs/msg/String` | JSON object: `status`, `planned_waypoints` (`north_m`, `east_m`, `yaw_rad`), `reason`, `minimum_clearance_m` |

The moving-object timestamp and the node clock must use the same time basis.
The adapter reads current time from the ROS clock, so `use_sim_time` may be
enabled by the caller when appropriate. `course_boundary_ned` is an optional
JSON-string ROS parameter containing ordered `[north_m, east_m]` vertices,
for example `[[0, 0], [100, 0], [100, 50], [0, 50]]`.

The node publishes a new plan when any input is updated, once all three inputs
have been received. Malformed input is reported to the ROS logger and is not
used. Planner results, including `INVALID_INPUT`, are published as output.

## Task 4 supervisor

Run the separate supervisor with:

```bash
ros2 run task4_usv_ros task4_supervisor_node
```

The supervisor subscribes to `/task4_planner_node/plan` (`std_msgs/msg/String`)
and publishes `~/state` (`std_msgs/msg/String`) and `~/active_path`
(`nav_msgs/msg/Path`) under `/task4_supervisor_node`. Planner results map to
states as follows:

| Planner status | Supervisor state |
| --- | --- |
| `PROCEED` | `TASK4_PROCEED` |
| `DETOUR` | `TASK4_DETOUR` |
| `HOLD` | `TASK4_HOLD` |
| `EMERGENCY_ESCAPE` | `TASK4_ESCAPE` |
| `ABORT` | `TASK4_ABORT` |
| `INVALID_INPUT` | `TASK4_HOLD` |

`PROCEED`, `DETOUR`, and `EMERGENCY_ESCAPE` replace the active path with the
planner's `planned_waypoints`. For `HOLD` (including `INVALID_INPUT`) and
`ABORT`, the supervisor publishes an empty path, explicitly clearing any
previous detour or escape path. The path frame is `ned`; `position.x`,
`position.y`, and orientation yaw carry north, east, and yaw respectively.
Unknown or malformed planner results also fail safe to `TASK4_HOLD` with an
empty path. The initial state is `NORMAL` with an empty path. There is no
automatic transition back to `NORMAL` in this phase, and the supervisor never
publishes speed, thrust, or flight-controller commands.

## Build and run

From the repository root, with ROS 2 Humble sourced:

```bash
colcon build --packages-select task4_usv_ros
source install/setup.bash
ros2 run task4_usv_ros task4_planner_node
```

The supervisor can be started alongside the planner node:

```bash
ros2 run task4_usv_ros task4_supervisor_node
```

## Independent safety monitor

Run the safety monitor as a separate process:

```bash
ros2 run task4_usv_ros task4_safety_monitor_node
```

It independently subscribes to `/task4_planner_node/current_pose`
(`geometry_msgs/msg/Pose2D`) and `/task4_planner_node/moving_object`
(`std_msgs/msg/String`, the same local-NED JSON input used by the planner).
It evaluates at `monitor_rate_hz` (default `10.0 Hz`), compensates the object
position from `timestamp_s` using its heading and speed, then computes the
current distance and the minimum distance over `prediction_horizon_s`
(default `5.0 s`). For the short-horizon forecast, the current USV position is
held fixed because this input contract does not provide USV velocity.

Parameters are `protected_radius_m` (default `12.0`),
`warning_radius_m` (default `16.0`), `prediction_horizon_s` (default `5.0`),
`max_observation_age_s` (default `5.0`), and `monitor_rate_hz` (default
`10.0`). The warning radius must be greater than the protected radius. The
node publishes `~/safety_state` (`std_msgs/msg/String`, `SAFE`,
`REPLAN_REQUIRED`, or `EMERGENCY`), `~/distance_m` and
`~/predicted_min_distance_m` (`std_msgs/msg/Float64`). Missing, malformed,
invalid, future-dated, or stale inputs yield `REPLAN_REQUIRED`, with unavailable
distances represented as NaN. `~/reason` (`std_msgs/msg/String`) explains the
current result, including invalid-data reasons. This monitor does not modify
the supervisor state or send any vehicle control commands.
