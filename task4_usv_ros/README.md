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

## Build and run

From the repository root, with ROS 2 Humble sourced:

```bash
colcon build --packages-select task4_usv_ros
source install/setup.bash
ros2 run task4_usv_ros task4_planner_node
```
