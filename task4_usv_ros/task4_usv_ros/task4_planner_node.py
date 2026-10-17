"""ROS 2 adapter node for the pure Python Task 4 USV planner."""

from __future__ import annotations

import json
import math
from typing import Any, Dict, Optional, Tuple

from geometry_msgs.msg import Pose2D as RosPose2D
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from task4_usv_planner import (
    MovingObjectObservation,
    NEDPoint,
    Pose2D,
    Task4DynamicPlanner,
    Task4PlannerConfig,
)


def _yaw_from_quaternion(orientation: Any) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


class Task4PlannerNode(Node):
    """Subscribe to mission state, run the planner, and publish its decision."""

    def __init__(self) -> None:
        super().__init__("task4_planner_node")

        boundary_json = self.declare_parameter("course_boundary_ned", "").value
        boundary: Optional[Tuple[Tuple[float, float], ...]] = None
        if boundary_json:
            parsed_boundary = json.loads(boundary_json)
            if not isinstance(parsed_boundary, list):
                raise ValueError("course_boundary_ned must be a JSON array of [north, east] pairs")
            boundary = tuple(
                (vertex[0], vertex[1])
                for vertex in parsed_boundary
                if isinstance(vertex, list) and len(vertex) == 2
            )
            if len(boundary) != len(parsed_boundary):
                raise ValueError("course_boundary_ned vertices must be [north, east] pairs")

        self._planner = Task4DynamicPlanner(
            Task4PlannerConfig(course_boundary_ned=boundary)
        )
        self._current_pose: Optional[Pose2D] = None
        self._mission_waypoints: Optional[Tuple[Pose2D, ...]] = None
        self._observation: Optional[MovingObjectObservation] = None

        self._result_publisher = self.create_publisher(
            String, "~/plan", 10,
        )
        self.create_subscription(
            RosPose2D, "~/current_pose", self._on_current_pose, 10,
        )
        self.create_subscription(
            Path, "~/mission_path", self._on_mission_path, 10,
        )
        self.create_subscription(
            String, "~/moving_object", self._on_moving_object, 10,
        )

        self.get_logger().info(
            "Task 4 planner ready; awaiting current_pose, mission_path, and moving_object"
        )

    def _on_current_pose(self, message: RosPose2D) -> None:
        try:
            self._current_pose = Pose2D(message.x, message.y, message.theta)
            self._publish_plan_if_ready()
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"Rejected current pose: {exc}")

    def _on_mission_path(self, message: Path) -> None:
        try:
            self._mission_waypoints = tuple(
                Pose2D(
                    pose.pose.position.x,
                    pose.pose.position.y,
                    _yaw_from_quaternion(pose.pose.orientation),
                )
                for pose in message.poses
            )
            self._publish_plan_if_ready()
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"Rejected mission path: {exc}")

    def _on_moving_object(self, message: String) -> None:
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError("moving-object payload must be a JSON object")
            self._observation = MovingObjectObservation(
                position_ned=NEDPoint(
                    data["north_m"],
                    data["east_m"],
                    data.get("down_m", 0.0),
                ),
                heading_rad=data["heading_rad"],
                speed_mps=data["speed_mps"],
                timestamp=data["timestamp_s"],
            )
            self._publish_plan_if_ready()
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Rejected moving-object observation: {exc}")

    def _publish_plan_if_ready(self) -> None:
        if (
            self._current_pose is None
            or self._mission_waypoints is None
            or self._observation is None
        ):
            return

        now_s = self.get_clock().now().nanoseconds / 1_000_000_000.0
        result = self._planner.plan(
            self._current_pose,
            self._mission_waypoints,
            self._observation,
            now_s=now_s,
        )
        payload: Dict[str, Any] = {
            "status": result.status.value,
            "planned_waypoints": [
                {"north_m": point.x, "east_m": point.y, "yaw_rad": point.yaw}
                for point in result.waypoints
            ],
            "reason": result.reason,
            "minimum_clearance_m": result.minimum_clearance_m,
        }
        output = String()
        output.data = json.dumps(payload, allow_nan=False)
        self._result_publisher.publish(output)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Task4PlannerNode] = None
    try:
        node = Task4PlannerNode()
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
