"""Fail-safe Task 4 planner-result supervisor for ROS 2."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Tuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class SupervisorDecision:
    state: str
    waypoints: Tuple[Tuple[float, float, float], ...]


_STATUS_TO_STATE = {
    "PROCEED": "TASK4_PROCEED",
    "DETOUR": "TASK4_DETOUR",
    "HOLD": "TASK4_HOLD",
    "EMERGENCY_ESCAPE": "TASK4_ESCAPE",
    "ABORT": "TASK4_ABORT",
    "INVALID_INPUT": "TASK4_HOLD",
}


def resolve_planner_result(payload: Any) -> SupervisorDecision:
    """Map planner JSON to a state and a replacement active path."""
    if not isinstance(payload, dict):
        return SupervisorDecision("TASK4_HOLD", ())

    state = _STATUS_TO_STATE.get(payload.get("status"))
    if state is None:
        return SupervisorDecision("TASK4_HOLD", ())
    if state in ("TASK4_HOLD", "TASK4_ABORT"):
        return SupervisorDecision(state, ())

    planned_waypoints = payload.get("planned_waypoints")
    if not isinstance(planned_waypoints, list):
        return SupervisorDecision("TASK4_HOLD", ())

    waypoints = []
    for waypoint in planned_waypoints:
        if not isinstance(waypoint, dict):
            return SupervisorDecision("TASK4_HOLD", ())
        coordinates = (
            waypoint.get("north_m"),
            waypoint.get("east_m"),
            waypoint.get("yaw_rad"),
        )
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in coordinates
        ):
            return SupervisorDecision("TASK4_HOLD", ())
        waypoints.append(tuple(float(value) for value in coordinates))

    if not waypoints:
        return SupervisorDecision("TASK4_HOLD", ())
    return SupervisorDecision(state, tuple(waypoints))


class Task4SupervisorNode(Node):
    """Translate planner decisions to Task 4 state and active-path topics."""

    def __init__(self) -> None:
        super().__init__("task4_supervisor_node")
        self._state_publisher = self.create_publisher(String, "~/state", 10)
        self._path_publisher = self.create_publisher(Path, "~/active_path", 10)
        self.create_subscription(
            String,
            "/task4_planner_node/plan",
            self._on_planner_result,
            10,
        )

        self._publish_decision(SupervisorDecision("NORMAL", ()))
        self.get_logger().info(
            "Task 4 supervisor ready; awaiting /task4_planner_node/plan"
        )

    def _on_planner_result(self, message: String) -> None:
        try:
            payload: Any = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Malformed planner result; entering TASK4_HOLD: {exc}")
            self._publish_decision(SupervisorDecision("TASK4_HOLD", ()))
            return

        decision = resolve_planner_result(payload)
        planner_status = payload.get("status") if isinstance(payload, dict) else None
        if decision.state == "TASK4_HOLD" and planner_status not in (
            "HOLD",
            "INVALID_INPUT",
        ):
            self.get_logger().error(
                "Planner result is invalid or unsupported; entering TASK4_HOLD"
            )
        self._publish_decision(decision)

    def _publish_decision(self, decision: SupervisorDecision) -> None:
        state_message = String()
        state_message.data = decision.state
        self._state_publisher.publish(state_message)

        path_message = Path()
        path_message.header.frame_id = "ned"
        path_message.header.stamp = self.get_clock().now().to_msg()
        for north, east, yaw in decision.waypoints:
            pose = PoseStamped()
            pose.header = path_message.header
            pose.pose.position.x = north
            pose.pose.position.y = east
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_message.poses.append(pose)
        self._path_publisher.publish(path_message)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Task4SupervisorNode()
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
