"""Publish Task 4 reactive-avoidance decisions from a shared LaserScan."""

from __future__ import annotations

from dataclasses import asdict
import json
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from task4_usv_planner import ReactiveAvoidanceConfig, ReactiveAvoidanceCore


class Task4AvoidanceNode(Node):
    """Monitor the forward corridor; never publish actuator commands itself."""

    ACTIVE_STATES = frozenset(("TASK4_ACK", "TRANSIT"))

    def __init__(self) -> None:
        super().__init__("task4_avoidance_node")
        defaults = asdict(ReactiveAvoidanceConfig())
        values = {
            name: self.declare_parameter(name, value).value
            for name, value in defaults.items()
        }
        self._core = ReactiveAvoidanceCore(ReactiveAvoidanceConfig(**values))
        self._scan_timeout_s = float(
            self.declare_parameter("scan_timeout_s", 0.5).value
        )
        self._scan_topic = str(
            self.declare_parameter("scan_topic", "/mavros/obstacle/send").value
        )
        self._state_topic = str(
            self.declare_parameter("state_topic", "/task4_core/state").value
        )
        self._output_topic = str(
            self.declare_parameter("output_topic", "/task4_core/avoidance").value
        )
        if self._scan_timeout_s <= 0.0:
            raise ValueError("scan_timeout_s must be positive")

        self._state = "IDLE"
        self._last_scan_at: Optional[float] = None
        self._publisher = self.create_publisher(String, self._output_topic, 10)
        self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(String, self._state_topic, self._on_state, 10)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            f"Task 4 avoidance monitor ready: {self._scan_topic} -> {self._output_topic}"
        )

    def _on_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            next_state = payload.get("state") if isinstance(payload, dict) else None
        except (AttributeError, TypeError, json.JSONDecodeError):
            next_state = None
        if not isinstance(next_state, str):
            self._state = "INVALID"
            self._core.reset()
            return
        if next_state not in self.ACTIVE_STATES and self._state in self.ACTIVE_STATES:
            self._core.reset()
        self._state = next_state

    def _on_scan(self, message: LaserScan) -> None:
        self._last_scan_at = time.monotonic()
        if self._state not in self.ACTIVE_STATES:
            return
        now = time.time()
        result = self._core.evaluate(
            message.ranges,
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=message.range_max,
            timestamp=now,
        )
        payload = asdict(result)
        payload["action"] = result.action.value
        payload["steering_side"] = (
            None if result.steering_side is None else result.steering_side.value
        )
        payload["state"] = self._state
        self._publish(payload)

    def _watchdog(self) -> None:
        if self._state not in self.ACTIVE_STATES:
            return
        if (
            self._last_scan_at is not None
            and time.monotonic() - self._last_scan_at <= self._scan_timeout_s
        ):
            return
        now = time.time()
        self._publish({
            "action": "HOLD",
            "timestamp": now,
            "valid_until": now + self._core.config.decision_validity_s,
            "reason": "LaserScan missing or stale",
            "nearest_forward_m": None,
            "steering_side": None,
            "speed_scale": 0.0,
            "obstacle_points": 0,
            "state": self._state,
        })

    def _publish(self, payload: dict) -> None:
        self._publisher.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Task4AvoidanceNode] = None
    try:
        node = Task4AvoidanceNode()
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
