"""Small closed-loop WAM-V controller for Task 4 Core setpoints.

This is intentionally a low-level VRX adapter, not a replacement for the
Task 4 state machine.  It consumes the NED ``Pose2D`` setpoint and publishes
the two bridged ``Float64`` thrust commands.  It only drives while Core is in
TASK4_ACK or TRANSIT, zeros both thrusters on stale input or any other state,
and accepts the optional Task 4 reactive-avoidance safety decision.
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

from geometry_msgs.msg import Pose2D as RosPose2D
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String


def normalize_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def differential_thrust(
    current: RosPose2D,
    target: RosPose2D,
    *,
    max_thrust: float = 300.0,
    distance_gain: float = 45.0,
    turn_gain: float = 150.0,
    turn_only_threshold_rad: float = 0.9,
    minimum_turn_thrust: float = 55.0,
) -> tuple[float, float, float, float]:
    """Return left thrust, right thrust, distance and heading error.

    Pose coordinates are local NED: x=north, y=east, yaw=heading from north.
    Positive heading error is corrected with more left thrust, matching the
    WAM-V differential-thrust direction used by the existing VRX tests.
    """

    north_delta = target.x - current.x
    east_delta = target.y - current.y
    distance = math.hypot(north_delta, east_delta)
    desired_heading = math.atan2(east_delta, north_delta)
    heading_error = normalize_angle(desired_heading - current.theta)

    if distance <= 1e-6:
        return 0.0, 0.0, distance, heading_error

    turn = clamp(turn_gain * heading_error, -max_thrust, max_thrust)
    if abs(heading_error) >= turn_only_threshold_rad:
        base = minimum_turn_thrust
    else:
        # Reduce forward thrust while the bow is not aligned with the target.
        base = min(max_thrust, distance_gain * distance) * max(
            0.0, math.cos(heading_error)
        )

    left = clamp(base + turn, 0.0, max_thrust)
    right = clamp(base - turn, 0.0, max_thrust)
    return left, right, distance, heading_error


def apply_avoidance_to_thrust(
    left: float,
    right: float,
    *,
    action: str,
    steering_side: Optional[str],
    speed_scale: float,
    max_thrust: float,
    turn_thrust: float,
) -> tuple[float, float]:
    """Apply a time-valid avoidance decision at the final command boundary."""

    if action == "PROCEED":
        return left, right
    if action == "HOLD":
        return 0.0, 0.0
    if action != "REPLAN" or steering_side not in ("LEFT", "RIGHT"):
        return 0.0, 0.0

    scale = clamp(float(speed_scale), 0.0, 1.0)
    forward = clamp(0.5 * (left + right) * scale, 0.0, max_thrust)
    bias = clamp(turn_thrust, 0.0, max_thrust)
    if steering_side == "LEFT":
        return (
            clamp(forward + bias, 0.0, max_thrust),
            clamp(forward - bias, 0.0, max_thrust),
        )
    return (
        clamp(forward - bias, 0.0, max_thrust),
        clamp(forward + bias, 0.0, max_thrust),
    )


class Task4ControllerNode(Node):
    """Translate Task 4 Core setpoints into WAM-V differential thrust."""

    ACTIVE_STATES = frozenset(("TASK4_ACK", "TRANSIT"))

    def __init__(self) -> None:
        super().__init__("task4_controller_node")
        self._setpoint_topic = str(
            self.declare_parameter("setpoint_topic", "/task4_core/setpoint").value
        )
        self._pose_topic = str(
            self.declare_parameter("pose_topic", "/task4_planner_node/current_pose").value
        )
        self._state_topic = str(
            self.declare_parameter("state_topic", "/task4_core/state").value
        )
        self._avoidance_topic = str(
            self.declare_parameter(
                "avoidance_topic", "/task4_core/avoidance"
            ).value
        )
        self._left_topic = str(
            self.declare_parameter("left_thrust_topic", "/wamv/thrusters/left/thrust").value
        )
        self._right_topic = str(
            self.declare_parameter("right_thrust_topic", "/wamv/thrusters/right/thrust").value
        )
        self._max_thrust = float(self.declare_parameter("max_thrust", 300.0).value)
        self._distance_gain = float(self.declare_parameter("distance_gain", 45.0).value)
        self._turn_gain = float(self.declare_parameter("turn_gain", 150.0).value)
        self._stop_radius = float(self.declare_parameter("stop_radius_m", 4.5).value)
        self._input_timeout = float(self.declare_parameter("input_timeout_s", 1.0).value)
        self._avoidance_timeout = float(
            self.declare_parameter("avoidance_timeout_s", 0.6).value
        )
        self._avoidance_required = bool(
            self.declare_parameter("avoidance_required", False).value
        )
        self._avoidance_turn_thrust = float(
            self.declare_parameter("avoidance_turn_thrust", 80.0).value
        )
        if (
            self._max_thrust <= 0.0
            or self._stop_radius <= 0.0
            or self._input_timeout <= 0.0
            or self._avoidance_timeout <= 0.0
            or self._avoidance_turn_thrust <= 0.0
        ):
            raise ValueError("controller distances, timeouts and thrust limits must be positive")

        self._current_pose: Optional[RosPose2D] = None
        self._target_pose: Optional[RosPose2D] = None
        self._state = "IDLE"
        self._last_pose_time = 0.0
        self._last_setpoint_time = 0.0
        self._last_log_time = 0.0
        self._avoidance_seen = False
        self._avoidance_action = "PROCEED"
        self._avoidance_side: Optional[str] = None
        self._avoidance_speed_scale = 1.0
        self._avoidance_valid_until = 0.0
        self._last_avoidance_time = 0.0

        self._left_publisher = self.create_publisher(Float64, self._left_topic, 10)
        self._right_publisher = self.create_publisher(Float64, self._right_topic, 10)
        self.create_subscription(RosPose2D, self._pose_topic, self._on_pose, 10)
        self.create_subscription(RosPose2D, self._setpoint_topic, self._on_setpoint, 10)
        self.create_subscription(String, self._state_topic, self._on_state, 10)
        self.create_subscription(String, self._avoidance_topic, self._on_avoidance, 10)
        self.create_timer(0.1, self._on_timer)
        self.get_logger().info(
            f"Task 4 controller ready: {self._setpoint_topic} -> "
            f"{self._left_topic}, {self._right_topic}; avoidance={self._avoidance_topic}"
        )

    def _on_pose(self, message: RosPose2D) -> None:
        self._current_pose = message
        self._last_pose_time = time.monotonic()

    def _on_setpoint(self, message: RosPose2D) -> None:
        self._target_pose = message
        self._last_setpoint_time = time.monotonic()

    def _on_state(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if isinstance(value, dict) and isinstance(value.get("state"), str):
                self._state = value["state"]
        except (TypeError, ValueError, json.JSONDecodeError):
            self._state = "INVALID"

    def _on_avoidance(self, message: String) -> None:
        self._avoidance_seen = True
        self._last_avoidance_time = time.monotonic()
        try:
            payload = json.loads(message.data)
            action = payload.get("action") if isinstance(payload, dict) else None
            side = payload.get("steering_side") if isinstance(payload, dict) else None
            scale = float(payload.get("speed_scale", 0.0))
            valid_until = float(payload["valid_until"])
            if action not in ("PROCEED", "HOLD", "REPLAN"):
                raise ValueError("unknown avoidance action")
            if action == "REPLAN" and side not in ("LEFT", "RIGHT"):
                raise ValueError("REPLAN requires LEFT or RIGHT steering_side")
            if not math.isfinite(scale) or not 0.0 <= scale <= 1.0:
                raise ValueError("invalid avoidance speed_scale")
            if not math.isfinite(valid_until):
                raise ValueError("invalid avoidance validity")
            self._avoidance_action = action
            self._avoidance_side = side
            self._avoidance_speed_scale = scale
            self._avoidance_valid_until = valid_until
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            # A malformed safety decision must never permit motion.
            self._avoidance_action = "HOLD"
            self._avoidance_side = None
            self._avoidance_speed_scale = 0.0
            self._avoidance_valid_until = time.time() + self._avoidance_timeout

    def _on_timer(self) -> None:
        now = time.monotonic()
        if (
            self._state not in self.ACTIVE_STATES
            or self._current_pose is None
            or self._target_pose is None
            or now - self._last_pose_time > self._input_timeout
            or now - self._last_setpoint_time > self._input_timeout
        ):
            self._publish_thrust(0.0, 0.0)
            return

        avoidance_fresh = (
            self._avoidance_seen
            and now - self._last_avoidance_time <= self._avoidance_timeout
            and time.time() <= self._avoidance_valid_until
        )
        if (self._avoidance_required or self._avoidance_seen) and not avoidance_fresh:
            self._publish_thrust(0.0, 0.0)
            return

        left, right, distance, heading_error = differential_thrust(
            self._current_pose,
            self._target_pose,
            max_thrust=self._max_thrust,
            distance_gain=self._distance_gain,
            turn_gain=self._turn_gain,
        )
        if distance <= self._stop_radius:
            left, right = 0.0, 0.0
        elif avoidance_fresh:
            left, right = apply_avoidance_to_thrust(
                left,
                right,
                action=self._avoidance_action,
                steering_side=self._avoidance_side,
                speed_scale=self._avoidance_speed_scale,
                max_thrust=self._max_thrust,
                turn_thrust=self._avoidance_turn_thrust,
            )
        self._publish_thrust(left, right)

        if now - self._last_log_time >= 1.0:
            self.get_logger().info(
                f"state={self._state} distance={distance:.2f}m "
                f"heading_error={math.degrees(heading_error):.1f}deg "
                f"avoidance={self._avoidance_action if avoidance_fresh else 'OFF'} "
                f"thrust=({left:.1f},{right:.1f})"
            )
            self._last_log_time = now

    def _publish_thrust(self, left: float, right: float) -> None:
        left_message = Float64()
        right_message = Float64()
        left_message.data = float(left)
        right_message.data = float(right)
        self._left_publisher.publish(left_message)
        self._right_publisher.publish(right_message)

    def destroy_node(self) -> bool:
        # During SIGINT rclpy may invalidate the context before finally() is
        # entered.  Best-effort zeroing is still useful, but shutdown must not
        # turn into a traceback if the publisher is already invalid.
        try:
            if rclpy.ok():
                self._publish_thrust(0.0, 0.0)
        except Exception:  # pragma: no cover - depends on shutdown timing
            pass
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Task4ControllerNode] = None
    try:
        node = Task4ControllerNode()
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
