"""Independent real-time safety-distance monitor for Task 4."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Optional, Tuple

from geometry_msgs.msg import Pose2D as RosPose2D
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String


@dataclass(frozen=True)
class MovingObjectState:
    north_m: Any
    east_m: Any
    heading_rad: Any
    speed_mps: Any
    timestamp_s: Any


@dataclass(frozen=True)
class SafetyResult:
    state: str
    distance_m: Optional[float]
    predicted_min_distance_m: Optional[float]
    reason: str


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def evaluate_safety(
    usv_pose_ned: Optional[Tuple[Any, Any]],
    observation: Optional[MovingObjectState],
    *,
    now_s: float,
    protected_radius_m: float = 12.0,
    warning_radius_m: float = 16.0,
    prediction_horizon_s: float = 5.0,
    max_observation_age_s: float = 5.0,
    invalid_reason: str = "",
) -> SafetyResult:
    """Evaluate current and short-horizon distance, failing safe on bad input."""
    if not _finite_number(now_s):
        return SafetyResult(
            "REPLAN_REQUIRED", None, None, "current time is invalid"
        )
    if (
        not _finite_number(protected_radius_m)
        or protected_radius_m <= 0
        or not _finite_number(warning_radius_m)
        or warning_radius_m <= protected_radius_m
        or not _finite_number(prediction_horizon_s)
        or prediction_horizon_s <= 0
        or not _finite_number(max_observation_age_s)
        or max_observation_age_s <= 0
    ):
        return SafetyResult(
            "REPLAN_REQUIRED", None, None, "safety monitor configuration is invalid"
        )
    if invalid_reason:
        return SafetyResult("REPLAN_REQUIRED", None, None, invalid_reason)
    if usv_pose_ned is None or len(usv_pose_ned) != 2:
        return SafetyResult(
            "REPLAN_REQUIRED", None, None, "USV pose is unavailable or invalid"
        )
    if observation is None:
        return SafetyResult(
            "REPLAN_REQUIRED", None, None, "moving-object observation is unavailable"
        )

    pose_north, pose_east = usv_pose_ned
    object_values = (
        observation.north_m,
        observation.east_m,
        observation.heading_rad,
        observation.speed_mps,
        observation.timestamp_s,
    )
    if not all(_finite_number(value) for value in (pose_north, pose_east, *object_values)):
        return SafetyResult(
            "REPLAN_REQUIRED",
            None,
            None,
            "USV pose or moving-object observation contains invalid numeric data",
        )
    if observation.speed_mps < 0:
        return SafetyResult(
            "REPLAN_REQUIRED",
            None,
            None,
            "moving-object speed is negative",
        )

    age_s = now_s - observation.timestamp_s
    if age_s < -1e-9:
        return SafetyResult(
            "REPLAN_REQUIRED", None, None, "moving-object observation is from the future"
        )
    if age_s > max_observation_age_s:
        return SafetyResult(
            "REPLAN_REQUIRED", None, None, "moving-object observation is stale"
        )

    heading = observation.heading_rad
    north_velocity = observation.speed_mps * math.cos(heading)
    east_velocity = observation.speed_mps * math.sin(heading)
    age_s = max(0.0, age_s)
    object_north = observation.north_m + north_velocity * age_s
    object_east = observation.east_m + east_velocity * age_s
    if not all(
        _finite_number(value)
        for value in (north_velocity, east_velocity, object_north, object_east)
    ):
        return SafetyResult(
            "REPLAN_REQUIRED",
            None,
            None,
            "moving-object projection contains invalid numeric data",
        )

    relative_north = object_north - pose_north
    relative_east = object_east - pose_east
    distance_m = math.hypot(relative_north, relative_east)
    velocity_squared = north_velocity ** 2 + east_velocity ** 2
    if velocity_squared <= 1e-18:
        closest_time_s = 0.0
    else:
        closest_time_s = min(
            prediction_horizon_s,
            max(
                0.0,
                -(relative_north * north_velocity + relative_east * east_velocity)
                / velocity_squared,
            ),
        )
    predicted_north = relative_north + north_velocity * closest_time_s
    predicted_east = relative_east + east_velocity * closest_time_s
    predicted_min_distance_m = math.hypot(predicted_north, predicted_east)
    if not all(
        _finite_number(value)
        for value in (distance_m, predicted_min_distance_m)
    ):
        return SafetyResult(
            "REPLAN_REQUIRED",
            None,
            None,
            "computed safety distance is invalid",
        )

    if distance_m < protected_radius_m:
        return SafetyResult(
            "EMERGENCY",
            distance_m,
            predicted_min_distance_m,
            "current USV-to-object distance is below the protected radius",
        )
    if predicted_min_distance_m < warning_radius_m:
        return SafetyResult(
            "REPLAN_REQUIRED",
            distance_m,
            predicted_min_distance_m,
            "predicted distance within the short horizon is below the warning radius",
        )
    return SafetyResult(
        "SAFE",
        distance_m,
        predicted_min_distance_m,
        "current and predicted distances satisfy the configured margins",
    )


class Task4SafetyMonitorNode(Node):
    """Monitor live pose and object updates independently of the planner."""

    def __init__(self) -> None:
        super().__init__("task4_safety_monitor_node")

        self._protected_radius_m = float(
            self.declare_parameter("protected_radius_m", 12.0).value
        )
        self._warning_radius_m = float(
            self.declare_parameter("warning_radius_m", 16.0).value
        )
        self._prediction_horizon_s = float(
            self.declare_parameter("prediction_horizon_s", 5.0).value
        )
        self._max_observation_age_s = float(
            self.declare_parameter("max_observation_age_s", 5.0).value
        )
        monitor_rate_hz = float(self.declare_parameter("monitor_rate_hz", 10.0).value)
        if (
            not _finite_number(monitor_rate_hz)
            or monitor_rate_hz <= 0
            or not _finite_number(self._protected_radius_m)
            or self._protected_radius_m <= 0
            or not _finite_number(self._warning_radius_m)
            or self._warning_radius_m <= self._protected_radius_m
            or not _finite_number(self._prediction_horizon_s)
            or self._prediction_horizon_s <= 0
            or not _finite_number(self._max_observation_age_s)
            or self._max_observation_age_s <= 0
        ):
            raise ValueError("safety monitor parameters must be positive and warning > protected")

        self._usv_pose_ned: Optional[Tuple[Any, Any]] = None
        self._observation: Optional[MovingObjectState] = None
        self._invalid_reason = ""

        self._state_publisher = self.create_publisher(String, "~/safety_state", 10)
        self._distance_publisher = self.create_publisher(Float64, "~/distance_m", 10)
        self._predicted_distance_publisher = self.create_publisher(
            Float64, "~/predicted_min_distance_m", 10
        )
        self._reason_publisher = self.create_publisher(String, "~/reason", 10)
        self.create_subscription(
            RosPose2D,
            "/task4_planner_node/current_pose",
            self._on_usv_pose,
            10,
        )
        self.create_subscription(
            String,
            "/task4_planner_node/moving_object",
            self._on_moving_object,
            10,
        )
        self._timer = self.create_timer(1.0 / monitor_rate_hz, self._evaluate_and_publish)
        self.get_logger().info(
            f"Safety monitor ready at {monitor_rate_hz:g} Hz; "
            f"protected={self._protected_radius_m:g} m, "
            f"warning={self._warning_radius_m:g} m"
        )

    def _on_usv_pose(self, message: RosPose2D) -> None:
        self._usv_pose_ned = (message.x, message.y)
        self._invalid_reason = ""

    def _on_moving_object(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("moving-object payload must be a JSON object")
            self._observation = MovingObjectState(
                north_m=payload["north_m"],
                east_m=payload["east_m"],
                heading_rad=payload["heading_rad"],
                speed_mps=payload["speed_mps"],
                timestamp_s=payload["timestamp_s"],
            )
            self._invalid_reason = ""
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._observation = None
            self._invalid_reason = f"moving-object observation is invalid: {exc}"
            self.get_logger().error(self._invalid_reason)

    def _evaluate_and_publish(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1_000_000_000.0
        result = evaluate_safety(
            self._usv_pose_ned,
            self._observation,
            now_s=now_s,
            protected_radius_m=self._protected_radius_m,
            warning_radius_m=self._warning_radius_m,
            prediction_horizon_s=self._prediction_horizon_s,
            max_observation_age_s=self._max_observation_age_s,
            invalid_reason=self._invalid_reason,
        )

        state_message = String()
        state_message.data = result.state
        self._state_publisher.publish(state_message)
        distance_message = Float64()
        distance_message.data = (
            result.distance_m if result.distance_m is not None else math.nan
        )
        self._distance_publisher.publish(distance_message)
        predicted_distance_message = Float64()
        predicted_distance_message.data = (
            result.predicted_min_distance_m
            if result.predicted_min_distance_m is not None
            else math.nan
        )
        self._predicted_distance_publisher.publish(predicted_distance_message)
        reason_message = String()
        reason_message.data = result.reason
        self._reason_publisher.publish(reason_message)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Task4SafetyMonitorNode()
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
