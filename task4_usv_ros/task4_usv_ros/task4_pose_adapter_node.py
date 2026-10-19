"""Convert the VRX WAM-V pose from Gazebo ENU into Task 4 local NED."""

from __future__ import annotations

import math
from typing import Any, Iterable, Optional, Tuple

from geometry_msgs.msg import Pose2D as RosPose2D
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(orientation: Any) -> float:
    """Return ENU yaw from a ROS quaternion."""

    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle to [-pi, pi)."""

    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def enu_to_ned_pose(
    x_enu: float,
    y_enu: float,
    z_enu: float,
    yaw_enu_rad: float,
    origin_enu: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Tuple[float, float, float, float]:
    """Convert a Gazebo ENU pose to Task 4's local NED convention.

    Task 4 uses ``north=x``, ``east=y`` and ``down=z`` in its own local NED
    coordinates. Gazebo's world pose is ENU: ``x=east``, ``y=north`` and
    ``z=up``. The heading conversion is therefore ``pi/2 - yaw_enu``.
    """

    origin_x, origin_y, origin_z = origin_enu
    north_m = y_enu - origin_y
    east_m = x_enu - origin_x
    down_m = -(z_enu - origin_z)
    heading_ned_rad = normalize_angle(math.pi / 2.0 - yaw_enu_rad)
    return north_m, east_m, down_m, heading_ned_rad


def select_base_transform(
    transforms: Iterable[Any], base_frame: str,
) -> Optional[Any]:
    """Select the WAM-V base transform from a Pose_V-to-TFMessage bridge."""

    transforms = tuple(transforms)
    normalized = base_frame.strip("/")
    exact = [
        transform for transform in transforms
        if transform.child_frame_id.strip("/") == normalized
    ]
    if exact:
        return exact[0]

    suffix = "/" + normalized.rsplit("/", 1)[-1]
    suffix_matches = [
        transform for transform in transforms
        if transform.child_frame_id.strip("/").endswith(suffix)
    ]
    if suffix_matches:
        return suffix_matches[0]

    # A future bridge configuration may publish only one transform. Keeping
    # this fallback makes the adapter useful with both model-pose variants,
    # while the multi-transform case still fails safe instead of guessing.
    return transforms[0] if len(transforms) == 1 else None


def gps_to_local_ned(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    origin: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Convert a small-area GPS fix to local NED metres."""

    earth_radius_m = 6_378_137.0
    origin_lat_deg, origin_lon_deg, origin_alt_m = origin
    origin_lat_rad = math.radians(origin_lat_deg)
    north_m = math.radians(latitude_deg - origin_lat_deg) * earth_radius_m
    east_m = (
        math.radians(longitude_deg - origin_lon_deg)
        * earth_radius_m
        * math.cos(origin_lat_rad)
    )
    down_m = -(altitude_m - origin_alt_m)
    return north_m, east_m, down_m


class Task4PoseAdapterNode(Node):
    """Publish the live WAM-V pose on the Task 4 planner's NED input topic."""

    def __init__(self) -> None:
        super().__init__("task4_pose_adapter_node")

        self._pose_topic = str(self.declare_parameter("pose_topic", "/wamv/pose").value)
        self._gps_topic = str(
            self.declare_parameter("gps_topic", "/wamv/sensors/gps/gps/fix").value
        )
        self._imu_topic = str(
            self.declare_parameter("imu_topic", "/wamv/sensors/imu/imu/data").value
        )
        self._output_topic = str(
            self.declare_parameter(
                "output_topic", "/task4_planner_node/current_pose"
            ).value
        )
        self._base_frame = str(self.declare_parameter("base_frame", "wamv/base_link").value)
        self._use_first_pose_as_origin = bool(
            self.declare_parameter("use_first_pose_as_origin", True).value
        )
        self._origin_enu = (
            float(self.declare_parameter("origin_enu_x_m", 0.0).value),
            float(self.declare_parameter("origin_enu_y_m", 0.0).value),
            float(self.declare_parameter("origin_enu_z_m", 0.0).value),
        )
        self._origin_initialized = not self._use_first_pose_as_origin
        self._gps_origin: Optional[Tuple[float, float, float]] = None
        self._latest_gps: Optional[NavSatFix] = None
        self._latest_imu: Optional[Imu] = None
        self._warned_missing_transform = False
        self._warned_gps_fallback = False

        self._publisher = self.create_publisher(RosPose2D, self._output_topic, 10)
        self.create_subscription(TFMessage, self._pose_topic, self._on_pose, 10)
        self.create_subscription(NavSatFix, self._gps_topic, self._on_gps, 10)
        self.create_subscription(Imu, self._imu_topic, self._on_imu, 10)
        self.get_logger().info(
            f"Task 4 ENU->NED pose adapter ready: {self._pose_topic} -> "
            f"{self._output_topic}; base_frame={self._base_frame}; "
            f"GPS/IMU fallback={self._gps_topic}, {self._imu_topic}"
        )

    def _on_pose(self, message: TFMessage) -> None:
        transform = select_base_transform(message.transforms, self._base_frame)
        if transform is None:
            if not self._warned_missing_transform:
                self.get_logger().warning(
                    "No unambiguous WAM-V base transform found in /wamv/pose; "
                    "waiting for GPS/IMU fallback"
                )
                self._warned_missing_transform = True
            return

        translation = transform.transform.translation
        if not self._origin_initialized:
            self._origin_enu = (translation.x, translation.y, translation.z)
            self._origin_initialized = True
            self.get_logger().info(
                "Initialized local NED origin from the first WAM-V pose: "
                f"ENU=({translation.x:.2f}, {translation.y:.2f}, {translation.z:.2f})"
            )

        north_m, east_m, _down_m, heading_ned_rad = enu_to_ned_pose(
            translation.x,
            translation.y,
            translation.z,
            yaw_from_quaternion(transform.transform.rotation),
            self._origin_enu,
        )
        output = RosPose2D()
        output.x = north_m
        output.y = east_m
        output.theta = heading_ned_rad
        self._publisher.publish(output)

    def _on_gps(self, message: NavSatFix) -> None:
        if not all(math.isfinite(value) for value in (
            message.latitude, message.longitude, message.altitude,
        )):
            return
        self._latest_gps = message
        if self._gps_origin is None:
            self._gps_origin = (
                message.latitude,
                message.longitude,
                message.altitude,
            )
            self.get_logger().info(
                "Initialized local NED origin from the first GPS fix: "
                f"lat={message.latitude:.8f}, lon={message.longitude:.8f}"
            )
        self._publish_gps_imu_pose()

    def _on_imu(self, message: Imu) -> None:
        self._latest_imu = message
        self._publish_gps_imu_pose()

    def _publish_gps_imu_pose(self) -> None:
        if self._latest_gps is None or self._latest_imu is None or self._gps_origin is None:
            return
        if not self._warned_gps_fallback:
            self.get_logger().info(
                "Using VRX GPS/IMU fallback because /wamv/pose has no WAM-V base pose"
            )
            self._warned_gps_fallback = True
        north_m, east_m, _down_m = gps_to_local_ned(
            self._latest_gps.latitude,
            self._latest_gps.longitude,
            self._latest_gps.altitude,
            self._gps_origin,
        )
        output = RosPose2D()
        output.x = north_m
        output.y = east_m
        output.theta = normalize_angle(
            math.pi / 2.0 - yaw_from_quaternion(self._latest_imu.orientation)
        )
        self._publisher.publish(output)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Task4PoseAdapterNode] = None
    try:
        node = Task4PoseAdapterNode()
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
