"""ROS 2 wrapper for the Task 4 USV Core sub-state machine."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Optional

from geometry_msgs.msg import Pose2D as RosPose2D
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from task4_usv_planner import (
    CommonError,
    CoreCheckpoint,
    CorePlannerConfig,
    CoreTask4Begin,
    GpsReference,
    Pose2D,
    Task4Step,
    Task4CoreStateMachine,
    encode_progress,
)
from task4_usv_planner.core_state_machine import CoreState


class Task4CoreNode(Node):
    """Translate the team JSON topics into the pure Core state machine."""

    def __init__(self) -> None:
        super().__init__("task4_core_node")
        self._task_cmd_topic = str(
            self.declare_parameter("task_cmd_topic", "/tongji_mrl/twave/task_cmd").value
        )
        self._checkpoint_topic = str(
            self.declare_parameter("checkpoint_topic", "/tongji_mrl/twave/checkpoint").value
        )
        self._progress_topic = str(
            self.declare_parameter("progress_topic", "/tongji_mrl/twave/progress").value
        )
        self._gps_topic = str(
            self.declare_parameter("gps_topic", "/wamv/sensors/gps/gps/fix").value
        )
        self._pose_topic = str(
            self.declare_parameter(
                "current_pose_topic", "/task4_planner_node/current_pose"
            ).value
        )
        self._setpoint_topic = str(
            self.declare_parameter("setpoint_topic", "/task4_core/setpoint").value
        )
        self._state_topic = str(
            self.declare_parameter("state_topic", "/task4_core/state").value
        )
        self._checkpoint_file = str(self.declare_parameter("checkpoint_file", "").value)
        self._checkpoint_task = str(
            self.declare_parameter("checkpoint_task", "UNKNOWN").value
        )
        self._checkpoint_progress = str(
            self.declare_parameter("checkpoint_progress", "UNKNOWN").value
        )
        self._machine = Task4CoreStateMachine(CorePlannerConfig(
            arrival_radius_m=float(
                self.declare_parameter("arrival_radius_m", 5.0).value
            ),
            max_transit_distance_m=float(
                self.declare_parameter("max_transit_distance_m", 1000.0).value
            ),
        ))
        self._reference: Optional[GpsReference] = None
        self._current_pose: Optional[Pose2D] = None
        self._pending_begin: Optional[dict[str, Any]] = None

        self._checkpoint_publisher = self.create_publisher(String, self._checkpoint_topic, 10)
        self._progress_publisher = self.create_publisher(String, self._progress_topic, 10)
        self._state_publisher = self.create_publisher(String, self._state_topic, 10)
        self._setpoint_publisher = self.create_publisher(RosPose2D, self._setpoint_topic, 10)
        self.create_subscription(String, self._task_cmd_topic, self._on_task_cmd, 10)
        self.create_subscription(NavSatFix, self._gps_topic, self._on_gps, 10)
        self.create_subscription(RosPose2D, self._pose_topic, self._on_pose, 10)
        self.create_timer(0.1, self._on_timer)
        self.get_logger().info(
            f"Task 4 Core ready: task_cmd={self._task_cmd_topic}, "
            f"checkpoint={self._checkpoint_topic}, progress={self._progress_topic}"
        )

    def _on_pose(self, message: RosPose2D) -> None:
        self._current_pose = Pose2D(message.x, message.y, message.theta)
        self._try_start_pending_begin()

    def _on_gps(self, message: NavSatFix) -> None:
        if not all(value == value and abs(value) != float("inf") for value in (
            message.latitude, message.longitude, message.altitude,
        )):
            return
        if self._reference is None:
            self._reference = GpsReference(
                message.latitude, message.longitude, message.altitude
            )
            self.get_logger().info(
                f"Task 4 GPS reference initialized at "
                f"({message.latitude:.8f}, {message.longitude:.8f})"
            )
            self._try_start_pending_begin()

    def _on_task_cmd(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict) or str(payload.get("vehicle", "")).lower() != "usv":
                return
            command = payload.get("command")
            if isinstance(command, dict) and command.get("type") == "TASK4_BEGIN":
                self._pending_begin = {"envelope": payload, "command": command}
                self._try_start_pending_begin()
            elif payload.get("command") == "resume":
                self._handle_resume(payload)
            else:
                self._publish_error(
                    "unsupported Task4 command",
                    error=CommonError.INVALID_INPUT,
                    detail_code="COMMAND_TYPE_UNSUPPORTED",
                    command_seq=_optional_command_seq(payload),
                )
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Rejected Task4 task_cmd: {exc}")
            self._publish_error(
                str(exc),
                error=CommonError.INVALID_INPUT,
                detail_code="MALFORMED_TASK_CMD",
            )

    def _try_start_pending_begin(self) -> None:
        if self._pending_begin is None or self._reference is None or self._current_pose is None:
            return
        command = self._pending_begin["command"]
        position = command.get("position", {})
        try:
            begin = CoreTask4Begin.from_gps(
                command_seq=int(command["command_seq"]),
                latitude_deg=float(position["latitude"]),
                longitude_deg=float(position["longitude"]),
                altitude_m=(
                    float(position["altitude"])
                    if "altitude" in position else None
                ),
                reference=self._reference,
                target_vehicle=str(command.get("target_vehicle", "uav")),
            )
            checkpoint = self._load_checkpoint()
            snapshot = self._machine.receive_task4_begin(
                begin, checkpoint, self._current_pose
            )
            self._publish_checkpoint(checkpoint)
            self._publish_snapshot(snapshot)
            self._pending_begin = None
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Rejected TASK4_BEGIN: {exc}")
            error = (
                0b1001
                if "maximum transit distance" in str(exc)
                else 0b1000
            )
            step = Task4Step.GO_TO_TARGET if error == 0b1001 else Task4Step.READY
            self._publish_error(
                str(exc),
                error=error,
                step=step,
                detail_code=(
                    "TARGET_UNREACHABLE" if error == 0b1001 else "TASK4_BEGIN_INVALID"
                ),
                command_seq=_optional_command_seq(command),
            )
            self._pending_begin = None

    def _on_timer(self) -> None:
        if self._current_pose is None:
            return
        snapshot = self._machine.tick(self._current_pose)
        self._publish_snapshot(snapshot)

    def _handle_resume(self, payload: dict[str, Any]) -> None:
        try:
            snapshot = self._machine.receive_resume(
                str(payload["task"]), str(payload["progress"])
            )
            self._publish_snapshot(snapshot)
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            self.get_logger().error(f"Rejected Task4 resume: {exc}")
            self._publish_error(
                str(exc),
                error=0b1001,
                step=Task4Step.WAIT_RESUME,
                detail_code="RESUME_COMMAND_REJECTED",
                command_seq=_optional_command_seq(payload),
            )

    def _load_checkpoint(self) -> CoreCheckpoint:
        data: dict[str, Any] = {}
        if self._checkpoint_file:
            path = Path(self._checkpoint_file)
            if path.exists():
                loaded = json.loads(path.read_text(encoding="utf-8"))
                if not isinstance(loaded, dict):
                    raise ValueError("checkpoint file must contain a JSON object")
                data = loaded
        return CoreCheckpoint(
            vehicle=str(data.get("vehicle", "usv")),
            task=str(data.get("task", self._checkpoint_task)),
            progress=str(data.get("progress", self._checkpoint_progress)),
            error=data.get("error"),
            command=data.get("command"),
        )

    def _publish_checkpoint(self, checkpoint: CoreCheckpoint) -> None:
        output = String()
        output.data = json.dumps({
            "vehicle": checkpoint.vehicle,
            "task": checkpoint.task,
            "progress": checkpoint.progress,
            "error": checkpoint.error,
            "command": "checkpoint",
        }, ensure_ascii=False)
        self._checkpoint_publisher.publish(output)

    def _publish_snapshot(self, snapshot: Any) -> None:
        if snapshot.progress_to_report:
            output = String()
            output.data = json.dumps(
                encode_progress(
                    snapshot.progress_to_report,
                    command_seq=snapshot.command_seq,
                ),
                ensure_ascii=False,
            )
            self._progress_publisher.publish(output)
        if snapshot.active_setpoint is not None:
            output = RosPose2D()
            output.x = snapshot.active_setpoint.x
            output.y = snapshot.active_setpoint.y
            output.theta = snapshot.active_setpoint.yaw
            self._setpoint_publisher.publish(output)
        state = String()
        status = self._status_for_snapshot(snapshot)
        state.data = json.dumps({
            "state": snapshot.state.value,
            "command_seq": snapshot.command_seq,
            "reason": snapshot.reason,
            "resume_task": snapshot.resume_task,
            "resume_progress": snapshot.resume_progress,
            "status_code": status.pack(),
            "status_bits": status.bit_string(),
        }, ensure_ascii=False)
        self._state_publisher.publish(state)

    def _publish_error(
        self,
        detail: str,
        *,
        error: int | CommonError,
        step: int | Task4Step | None = None,
        detail_code: str,
        command_seq: int | None = None,
    ) -> None:
        """Publish a Task3-compatible structured fault on the progress topic."""

        snapshot = self._machine.snapshot()
        output = String()
        output.data = json.dumps(
            encode_progress(
                "TASK4_ERROR",
                command_seq=command_seq if command_seq is not None else snapshot.command_seq,
                step=step if step is not None else _status_step(snapshot.state),
                error=error,
                detail=detail,
                fault_source="task4_core_node",
                detail_code=detail_code,
            ),
            ensure_ascii=False,
        )
        self._progress_publisher.publish(output)

    @staticmethod
    def _status_for_snapshot(snapshot: Any):
        from task4_usv_planner import Task4StatusCode

        return Task4StatusCode(step=_status_step(snapshot.state))


def _status_step(state: CoreState) -> Task4Step:
    return {
        CoreState.IDLE: Task4Step.READY,
        CoreState.TASK4_ACK: Task4Step.READY,
        CoreState.TRANSIT: Task4Step.GO_TO_TARGET,
        CoreState.TASK4_READY: Task4Step.TARGET_REACHED,
        CoreState.RESUMED: Task4Step.RESUMING,
    }[state]


def _optional_command_seq(value: Any) -> int | None:
    raw = value.get("command_seq") if isinstance(value, dict) else None
    if isinstance(raw, bool) or not isinstance(raw, int) or raw <= 0:
        return None
    return raw


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Task4CoreNode] = None
    try:
        node = Task4CoreNode()
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
