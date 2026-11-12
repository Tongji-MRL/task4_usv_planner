"""ROS 2 wrapper for the Task 4 USV Core sub-state machine."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Optional

from geometry_msgs.msg import Pose2D as RosPose2D
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from task4_usv_planner import (
    CoreCheckpoint,
    CorePlannerConfig,
    CoreTask4Begin,
    GpsReference,
    Pose2D,
    Task4CoreStateMachine,
    Task4CommunicationError,
    Task4CommunicationSession,
    parse_assistance_request,
    parse_readiness_confirm,
    normalize_main_fsm_command,
    normalize_ocs_command,
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
        self._report_topic = str(
            self.declare_parameter(
                "report_topic", "/tongji_mrl/twave/task4_report"
            ).value
        )
        self._vehicle_report_topic = str(
            self.declare_parameter(
                "vehicle_report_topic", "/tongji_mrl/twave/vehicle_report"
            ).value
        )
        self._vehicle_id = str(self.declare_parameter("vehicle_id", "USV1").value)
        self._team_id = str(self.declare_parameter("team_id", "").value)
        self._run_id = int(self.declare_parameter("run_id", 0).value)
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
        self._communication = Task4CommunicationSession(
            vehicle_id=self._vehicle_id,
            team_id=self._team_id,
        )

        self._checkpoint_publisher = self.create_publisher(String, self._checkpoint_topic, 10)
        self._progress_publisher = self.create_publisher(String, self._progress_topic, 10)
        self._report_publisher = self.create_publisher(String, self._report_topic, 10)
        self._vehicle_report_publisher = self.create_publisher(
            String, self._vehicle_report_topic, 10
        )
        self._state_publisher = self.create_publisher(String, self._state_topic, 10)
        self._setpoint_publisher = self.create_publisher(RosPose2D, self._setpoint_topic, 10)
        self.create_subscription(String, self._task_cmd_topic, self._on_task_cmd, 10)
        self.create_subscription(NavSatFix, self._gps_topic, self._on_gps, 10)
        self.create_subscription(RosPose2D, self._pose_topic, self._on_pose, 10)
        self.create_timer(0.1, self._on_timer)
        self.get_logger().info(
            f"Task 4 Core ready: task_cmd={self._task_cmd_topic}, "
            f"checkpoint={self._checkpoint_topic}, progress={self._progress_topic}, "
            f"official_report={self._report_topic}, vehicle_report={self._vehicle_report_topic}"
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
            if not isinstance(payload, dict):
                return
            if _looks_like_main_fsm_task4(payload):
                if str(payload.get("vehicle", "")).lower() != "usv":
                    return
                self._handle_official_command(
                    normalize_main_fsm_command(payload, vehicle_id=self._vehicle_id)
                )
                return
            if _looks_like_official_task4(payload):
                self._handle_official_command(payload)
                return
            if str(payload.get("vehicle", "")).lower() != "usv":
                return
            command = payload.get("command")
            command_type = str(command.get("type", "")).upper() if isinstance(command, dict) else ""
            if command_type == "TASK4_BEGIN":
                if str(command.get("target_vehicle", "usv")).lower() != "usv":
                    return
                self._pending_begin = {"envelope": payload, "command": command}
                self._try_start_pending_begin()
            elif payload.get("command") == "resume":
                self._handle_resume(payload)
            else:
                self._publish_failure("unsupported Task4 command")
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Rejected Task4 task_cmd: {exc}")
            self._publish_failure(str(exc))

    def _handle_official_command(self, payload: dict[str, Any]) -> None:
        """Handle the JSON bridge equivalent of the official RxCommand oneof."""

        try:
            if self._run_id and payload.get("run_id") not in (None, self._run_id):
                raise Task4CommunicationError(
                    f"command belongs to run_id={payload.get('run_id')}, expected {self._run_id}"
                )
            payload = normalize_ocs_command(payload)
            kind = _official_kind(payload)
            if kind in {"assistance_request", "assistancerequest"}:
                request = parse_assistance_request(
                    payload, expected_vehicle_id=self._vehicle_id
                )
                event = self._communication.accept_assistance(request)
                if not event.accepted:
                    self._publish_failure(event.reason)
                    return
                # The official Core requirement is an immediate IncidentAck;
                # the actual begin waits until GPS and pose are available.
                self._publish_official(self._communication.incident_ack(request.command_seq))
                self._pending_begin = {
                    "envelope": payload,
                    "command": {
                        "type": "TASK4_BEGIN",
                        "command_seq": request.command_seq,
                        "position": {
                            "latitude": request.latitude_deg,
                            "longitude": request.longitude_deg,
                            **({"altitude": request.altitude_m} if request.altitude_m is not None else {}),
                        },
                        "target_vehicle": "usv",
                    },
                    "official": True,
                }
                self._try_start_pending_begin()
                return

            if kind in {"readiness_confirm", "readinessconfirm"}:
                confirm = parse_readiness_confirm(
                    payload, expected_vehicle_id=self._vehicle_id
                )
                event = self._communication.accept_readiness_confirm(confirm)
                if not event.accepted:
                    self._publish_failure(event.reason)
                    return
                checkpoint = self._machine.snapshot().checkpoint
                if checkpoint is None:
                    raise Task4CommunicationError(
                        "cannot resume without the interrupted-task checkpoint"
                    )
                self._handle_resume_values(checkpoint.task, checkpoint.progress)
                return

            raise Task4CommunicationError(f"unsupported official Task 4 command: {kind or 'missing type'}")
        except (KeyError, TypeError, ValueError, Task4CommunicationError) as exc:
            self.get_logger().error(f"Rejected official Task 4 command: {exc}")
            self._publish_failure(str(exc))

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
                target_vehicle=str(command.get("target_vehicle", "usv")),
            )
            checkpoint = self._load_checkpoint()
            snapshot = self._machine.receive_task4_begin(
                begin, checkpoint, self._current_pose
            )
            self._publish_checkpoint(checkpoint)
            self._publish_snapshot(snapshot)
            if self._pending_begin.get("official", False):
                self._publish_vehicle_ack(
                    command_seq=begin.command_seq,
                    state="TASK4_ACK",
                    detail="checkpoint captured; Task 4 assistance accepted",
                )
            self._pending_begin = None
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Rejected TASK4_BEGIN: {exc}")
            self._publish_failure(str(exc))
            self._pending_begin = None

    def _on_timer(self) -> None:
        if self._current_pose is None:
            return
        snapshot = self._machine.tick(self._current_pose)
        self._publish_snapshot(snapshot)

    def _handle_resume(self, payload: dict[str, Any]) -> None:
        try:
            self._handle_resume_values(str(payload["task"]), str(payload["progress"]))
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            self.get_logger().error(f"Rejected Task4 resume: {exc}")
            self._publish_failure(str(exc))

    def _handle_resume_values(self, task: str, progress: str) -> None:
        snapshot = self._machine.receive_resume(task, progress)
        self._publish_snapshot(snapshot)

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
            self._publish_progress(snapshot.progress_to_report)
            if snapshot.progress_to_report == "TASK4_FINISH":
                try:
                    report = self._communication.readiness_report(snapshot.command_seq)
                    self._publish_official(report)
                    self._publish_vehicle_task_report(report)
                except (Task4CommunicationError, TypeError, ValueError) as exc:
                    self.get_logger().error(f"Unable to publish ReadinessReport: {exc}")
                    self._publish_failure(str(exc))
        if snapshot.active_setpoint is not None:
            output = RosPose2D()
            output.x = snapshot.active_setpoint.x
            output.y = snapshot.active_setpoint.y
            output.theta = snapshot.active_setpoint.yaw
            self._setpoint_publisher.publish(output)
        state = String()
        state.data = json.dumps({
            "state": snapshot.state.value,
            "progress": _progress_for_snapshot(snapshot),
            "command_seq": snapshot.command_seq,
            "reason": snapshot.reason,
            "resume_task": snapshot.resume_task,
            "resume_progress": snapshot.resume_progress,
        }, ensure_ascii=False)
        self._state_publisher.publish(state)

    def _publish_progress(self, progress: str, *, error: str | None = None) -> None:
        """Publish only the progress contract required by the main FSM."""

        output = String()
        output.data = json.dumps(
            _progress_payload(progress, error=error), ensure_ascii=False
        )
        self._progress_publisher.publish(output)

    def _publish_official(self, payload: dict[str, Any]) -> None:
        output = String()
        output.data = json.dumps(payload, ensure_ascii=False)
        self._report_publisher.publish(output)

    def _publish_vehicle_ack(self, *, command_seq: int, state: str, detail: str) -> None:
        self._publish_vehicle_envelope(
            "ack",
            {
                "command_seq": command_seq,
                "accepted": True,
                "state": state,
                "detail": detail,
            },
        )

    def _publish_vehicle_task_report(self, official_report: dict[str, Any]) -> None:
        body = official_report.get("body", {})
        report = {
            "type": body.get("type", ""),
            "vehicle_id": official_report.get("vehicle_id", self._vehicle_id),
            "seq": official_report.get("seq"),
            "command_seq": body.get("command_seq"),
        }
        self._publish_vehicle_envelope(
            "task_report",
            {"official_report": report},
        )

    def _publish_vehicle_envelope(self, kind: str, payload: dict[str, Any]) -> None:
        """Publish the OCS vehicle-link envelope consumed by the MQTT bridge."""

        output = String()
        output.data = json.dumps(
            {
                "protocol_version": "1",
                "message_id": f"task4-{self._vehicle_id}-{int(time.time() * 1000)}",
                "sent_at_unix_ms": int(time.time() * 1000),
                "run_id": self._run_id or None,
                "vehicle_id": self._vehicle_id,
                "source": self._vehicle_id,
                "frame_id": "task4_core",
                "kind": kind,
                "payload": payload,
            },
            ensure_ascii=False,
        )
        self._vehicle_report_publisher.publish(output)

    def _publish_failure(self, detail: str) -> None:
        """Log diagnostics locally and expose failure only as progress."""

        self.get_logger().error(detail)
        self._publish_progress("TASK4_ERROR", error=detail)

def _progress_for_snapshot(snapshot: Any) -> str:
    return {
        CoreState.IDLE: "TASK4_IDLE",
        CoreState.TASK4_ACK: "TASK4_ACK",
        CoreState.TRANSIT: "TASK4_TRANSIT",
        CoreState.TASK4_READY: "TASK4_FINISH",
        CoreState.FINISHED: "TASK4_FINISH",
    }[snapshot.state]


def _progress_payload(
    progress: str, *, error: str | None = None
) -> dict[str, str | None]:
    if not isinstance(progress, str) or not progress:
        raise ValueError("progress must be a non-empty string")
    if error is not None and (not isinstance(error, str) or not error):
        raise ValueError("error must be null or a non-empty string")
    return {
        "vehicle": "usv",
        "task": "TASK4",
        "progress": progress,
        "error": error,
    }


def _official_kind(payload: dict[str, Any]) -> str:
    if payload.get("kind") == "command" and isinstance(payload.get("payload"), dict):
        command = str(payload["payload"].get("command", "")).lower()
        return {
            "task4_assistance": "assistance_request",
            "task4_readiness_confirm": "readiness_confirm",
        }.get(command, command)
    body = payload.get("body")
    if not isinstance(body, dict):
        body = payload
    raw = body.get("type", body.get("message_type", body.get("command")))
    if raw is None and isinstance(payload.get("body"), dict):
        for key in (
            "assistance_request",
            "assistanceRequest",
            "readiness_confirm",
            "readinessConfirm",
        ):
            if key in payload["body"]:
                raw = key
                break
    if isinstance(raw, dict):
        raw = raw.get("type", raw.get("message_type"))
    return str(raw or "").replace("-", "_").replace(" ", "_").lower()


def _looks_like_official_task4(payload: dict[str, Any]) -> bool:
    return _official_kind(payload) in {
        "assistance_request",
        "assistancerequest",
        "readiness_confirm",
        "readinessconfirm",
    }


def _looks_like_main_fsm_task4(payload: dict[str, Any]) -> bool:
    command = payload.get("command")
    return (
        payload.get("task") == "TASK4"
        and isinstance(command, dict)
        and str(command.get("type", "")).upper()
        in {"ASSISTANCE_REQUEST", "READINESS_CONFIRM"}
    )


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
