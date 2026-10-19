"""Minimal Task 4 Core sub-state machine from the team Task4 protocol."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple

from .types import NEDPoint, Pose2D
from .geodesy import GpsReference


class CoreState(str, Enum):
    """Task4-owned states; the main planner owns the interrupted task."""

    IDLE = "IDLE"
    TASK4_ACK = "TASK4_ACK"
    TRANSIT = "TRANSIT"
    TASK4_READY = "TASK4_READY"
    RESUMED = "RESUMED"


@dataclass(frozen=True)
class CoreCheckpoint:
    """The progress record that Task4 reports before accepting the task."""

    vehicle: str
    task: str
    progress: str
    error: Optional[str] = None
    command: Optional[str] = None

    def __post_init__(self) -> None:
        if not self.vehicle or not self.task or not self.progress:
            raise ValueError("checkpoint vehicle, task and progress are required")


@dataclass(frozen=True)
class CoreTask4Begin:
    """Decoded TASK4_BEGIN command; transport/protobuf stays outside the core."""

    command_seq: int
    target_ned: NEDPoint
    target_vehicle: str = "usv"

    def __post_init__(self) -> None:
        if self.command_seq <= 0:
            raise ValueError("command_seq must be positive")
        if not self.target_vehicle:
            raise ValueError("target_vehicle must not be empty")

    @classmethod
    def from_gps(
        cls,
        command_seq: int,
        latitude_deg: float,
        longitude_deg: float,
        reference: GpsReference,
        altitude_m: Optional[float] = None,
        target_vehicle: str = "uav",
    ) -> "CoreTask4Begin":
        return cls(
            command_seq=command_seq,
            target_ned=reference.to_ned(latitude_deg, longitude_deg, altitude_m),
            target_vehicle=target_vehicle,
        )


@dataclass(frozen=True)
class CorePlannerConfig:
    arrival_radius_m: float = 5.0
    max_transit_distance_m: float = 1000.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.arrival_radius_m) or self.arrival_radius_m <= 0.0:
            raise ValueError("arrival_radius_m must be positive and finite")
        if not math.isfinite(self.max_transit_distance_m) or self.max_transit_distance_m <= 0.0:
            raise ValueError("max_transit_distance_m must be positive and finite")


@dataclass(frozen=True)
class CoreSnapshot:
    """State, direct setpoint and protocol outputs for the outer ROS node."""

    state: CoreState
    command_seq: Optional[int]
    checkpoint: Optional[CoreCheckpoint]
    target_ned: Optional[NEDPoint]
    active_setpoint: Optional[Pose2D]
    progress_to_report: Optional[str]
    resume_task: Optional[str]
    resume_progress: Optional[str]
    reason: str


def _distance_xy(first: Pose2D, second: NEDPoint) -> float:
    return math.hypot(first.x - second.north, first.y - second.east)


def _direct_setpoint(current_pose: Pose2D, target: NEDPoint) -> Pose2D:
    north_delta = target.north - current_pose.x
    east_delta = target.east - current_pose.y
    distance = math.hypot(north_delta, east_delta)
    heading = current_pose.yaw if distance <= 1e-9 else math.atan2(east_delta, north_delta)
    return Pose2D(target.north, target.east, heading)


class Task4CoreStateMachine:
    """Handle only Task4's USV sub-flow, not the main mission lifecycle."""

    def __init__(self, config: CorePlannerConfig = CorePlannerConfig()) -> None:
        self._config = config
        self._state = CoreState.IDLE
        self._command: Optional[CoreTask4Begin] = None
        self._checkpoint: Optional[CoreCheckpoint] = None
        self._target: Optional[NEDPoint] = None
        self._setpoint: Optional[Pose2D] = None
        self._progress_to_report: Optional[str] = None
        self._resume_task: Optional[str] = None
        self._resume_progress: Optional[str] = None
        self._reason = "waiting for TASK4_BEGIN"

    def receive_task4_begin(
        self,
        command: CoreTask4Begin,
        checkpoint: CoreCheckpoint,
        current_pose: Pose2D,
    ) -> CoreSnapshot:
        """Store the target, report checkpoint externally, then acknowledge Task4."""

        target_distance = _distance_xy(current_pose, command.target_ned)
        if target_distance > self._config.max_transit_distance_m:
            raise ValueError(
                "TASK4 target exceeds the configured maximum transit distance "
                f"({target_distance:.1f} m > {self._config.max_transit_distance_m:.1f} m)"
            )
        self._command = command
        self._checkpoint = checkpoint
        self._target = command.target_ned
        self._setpoint = _direct_setpoint(current_pose, command.target_ned)
        self._progress_to_report = "TASK4_ACK"
        self._resume_task = None
        self._resume_progress = None
        self._state = CoreState.TASK4_ACK
        self._reason = "checkpoint captured; TASK4_BEGIN acknowledged"
        return self.snapshot()

    def tick(self, current_pose: Pose2D) -> CoreSnapshot:
        """Advance ACK/transit/ready using the latest USV pose."""

        self._progress_to_report = None
        if self._state == CoreState.TASK4_ACK:
            self._state = CoreState.TRANSIT
            self._reason = "transiting directly to the Task4 target"
        elif self._state == CoreState.TRANSIT and self._target is not None:
            if _distance_xy(current_pose, self._target) <= self._config.arrival_radius_m:
                self._state = CoreState.TASK4_READY
                self._progress_to_report = "TASK4_READY"
                self._reason = "target reached; TASK4_READY reported"
        return self.snapshot()

    def receive_resume(self, task: str, progress: str) -> CoreSnapshot:
        """Record the main planner's resume instruction without restoring a route."""

        if self._state != CoreState.TASK4_READY:
            raise RuntimeError("resume is only accepted after TASK4_READY")
        if not task or not progress:
            raise ValueError("resume task and progress are required")
        self._resume_task = task
        self._resume_progress = progress
        self._setpoint = None
        self._state = CoreState.RESUMED
        self._reason = "resume command recorded; main planner owns continuation"
        return self.snapshot()

    def reset(self) -> CoreSnapshot:
        """Return the Task4 sub-state machine to its idle state."""

        self._state = CoreState.IDLE
        self._command = None
        self._checkpoint = None
        self._target = None
        self._setpoint = None
        self._progress_to_report = None
        self._resume_task = None
        self._resume_progress = None
        self._reason = "waiting for TASK4_BEGIN"
        return self.snapshot()

    def snapshot(self) -> CoreSnapshot:
        return CoreSnapshot(
            state=self._state,
            command_seq=self._command.command_seq if self._command else None,
            checkpoint=self._checkpoint,
            target_ned=self._target,
            active_setpoint=self._setpoint,
            progress_to_report=self._progress_to_report,
            resume_task=self._resume_task,
            resume_progress=self._resume_progress,
            reason=self._reason,
        )


__all__ = [
    "CoreCheckpoint",
    "CorePlannerConfig",
    "CoreSnapshot",
    "CoreState",
    "CoreTask4Begin",
    "Task4CoreStateMachine",
]
