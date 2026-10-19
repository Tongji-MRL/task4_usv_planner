"""Task 4 Core status and fault encoding.

The layout is kept identical to the Task 3 diagnostic word so the main
planner can decode both child state machines uniformly::

    vehicle(1) | task(2) | progress(4) | error(4) | command(3)

The JSON progress message remains human-readable and carries the packed word
as an additional diagnostic field.  ``error`` is ``null`` for normal progress
and a structured object for a fault.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
import json
from typing import Any, Mapping


TASK4_ERROR_CATALOG_VERSION = "task4-errors-v1"


class VehicleCode(IntEnum):
    UAV = 0b0
    USV = 0b1


class TaskCode(IntEnum):
    NOT_STARTED = 0b00
    TASK1 = 0b01
    TASK3 = 0b10
    TASK4 = 0b11


class Task4Step(IntEnum):
    READY = 0b0000
    GO_TO_TARGET = 0b0001
    TARGET_REACHED = 0b0010
    WAIT_RESUME = 0b0011
    RESUMING = 0b0100
    COMPLETE = 0b0101


class CommonError(IntEnum):
    NONE = 0b0000
    TIMEOUT = 0b0001
    INVALID_INPUT = 0b0010
    STALE_DATA = 0b0011
    COMMUNICATION_LOST = 0b0100
    EXECUTION_REJECTED = 0b0101
    DEPENDENCY_UNAVAILABLE = 0b0110
    UNKNOWN_ERROR = 0b0111


class PlannerCommand(IntEnum):
    NONE = 0b000
    ACK = 0b001
    REQUEST_PAUSE = 0b010
    REQUEST_RESUME = 0b011
    REPORT_TASK_COMPLETE = 0b100
    REPORT_TASK_FAILED = 0b101
    REQUEST_SAFE_HOLD = 0b110
    REQUEST_ABORT = 0b111


class RecoveryAction(str, Enum):
    NO_ACTION = "NO_ACTION"
    RETRY_CURRENT_STEP = "RETRY_CURRENT_STEP"
    WAIT_FOR_RECOVERY = "WAIT_FOR_RECOVERY"
    REPLAN_FROM_CURRENT_POSE = "REPLAN_FROM_CURRENT_POSE"
    SAFE_HOLD = "SAFE_HOLD"
    ABORT_TASK = "ABORT_TASK"


@dataclass(frozen=True)
class ErrorDefinition:
    name: str
    retryable: bool
    recovery: RecoveryAction


def _err(name: str, recovery: RecoveryAction, retryable: bool = True) -> ErrorDefinition:
    return ErrorDefinition(name, retryable, recovery)


COMMON_ERROR_DEFINITIONS: Mapping[int, ErrorDefinition] = {
    CommonError.NONE: _err("NONE", RecoveryAction.NO_ACTION, False),
    CommonError.TIMEOUT: _err("TIMEOUT", RecoveryAction.RETRY_CURRENT_STEP),
    CommonError.INVALID_INPUT: _err("INVALID_INPUT", RecoveryAction.WAIT_FOR_RECOVERY),
    CommonError.STALE_DATA: _err("STALE_DATA", RecoveryAction.WAIT_FOR_RECOVERY),
    CommonError.COMMUNICATION_LOST: _err("COMMUNICATION_LOST", RecoveryAction.SAFE_HOLD),
    CommonError.EXECUTION_REJECTED: _err("EXECUTION_REJECTED", RecoveryAction.WAIT_FOR_RECOVERY),
    CommonError.DEPENDENCY_UNAVAILABLE: _err("DEPENDENCY_UNAVAILABLE", RecoveryAction.WAIT_FOR_RECOVERY),
    CommonError.UNKNOWN_ERROR: _err("UNKNOWN_ERROR", RecoveryAction.SAFE_HOLD, False),
}


# Keep all eight step-specific slots assigned.  This prevents a future error
# from changing the meaning of an already deployed 14-bit status word.
TASK4_ERROR_DEFINITIONS: Mapping[Task4Step, Mapping[int, ErrorDefinition]] = {
    Task4Step.READY: {
        0b1000: _err("TASK4_TARGET_INVALID", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1001: _err("TASK4_CHECKPOINT_INVALID", RecoveryAction.ABORT_TASK, False),
        0b1010: _err("TASK4_GPS_REFERENCE_UNAVAILABLE", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1011: _err("TASK4_CURRENT_POSE_UNAVAILABLE", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1100: _err("TASK4_COMMAND_SEQUENCE_INVALID", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1101: _err("TASK4_CONFIG_INVALID", RecoveryAction.ABORT_TASK, False),
        0b1110: _err("TASK4_START_CONDITION_UNMET", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1111: _err("TASK4_COMMAND_DUPLICATE", RecoveryAction.WAIT_FOR_RECOVERY),
    },
    Task4Step.GO_TO_TARGET: {
        0b1000: _err("TASK4_NAVIGATION_FAILED", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1001: _err("TASK4_TARGET_UNREACHABLE", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1010: _err("TASK4_CURRENT_POSE_STALE", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1011: _err("TASK4_SETPOINT_REJECTED", RecoveryAction.SAFE_HOLD),
        0b1100: _err("TASK4_CONTROLLER_FEEDBACK_LOST", RecoveryAction.SAFE_HOLD),
        0b1101: _err("TASK4_TRANSIT_TIMEOUT", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1110: _err("TASK4_SAFETY_DISTANCE_EXCEEDED", RecoveryAction.SAFE_HOLD, False),
        0b1111: _err("TASK4_TARGET_FRAME_INVALID", RecoveryAction.ABORT_TASK, False),
    },
    Task4Step.TARGET_REACHED: {
        0b1000: _err("TASK4_REACHED_REPORT_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1001: _err("TASK4_ARRIVAL_NOT_CONFIRMED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1010: _err("TASK4_TARGET_POSE_INVALID", RecoveryAction.SAFE_HOLD, False),
        0b1011: _err("TASK4_PROGRESS_PUBLISH_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1100: _err("TASK4_PLANNER_ACK_TIMEOUT", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1101: _err("TASK4_POSITION_SOURCE_LOST", RecoveryAction.SAFE_HOLD),
        0b1110: _err("TASK4_CHECKPOINT_REPORT_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1111: _err("TASK4_READY_STATE_CONFLICT", RecoveryAction.SAFE_HOLD, False),
    },
    Task4Step.WAIT_RESUME: {
        0b1000: _err("TASK4_RESUME_TIMEOUT", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1001: _err("TASK4_RESUME_COMMAND_INVALID", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1010: _err("TASK4_CHECKPOINT_MISMATCH", RecoveryAction.ABORT_TASK, False),
        0b1011: _err("TASK4_RESUME_TASK_INVALID", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1100: _err("TASK4_RESUME_PROGRESS_INVALID", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1101: _err("TASK4_DUPLICATE_RESUME", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1110: _err("TASK4_RESUME_REJECTED", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1111: _err("TASK4_MAIN_PLANNER_UNAVAILABLE", RecoveryAction.SAFE_HOLD),
    },
    Task4Step.RESUMING: {
        0b1000: _err("TASK4_CHECKPOINT_INVALID", RecoveryAction.ABORT_TASK, False),
        0b1001: _err("TASK4_RESUME_REJECTED", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1010: _err("TASK4_CONTINUATION_UNAVAILABLE", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1011: _err("TASK4_RESUME_TASK_UNSUPPORTED", RecoveryAction.ABORT_TASK, False),
        0b1100: _err("TASK4_RESUME_CONFIRMATION_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1101: _err("TASK4_RESUME_STATE_CONFLICT", RecoveryAction.SAFE_HOLD, False),
        0b1110: _err("TASK4_RESUME_REPORT_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1111: _err("TASK4_RESUME_RETRY_EXHAUSTED", RecoveryAction.ABORT_TASK, False),
    },
    Task4Step.COMPLETE: {
        0b1000: _err("TASK4_COMPLETE_REPORT_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1001: _err("TASK4_COMPLETION_REJECTED", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1010: _err("TASK4_FINAL_STATE_INCONSISTENT", RecoveryAction.SAFE_HOLD, False),
        0b1011: _err("TASK4_CLEANUP_FAILED", RecoveryAction.SAFE_HOLD, False),
        0b1100: _err("TASK4_DUPLICATE_COMPLETION", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1101: _err("TASK4_RESULT_INCOMPLETE", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1110: _err("TASK4_RESULT_STALE", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1111: _err("TASK4_UNKNOWN_COMPLETION_FAILURE", RecoveryAction.SAFE_HOLD, False),
    },
}


_VEHICLE_SHIFT = 13
_TASK_SHIFT = 11
_STEP_SHIFT = 7
_ERROR_SHIFT = 3
_WIRE_MASK = (1 << 14) - 1


def error_definition(step: int, error: int) -> ErrorDefinition:
    if error < 0b1000:
        return COMMON_ERROR_DEFINITIONS[CommonError(error)]
    return TASK4_ERROR_DEFINITIONS[Task4Step(step)][error]


@dataclass(frozen=True)
class Task4StatusCode:
    vehicle: VehicleCode = VehicleCode.USV
    task: TaskCode = TaskCode.TASK4
    step: int = Task4Step.READY
    error: int = CommonError.NONE
    command: PlannerCommand = PlannerCommand.NONE

    def __post_init__(self) -> None:
        object.__setattr__(self, "vehicle", VehicleCode(self.vehicle))
        object.__setattr__(self, "task", TaskCode(self.task))
        object.__setattr__(self, "command", PlannerCommand(self.command))
        for name, value, maximum in (("step", self.step, 0b1111), ("error", self.error, 0b1111)):
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
                raise ValueError(f"{name} must fit in 4 bits")
        if self.task is not TaskCode.TASK4:
            raise ValueError("Task4StatusCode task must be TASK4")
        try:
            Task4Step(self.step)
        except ValueError as exc:
            raise ValueError("unknown Task4 progress code") from exc
        if self.error >= 0b1000:
            error_definition(self.step, self.error)

    def pack(self) -> int:
        return (
            (int(self.vehicle) << _VEHICLE_SHIFT)
            | (int(self.task) << _TASK_SHIFT)
            | (self.step << _STEP_SHIFT)
            | (self.error << _ERROR_SHIFT)
            | int(self.command)
        )

    def bit_string(self) -> str:
        return f"{int(self.vehicle):01b} {int(self.task):02b} {self.step:04b} {self.error:04b} {int(self.command):03b}"

    def describe(self) -> dict[str, Any]:
        definition = error_definition(self.step, self.error)
        return {
            "packed": self.pack(),
            "bits": self.bit_string(),
            "error_catalog_version": TASK4_ERROR_CATALOG_VERSION,
            "vehicle": self.vehicle.name,
            "task": self.task.name,
            "step": Task4Step(self.step).name,
            "error": definition.name,
            "error_retryable": definition.retryable,
            "recovery_action": definition.recovery.value,
            "command": self.command.name,
        }

    @classmethod
    def unpack(cls, packed: int) -> "Task4StatusCode":
        if isinstance(packed, bool) or not isinstance(packed, int):
            raise TypeError("packed status must be an integer")
        if not 0 <= packed <= _WIRE_MASK:
            raise ValueError("packed status must fit in 14 bits")
        return cls(
            vehicle=(packed >> _VEHICLE_SHIFT) & 0b1,
            task=(packed >> _TASK_SHIFT) & 0b11,
            step=(packed >> _STEP_SHIFT) & 0b1111,
            error=(packed >> _ERROR_SHIFT) & 0b1111,
            command=packed & 0b111,
        )


def _error_payload(status: Task4StatusCode, *, detail: str, source: str, detail_code: str, retry_count: int) -> dict[str, Any] | None:
    definition = error_definition(status.step, status.error)
    if status.error == CommonError.NONE:
        return None
    return {
        "code": status.error,
        "name": definition.name,
        "packed": status.pack(),
        "bits": status.bit_string(),
        "catalog_version": TASK4_ERROR_CATALOG_VERSION,
        "retryable": definition.retryable,
        "recovery_action": definition.recovery.value,
        "source": source,
        "detail_code": detail_code,
        "detail": detail,
        "retry_count": retry_count,
    }


def encode_progress(
    progress: str,
    *,
    command_seq: int | None = None,
    step: int | Task4Step | None = None,
    error: int | CommonError = CommonError.NONE,
    detail: str = "",
    fault_source: str = "",
    detail_code: str = "",
    retry_count: int = 0,
) -> dict[str, Any]:
    """Build the JSON progress envelope used by ``/twave/progress``."""

    if not isinstance(progress, str) or not progress:
        raise ValueError("progress must be a non-empty string")
    if command_seq is not None and (isinstance(command_seq, bool) or not isinstance(command_seq, int) or command_seq <= 0):
        raise ValueError("command_seq must be a positive integer or null")
    if not all(isinstance(value, str) for value in (detail, fault_source, detail_code)):
        raise TypeError("fault fields must be strings")
    if isinstance(retry_count, bool) or not isinstance(retry_count, int) or retry_count < 0:
        raise ValueError("retry_count must be a non-negative integer")
    normalized_step = int(step if step is not None else _step_from_progress(progress))
    status = Task4StatusCode(step=normalized_step, error=int(error))
    if status.error == CommonError.UNKNOWN_ERROR and not detail.strip():
        raise ValueError("UNKNOWN_ERROR requires non-empty detail")
    return {
        "vehicle": "usv",
        "task": "TASK4",
        "progress": progress,
        "error": _error_payload(
            status, detail=detail, source=fault_source,
            detail_code=detail_code, retry_count=retry_count,
        ),
        "command": None,
        "command_seq": command_seq,
        "status_code": status.pack(),
        "status_bits": status.bit_string(),
    }


def _step_from_progress(progress: str) -> Task4Step:
    return {
        "TASK4_ACK": Task4Step.READY,
        "TASK4_TRANSIT": Task4Step.GO_TO_TARGET,
        "TASK4_READY": Task4Step.TARGET_REACHED,
        "TASK4_RESUMED": Task4Step.RESUMING,
        "TASK4_COMPLETE": Task4Step.COMPLETE,
    }.get(progress, Task4Step.READY)


def encode_json_progress(*args: Any, **kwargs: Any) -> str:
    """Serialize :func:`encode_progress` for a std_msgs/String topic."""

    return json.dumps(encode_progress(*args, **kwargs), ensure_ascii=False, separators=(",", ":"))


__all__ = [
    "CommonError",
    "ErrorDefinition",
    "PlannerCommand",
    "RecoveryAction",
    "TASK4_ERROR_CATALOG_VERSION",
    "TASK4_ERROR_DEFINITIONS",
    "Task4StatusCode",
    "Task4Step",
    "VehicleCode",
    "encode_json_progress",
    "encode_progress",
    "error_definition",
]
