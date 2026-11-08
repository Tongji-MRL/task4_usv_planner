"""Small JSON status table shared with the Task 4 main state machine.

The main FSM does not decode a packed bit word.  It only routes a JSON
progress envelope and checks three fields::

    {"task": "TASK4", "progress": "TASK4_ACK", "error": null}

Keep the wire contract deliberately small.  Detailed diagnostic information
is allowed inside ``error`` but must not be required to understand normal
progress.  This module intentionally does not create the old Task 3-style
14-bit status word.
"""

from __future__ import annotations

from enum import Enum
import json
from typing import Any


TASK4_ERROR_CATALOG_VERSION = "task4-errors-v2"


class Task4Step(str, Enum):
    """Human-readable sub-state labels, not a wire bit-field."""

    READY = "READY"
    GO_TO_TARGET = "GO_TO_TARGET"
    TARGET_REACHED = "TARGET_REACHED"
    WAIT_RESUME = "WAIT_RESUME"
    RESUMING = "RESUMING"
    COMPLETE = "COMPLETE"


class CommonError(str, Enum):
    """Flat error names understood by operators and the main FSM."""

    NONE = "NONE"
    INVALID_INPUT = "INVALID_INPUT"
    TARGET_UNREACHABLE = "TARGET_UNREACHABLE"
    CHECKPOINT_INVALID = "CHECKPOINT_INVALID"
    GPS_REFERENCE_UNAVAILABLE = "GPS_REFERENCE_UNAVAILABLE"
    CURRENT_POSE_UNAVAILABLE = "CURRENT_POSE_UNAVAILABLE"
    COMMUNICATION_LOST = "COMMUNICATION_LOST"
    CONTROLLER_FEEDBACK_LOST = "CONTROLLER_FEEDBACK_LOST"
    RESUME_REJECTED = "RESUME_REJECTED"
    UNKNOWN_ERROR = "UNKNOWN_ERROR"


# The code is a single stable name. There is no step/error Cartesian product.
TASK4_ERROR_DEFINITIONS: dict[str, dict[str, Any]] = {
    CommonError.INVALID_INPUT.value: {
        "retryable": False,
        "recovery_action": "WAIT_FOR_RECOVERY",
    },
    CommonError.TARGET_UNREACHABLE.value: {
        "retryable": False,
        "recovery_action": "ABORT_TASK",
    },
    CommonError.CHECKPOINT_INVALID.value: {
        "retryable": False,
        "recovery_action": "ABORT_TASK",
    },
    CommonError.GPS_REFERENCE_UNAVAILABLE.value: {
        "retryable": True,
        "recovery_action": "WAIT_FOR_RECOVERY",
    },
    CommonError.CURRENT_POSE_UNAVAILABLE.value: {
        "retryable": True,
        "recovery_action": "WAIT_FOR_RECOVERY",
    },
    CommonError.COMMUNICATION_LOST.value: {
        "retryable": True,
        "recovery_action": "SAFE_HOLD",
    },
    CommonError.CONTROLLER_FEEDBACK_LOST.value: {
        "retryable": True,
        "recovery_action": "SAFE_HOLD",
    },
    CommonError.RESUME_REJECTED.value: {
        "retryable": False,
        "recovery_action": "WAIT_FOR_RECOVERY",
    },
    CommonError.UNKNOWN_ERROR.value: {
        "retryable": False,
        "recovery_action": "SAFE_HOLD",
    },
}


def _normalise_error(error: Any) -> str | None:
    if error is None:
        return None
    if isinstance(error, CommonError):
        value = error.value
    elif isinstance(error, str):
        value = error.upper()
    else:
        raise TypeError("error must be a CommonError, string, or None")
    if value == CommonError.NONE.value:
        return None
    if value not in TASK4_ERROR_DEFINITIONS:
        raise ValueError(f"unknown Task4 error: {value}")
    return value


def encode_progress(
    progress: str,
    *,
    command_seq: int | None = None,
    step: str | Task4Step | None = None,
    error: str | CommonError | None = None,
    detail: str = "",
    fault_source: str = "",
    detail_code: str = "",
    retry_count: int = 0,
) -> dict[str, Any]:
    """Build the minimal JSON envelope consumed by the main FSM."""

    if not isinstance(progress, str) or not progress:
        raise ValueError("progress must be a non-empty string")
    if command_seq is not None and (
        isinstance(command_seq, bool) or not isinstance(command_seq, int) or command_seq <= 0
    ):
        raise ValueError("command_seq must be a positive integer or null")
    if not all(isinstance(value, str) for value in (detail, fault_source, detail_code)):
        raise TypeError("fault fields must be strings")
    if isinstance(retry_count, bool) or not isinstance(retry_count, int) or retry_count < 0:
        raise ValueError("retry_count must be a non-negative integer")

    error_code = _normalise_error(error)
    error_payload: dict[str, Any] | None = None
    if error_code is not None:
        definition = TASK4_ERROR_DEFINITIONS[error_code]
        error_payload = {
            "code": error_code,
            "detail": detail,
            "retryable": definition["retryable"],
            "recovery_action": definition["recovery_action"],
        }
        if step is not None:
            error_payload["step"] = str(step.value if isinstance(step, Task4Step) else step)
        if fault_source:
            error_payload["source"] = fault_source
        if detail_code:
            error_payload["detail_code"] = detail_code
        if retry_count:
            error_payload["retry_count"] = retry_count

    payload: dict[str, Any] = {
        "vehicle": "usv",
        "task": "TASK4",
        "progress": progress,
        "error": error_payload,
        "command": None,
    }
    if command_seq is not None:
        payload["command_seq"] = command_seq
    return payload


def encode_json_progress(*args: Any, **kwargs: Any) -> str:
    """Serialize :func:`encode_progress` for a ``std_msgs/String`` topic."""

    return json.dumps(encode_progress(*args, **kwargs), ensure_ascii=False, separators=(",", ":"))


__all__ = [
    "CommonError",
    "TASK4_ERROR_CATALOG_VERSION",
    "TASK4_ERROR_DEFINITIONS",
    "Task4Step",
    "encode_json_progress",
    "encode_progress",
]
