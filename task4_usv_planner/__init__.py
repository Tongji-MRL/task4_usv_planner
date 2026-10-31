"""Small geometry contracts for RobotX 2026 Task 4 Core."""

from .types import NEDPoint, Pose2D
from .geodesy import GpsReference
from .core_state_machine import (
    CoreCheckpoint,
    CorePlannerConfig,
    CoreSnapshot,
    CoreState,
    CoreTask4Begin,
    Task4CoreStateMachine,
)
from .task4_status_codec import (
    CommonError,
    PlannerCommand,
    RecoveryAction,
    Task4StatusCode,
    Task4Step,
    encode_progress,
)
from .task4_communication import (
    AssistanceRequest,
    CommunicationEvent,
    ReadinessConfirm,
    Task4CommunicationError,
    Task4CommunicationSession,
    parse_assistance_request,
    parse_readiness_confirm,
    normalize_ocs_command,
)

__all__ = [
    "CoreCheckpoint",
    "CorePlannerConfig",
    "CoreSnapshot",
    "CoreState",
    "CoreTask4Begin",
    "GpsReference",
    "NEDPoint",
    "Pose2D",
    "Task4CoreStateMachine",
    "CommonError",
    "PlannerCommand",
    "RecoveryAction",
    "Task4StatusCode",
    "Task4Step",
    "encode_progress",
    "AssistanceRequest",
    "CommunicationEvent",
    "ReadinessConfirm",
    "Task4CommunicationError",
    "Task4CommunicationSession",
    "parse_assistance_request",
    "parse_readiness_confirm",
    "normalize_ocs_command",
]
