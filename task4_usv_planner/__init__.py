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
]
