"""Small geometry contracts owned by the standalone Task 4 planner."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real


def _finite(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class NEDPoint:
    north: float
    east: float
    down: float

    def __post_init__(self) -> None:
        if not all(_finite(value) for value in (self.north, self.east, self.down)):
            raise ValueError("NED point coordinates must be finite")


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if not all(_finite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("pose coordinates and yaw must be finite")


__all__ = ["NEDPoint", "Pose2D"]
