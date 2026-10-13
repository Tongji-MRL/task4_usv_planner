"""USV planning logic for RobotX 2026 Task 4 Disruptive incidents.

This module is deliberately independent of ROS 2, RoboCommand, protobufs and
vehicle I/O.  It turns a moving surface-object report into a conservative
geometric decision for an existing USV mission path.

The official requirement is a minimum 10 m separation from the current and
projected motion of an affected moving surface object.  A planner should not
interpret a successful geometric result as proof that the real vessel can
track it; the caller still owns control, health and emergency-stop handling.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Optional, Sequence, Tuple

from .types import NEDPoint, Pose2D


_EPSILON = 1e-9


def _finite(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _point_xy(point: NEDPoint) -> Tuple[float, float]:
    return point.north, point.east


def _distance(first: Tuple[float, float], second: Tuple[float, float]) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def _point_segment_distance(
    point: Tuple[float, float],
    start: Tuple[float, float],
    end: Tuple[float, float],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= _EPSILON:
        return _distance(point, start)
    fraction = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / length_squared
    fraction = min(1.0, max(0.0, fraction))
    nearest = (start[0] + fraction * dx, start[1] + fraction * dy)
    return _distance(point, nearest)


def _orientation(
    first: Tuple[float, float],
    second: Tuple[float, float],
    third: Tuple[float, float],
) -> float:
    return (
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )


def _on_segment(
    first: Tuple[float, float],
    second: Tuple[float, float],
    point: Tuple[float, float],
) -> bool:
    return (
        min(first[0], second[0]) - _EPSILON <= point[0] <= max(first[0], second[0]) + _EPSILON
        and min(first[1], second[1]) - _EPSILON <= point[1] <= max(first[1], second[1]) + _EPSILON
        and abs(_orientation(first, second, point)) <= _EPSILON
    )


def _segments_intersect(
    first_start: Tuple[float, float],
    first_end: Tuple[float, float],
    second_start: Tuple[float, float],
    second_end: Tuple[float, float],
) -> bool:
    orientations = (
        _orientation(first_start, first_end, second_start),
        _orientation(first_start, first_end, second_end),
        _orientation(second_start, second_end, first_start),
        _orientation(second_start, second_end, first_end),
    )
    if (
        ((orientations[0] > _EPSILON and orientations[1] < -_EPSILON)
         or (orientations[0] < -_EPSILON and orientations[1] > _EPSILON))
        and ((orientations[2] > _EPSILON and orientations[3] < -_EPSILON)
             or (orientations[2] < -_EPSILON and orientations[3] > _EPSILON))
    ):
        return True
    return any(
        abs(orientations[index]) <= _EPSILON
        and _on_segment(
            first_start if index < 2 else second_start,
            first_end if index < 2 else second_end,
            (second_start, second_end, first_start, first_end)[index],
        )
        for index in range(4)
    )


def _segment_distance(
    first_start: Tuple[float, float],
    first_end: Tuple[float, float],
    second_start: Tuple[float, float],
    second_end: Tuple[float, float],
) -> float:
    if _segments_intersect(first_start, first_end, second_start, second_end):
        return 0.0
    return min(
        _point_segment_distance(first_start, second_start, second_end),
        _point_segment_distance(first_end, second_start, second_end),
        _point_segment_distance(second_start, first_start, first_end),
        _point_segment_distance(second_end, first_start, first_end),
    )


def _pose_point(pose: Pose2D) -> Tuple[float, float]:
    return pose.x, pose.y


def _pose_from_point(point: Tuple[float, float], yaw: float) -> Pose2D:
    return Pose2D(point[0], point[1], yaw)


class Task4PlanStatus(str, Enum):
    """Geometric action selected for the USV."""

    PROCEED = "PROCEED"
    DETOUR = "DETOUR"
    HOLD = "HOLD"
    EMERGENCY_ESCAPE = "EMERGENCY_ESCAPE"
    ABORT = "ABORT"
    INVALID_INPUT = "INVALID_INPUT"


@dataclass(frozen=True)
class MovingObjectObservation:
    """A decoded moving-surface-object observation in the USV NED frame.

    ``timestamp`` and ``now`` passed to :meth:`Task4DynamicPlanner.plan` must
    use the same monotonic or synchronized time basis.  The official GPS
    message is converted by an outer adapter; this core only consumes local
    metres and radians.
    """

    position_ned: NEDPoint
    heading_rad: float
    speed_mps: float
    timestamp: float

    def __post_init__(self) -> None:
        if not isinstance(self.position_ned, NEDPoint):
            raise TypeError("position_ned must be a NEDPoint")
        if not _finite(self.heading_rad):
            raise ValueError("heading_rad must be finite")
        if not _finite(self.speed_mps) or self.speed_mps < 0:
            raise ValueError("speed_mps must be finite and nonnegative")
        if not _finite(self.timestamp):
            raise ValueError("timestamp must be finite")


@dataclass(frozen=True)
class Task4PlannerConfig:
    """Safety and sampling policy for the pure USV planner.

    ``min_clearance_m`` is kept at or above the handbook's 10 m rule.  The
    vessel footprint and uncertainty terms enlarge the geometric protected
    corridor so the reference point is not treated as a zero-size robot.
    """

    min_clearance_m: float = 10.0
    usv_footprint_radius_m: float = 1.0
    uncertainty_margin_m: float = 1.0
    detour_margin_m: float = 1.0
    prediction_horizon_s: float = 60.0
    prediction_step_s: float = 1.0
    max_observation_age_s: float = 5.0
    nominal_usv_speed_mps: float = 1.0

    def __post_init__(self) -> None:
        positive = (
            "min_clearance_m", "prediction_horizon_s", "prediction_step_s",
            "max_observation_age_s", "nominal_usv_speed_mps",
        )
        for name in positive:
            value = getattr(self, name)
            if not _finite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.min_clearance_m < 10.0:
            raise ValueError("min_clearance_m cannot be below the official 10 m rule")
        for name in ("usv_footprint_radius_m", "uncertainty_margin_m", "detour_margin_m"):
            value = getattr(self, name)
            if not _finite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")

    @property
    def protected_radius_m(self) -> float:
        return (
            self.min_clearance_m
            + self.usv_footprint_radius_m
            + self.uncertainty_margin_m
        )


@dataclass(frozen=True)
class PredictedObjectTrack:
    """Constant-velocity projection represented as a 2-D swept segment."""

    start_ned: NEDPoint
    end_ned: NEDPoint
    horizon_s: float
    protected_radius_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.start_ned, NEDPoint) or not isinstance(self.end_ned, NEDPoint):
            raise TypeError("track endpoints must be NEDPoint values")
        if not _finite(self.horizon_s) or self.horizon_s <= 0:
            raise ValueError("horizon_s must be positive and finite")
        if not _finite(self.protected_radius_m) or self.protected_radius_m <= 0:
            raise ValueError("protected_radius_m must be positive and finite")

    @property
    def start_xy(self) -> Tuple[float, float]:
        return _point_xy(self.start_ned)

    @property
    def end_xy(self) -> Tuple[float, float]:
        return _point_xy(self.end_ned)

    @property
    def travel_length_m(self) -> float:
        return _distance(self.start_xy, self.end_xy)

    def position_at(self, elapsed_s: float) -> NEDPoint:
        if not _finite(elapsed_s):
            raise ValueError("elapsed_s must be finite")
        fraction = min(1.0, max(0.0, elapsed_s / self.horizon_s))
        return NEDPoint(
            self.start_ned.north + fraction * (self.end_ned.north - self.start_ned.north),
            self.start_ned.east + fraction * (self.end_ned.east - self.start_ned.east),
            self.start_ned.down + fraction * (self.end_ned.down - self.start_ned.down),
        )


@dataclass(frozen=True)
class Task4Plan:
    """A geometric Task4 decision and the path it recommends."""

    status: Task4PlanStatus
    waypoints: Tuple[Pose2D, ...] = ()
    reason: str = ""
    protected_radius_m: float = 0.0
    minimum_clearance_m: float = math.inf
    conflict_time_s: Optional[float] = None
    detour_side: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.status, Task4PlanStatus):
            raise TypeError("status must be a Task4PlanStatus")
        if not isinstance(self.waypoints, (tuple, list)):
            raise TypeError("waypoints must be a sequence")
        if not all(isinstance(waypoint, Pose2D) for waypoint in self.waypoints):
            raise TypeError("waypoints must contain Pose2D values")
        object.__setattr__(self, "waypoints", tuple(self.waypoints))
        if not isinstance(self.reason, str) or not self.reason:
            raise ValueError("reason must be nonempty")


class Task4DynamicPlanner:
    """Plan a conservative response to one current moving-object report.

    The planner treats the object's constant-velocity projection as a swept
    capsule.  A route that stays outside that capsule for the configured
    horizon is safe against the projection regardless of exact crossing time.
    This is intentionally conservative and fail-safe: if the simple left/right
    bypass candidates cannot be proven safe, the result is HOLD or ABORT rather
    than an unverified path.
    """

    def __init__(self, config: Task4PlannerConfig = Task4PlannerConfig()) -> None:
        if not isinstance(config, Task4PlannerConfig):
            raise TypeError("config must be a Task4PlannerConfig")
        self.config = config

    def predict_track(
        self,
        observation: MovingObjectObservation,
    ) -> PredictedObjectTrack:
        if not isinstance(observation, MovingObjectObservation):
            raise TypeError("observation must be a MovingObjectObservation")
        direction = (math.cos(observation.heading_rad), math.sin(observation.heading_rad))
        travel = observation.speed_mps * self.config.prediction_horizon_s
        end = NEDPoint(
            observation.position_ned.north + direction[0] * travel,
            observation.position_ned.east + direction[1] * travel,
            observation.position_ned.down,
        )
        return PredictedObjectTrack(
            start_ned=observation.position_ned,
            end_ned=end,
            horizon_s=self.config.prediction_horizon_s,
            protected_radius_m=self.config.protected_radius_m,
        )

    def plan(
        self,
        current_pose: Pose2D,
        mission_waypoints: Sequence[Pose2D],
        observation: MovingObjectObservation,
        *,
        now_s: float,
    ) -> Task4Plan:
        """Plan from the current USV pose toward the existing mission path.

        ``mission_waypoints`` are the original task's remaining goals, not a
        new Task4-specific destination.  The returned path always starts at
        ``current_pose`` when a path is available.
        """

        try:
            self._validate_inputs(current_pose, mission_waypoints, observation, now_s)
        except (TypeError, ValueError) as exc:
            return Task4Plan(Task4PlanStatus.INVALID_INPUT, reason=str(exc))

        age_s = now_s - observation.timestamp
        if age_s < -_EPSILON or age_s > self.config.max_observation_age_s:
            return Task4Plan(
                Task4PlanStatus.INVALID_INPUT,
                reason="moving-object observation is stale or from the future",
            )

        track = self.predict_track(observation)
        original_path = self._clean_path((current_pose, *mission_waypoints))
        minimum_clearance = self._path_clearance(original_path, track)
        conflict_time = self._estimate_conflict_time(original_path, track)
        if minimum_clearance >= track.protected_radius_m - _EPSILON:
            return Task4Plan(
                Task4PlanStatus.PROCEED,
                waypoints=original_path,
                reason="remaining mission path clears the projected moving-object corridor",
                protected_radius_m=track.protected_radius_m,
                minimum_clearance_m=minimum_clearance,
            )

        current_clearance = self._point_clearance(_pose_point(current_pose), track)
        if current_clearance < track.protected_radius_m - _EPSILON:
            escape = self._emergency_escape(current_pose, track)
            return Task4Plan(
                Task4PlanStatus.EMERGENCY_ESCAPE,
                waypoints=escape,
                reason="USV is already inside the protected moving-object corridor",
                protected_radius_m=track.protected_radius_m,
                minimum_clearance_m=current_clearance,
                conflict_time_s=0.0,
            )

        candidates = self._detour_candidates(current_pose, mission_waypoints, track)
        safe_candidates = [
            candidate for candidate in candidates
            if self._path_clearance(candidate[0], track) >= track.protected_radius_m - _EPSILON
        ]
        if safe_candidates:
            selected_path, side, clearance = min(
                safe_candidates,
                key=lambda item: self._path_length(item[0]),
            )
            return Task4Plan(
                Task4PlanStatus.DETOUR,
                waypoints=selected_path,
                reason="remaining mission path intersects the projected corridor; selected a proven bypass",
                protected_radius_m=track.protected_radius_m,
                minimum_clearance_m=clearance,
                conflict_time_s=conflict_time,
                detour_side=side,
            )

        if self._point_clearance(_pose_point(current_pose), track) >= track.protected_radius_m - _EPSILON:
            return Task4Plan(
                Task4PlanStatus.HOLD,
                waypoints=(current_pose,),
                reason="no simple bypass is proven safe; hold at the current safe position",
                protected_radius_m=track.protected_radius_m,
                minimum_clearance_m=current_clearance,
                conflict_time_s=conflict_time,
            )
        return Task4Plan(
            Task4PlanStatus.ABORT,
            reason="no safe straight-line bypass or hold position is available",
            protected_radius_m=track.protected_radius_m,
            minimum_clearance_m=current_clearance,
            conflict_time_s=conflict_time,
        )

    def _validate_inputs(
        self,
        current_pose: Pose2D,
        mission_waypoints: Sequence[Pose2D],
        observation: MovingObjectObservation,
        now_s: float,
    ) -> None:
        if not isinstance(current_pose, Pose2D):
            raise TypeError("current_pose must be a Pose2D")
        if not isinstance(mission_waypoints, (tuple, list)) or not mission_waypoints:
            raise ValueError("mission_waypoints must be a nonempty sequence")
        if not all(isinstance(waypoint, Pose2D) for waypoint in mission_waypoints):
            raise TypeError("mission_waypoints must contain Pose2D values")
        if not _finite(now_s):
            raise ValueError("now_s must be finite")
        if not isinstance(observation, MovingObjectObservation):
            raise TypeError("observation must be a MovingObjectObservation")

    @staticmethod
    def _clean_path(path: Sequence[Pose2D]) -> Tuple[Pose2D, ...]:
        cleaned = []
        for pose in path:
            if not cleaned or _distance(_pose_point(cleaned[-1]), _pose_point(pose)) > _EPSILON:
                cleaned.append(pose)
        return tuple(cleaned)

    @staticmethod
    def _point_clearance(point: Tuple[float, float], track: PredictedObjectTrack) -> float:
        return _point_segment_distance(point, track.start_xy, track.end_xy)

    @classmethod
    def _path_clearance(
        cls,
        path: Sequence[Pose2D],
        track: PredictedObjectTrack,
    ) -> float:
        if not path:
            return math.inf
        if len(path) == 1:
            return cls._point_clearance(_pose_point(path[0]), track)
        return min(
            _segment_distance(
                _pose_point(start), _pose_point(end), track.start_xy, track.end_xy,
            )
            for start, end in zip(path, path[1:])
        )

    @staticmethod
    def _path_length(path: Sequence[Pose2D]) -> float:
        return sum(
            _distance(_pose_point(start), _pose_point(end))
            for start, end in zip(path, path[1:])
        )

    def _estimate_conflict_time(
        self,
        path: Sequence[Pose2D],
        track: PredictedObjectTrack,
    ) -> Optional[float]:
        elapsed = 0.0
        sample_step = self.config.prediction_step_s
        for start, end in zip(path, path[1:]):
            segment_length = _distance(_pose_point(start), _pose_point(end))
            duration = segment_length / self.config.nominal_usv_speed_mps
            sample_count = max(1, int(math.ceil(duration / sample_step)))
            for index in range(sample_count + 1):
                fraction = index / sample_count
                point = (
                    start.x + fraction * (end.x - start.x),
                    start.y + fraction * (end.y - start.y),
                )
                object_point = _point_xy(track.position_at(elapsed + fraction * duration))
                if _distance(point, object_point) < track.protected_radius_m:
                    return min(track.horizon_s, elapsed + fraction * duration)
            elapsed += duration
        return None

    def _detour_candidates(
        self,
        current_pose: Pose2D,
        mission_waypoints: Sequence[Pose2D],
        track: PredictedObjectTrack,
    ) -> Tuple[Tuple[Tuple[Pose2D, ...], str, float], ...]:
        original_path = self._clean_path((current_pose, *mission_waypoints))
        conflict_index = None
        for index, (start, end) in enumerate(zip(original_path, original_path[1:])):
            if _segment_distance(
                _pose_point(start), _pose_point(end), track.start_xy, track.end_xy,
            ) < track.protected_radius_m - _EPSILON:
                conflict_index = index
                break
        if conflict_index is None:
            return ()

        # Preserve all already-safe mission segments.  Only the first
        # conflicting segment is replaced by a visibility-graph detour; the
        # remaining task waypoints are appended unchanged and re-validated.
        segment_start = original_path[conflict_index]
        segment_goal = original_path[conflict_index + 1]
        local_detour = self._visibility_detour(
            _pose_point(segment_start), _pose_point(segment_goal), track,
        )
        if local_detour is None:
            return ()
        local_path, side = local_detour
        combined = self._clean_path((
            *original_path[:conflict_index],
            *tuple(_pose_from_point(point, segment_start.yaw) for point in local_path),
            *original_path[conflict_index + 2:],
        ))
        return ((combined, side, self._path_clearance(combined, track)),)

    def _visibility_detour(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        track: PredictedObjectTrack,
    ) -> Optional[Tuple[Tuple[Tuple[float, float], ...], str]]:
        """Find a short path around the predicted capsule.

        The vertices are placed on a capsule expanded by the configurable
        detour margin.  Edges are accepted only when their true clearance from
        the unexpanded predicted track is at least the protected radius.  This
        avoids the endpoint-cutting error of a simple parallel-offset route.
        """

        vertices = self._capsule_vertices(
            track.start_xy,
            track.end_xy,
            track.protected_radius_m + self.config.detour_margin_m,
        )
        nodes = [start, goal, *vertices]
        adjacency = [[] for _ in nodes]
        for first_index in range(len(nodes)):
            for second_index in range(first_index + 1, len(nodes)):
                clearance = _segment_distance(
                    nodes[first_index], nodes[second_index], track.start_xy, track.end_xy,
                )
                if clearance < track.protected_radius_m - _EPSILON:
                    continue
                cost = _distance(nodes[first_index], nodes[second_index])
                adjacency[first_index].append((second_index, cost))
                adjacency[second_index].append((first_index, cost))

        distances = [math.inf] * len(nodes)
        previous = [None] * len(nodes)
        visited = set()
        distances[0] = 0.0
        while True:
            remaining = (
                (distances[index], index)
                for index in range(len(nodes))
                if index not in visited
            )
            current_distance, current_index = min(remaining, default=(math.inf, None))
            if current_index is None or current_index == 1:
                break
            visited.add(current_index)
            for neighbour, edge_cost in adjacency[current_index]:
                candidate_distance = current_distance + edge_cost
                if candidate_distance < distances[neighbour]:
                    distances[neighbour] = candidate_distance
                    previous[neighbour] = current_index
        if not math.isfinite(distances[1]):
            return None

        indices = []
        cursor = 1
        while cursor is not None:
            indices.append(cursor)
            cursor = previous[cursor]
        indices.reverse()
        path = tuple(nodes[index] for index in indices)
        side = self._classify_detour_side(path, track)
        return path, side

    @staticmethod
    def _capsule_vertices(
        start: Tuple[float, float],
        end: Tuple[float, float],
        radius: float,
        arc_steps: int = 16,
    ) -> Tuple[Tuple[float, float], ...]:
        vector = (end[0] - start[0], end[1] - start[1])
        length = math.hypot(vector[0], vector[1])
        if length <= _EPSILON:
            return tuple(
                (
                    start[0] + radius * math.cos(2 * math.pi * index / (arc_steps * 2)),
                    start[1] + radius * math.sin(2 * math.pi * index / (arc_steps * 2)),
                )
                for index in range(arc_steps * 2)
            )

        angle = math.atan2(vector[1], vector[0])
        normal_angle = angle + math.pi / 2.0
        vertices = [
            (start[0] + radius * math.cos(normal_angle), start[1] + radius * math.sin(normal_angle)),
            (end[0] + radius * math.cos(normal_angle), end[1] + radius * math.sin(normal_angle)),
        ]
        for index in range(1, arc_steps + 1):
            theta = normal_angle - math.pi * index / arc_steps
            vertices.append((
                end[0] + radius * math.cos(theta),
                end[1] + radius * math.sin(theta),
            ))
        for index in range(1, arc_steps + 1):
            theta = normal_angle + math.pi + math.pi * index / arc_steps
            vertices.append((
                start[0] + radius * math.cos(theta),
                start[1] + radius * math.sin(theta),
            ))
        return tuple(vertices)

    @staticmethod
    def _classify_detour_side(
        path: Sequence[Tuple[float, float]],
        track: PredictedObjectTrack,
    ) -> str:
        if len(path) < 3:
            return "UNKNOWN"
        vector = (
            track.end_xy[0] - track.start_xy[0],
            track.end_xy[1] - track.start_xy[1],
        )
        first_detour = path[1]
        relative = (
            first_detour[0] - track.start_xy[0],
            first_detour[1] - track.start_xy[1],
        )
        cross = vector[0] * relative[1] - vector[1] * relative[0]
        if abs(cross) <= _EPSILON:
            return "UNKNOWN"
        return "LEFT" if cross > 0 else "RIGHT"

    def _emergency_escape(
        self,
        current_pose: Pose2D,
        track: PredictedObjectTrack,
    ) -> Tuple[Pose2D, ...]:
        current = _pose_point(current_pose)
        projected = track.start_xy
        vector = (current[0] - projected[0], current[1] - projected[1])
        length = math.hypot(vector[0], vector[1])
        if length <= _EPSILON:
            track_vector = (
                track.end_xy[0] - track.start_xy[0],
                track.end_xy[1] - track.start_xy[1],
            )
            length = math.hypot(track_vector[0], track_vector[1])
            vector = (-track_vector[1], track_vector[0]) if length > _EPSILON else (1.0, 0.0)
            length = math.hypot(vector[0], vector[1])
        unit = (vector[0] / length, vector[1] / length)
        escape_distance = track.protected_radius_m + self.config.detour_margin_m
        escape = (
            projected[0] + unit[0] * escape_distance,
            projected[1] + unit[1] * escape_distance,
        )
        return (current_pose, _pose_from_point(escape, current_pose.yaw))


__all__ = [
    "MovingObjectObservation",
    "PredictedObjectTrack",
    "Task4DynamicPlanner",
    "Task4Plan",
    "Task4PlanStatus",
    "Task4PlannerConfig",
]
