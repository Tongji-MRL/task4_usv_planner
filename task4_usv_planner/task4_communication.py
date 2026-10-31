"""Transport-neutral Task 4 Core communication adapter.

The official RoboCommand protobuf puts the command sequence on ``RxCommand``
and the report sequence on ``RxReport``.  This module uses the same shape for
the JSON bridge between OCS and the USV node.  Protobuf/MQTT transport remains
an OCS responsibility; the planner only validates and translates the messages
it needs for the Core tier.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional


class Task4CommunicationError(ValueError):
    """A malformed, stale, or incorrectly addressed Task 4 message."""


@dataclass(frozen=True)
class AssistanceRequest:
    """Decoded official ``RxCommand.assistance_request``."""

    command_seq: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: Optional[float]
    vehicle_type: str
    team_id: str = ""
    vehicle_id: str = ""


@dataclass(frozen=True)
class ReadinessConfirm:
    """Decoded official ``RxCommand.readiness_confirm``."""

    report_seq: int
    vehicle_id: str
    command_seq: Optional[int] = None


@dataclass(frozen=True)
class CommunicationEvent:
    """Result of consuming one OCS command."""

    accepted: bool
    reason: str
    assistance: Optional[AssistanceRequest] = None
    readiness_confirm: Optional[ReadinessConfirm] = None


def _body(payload: dict[str, Any]) -> dict[str, Any]:
    value = payload.get("body")
    if not isinstance(value, dict):
        return payload
    for key in ("assistanceRequest", "assistance_request", "readinessConfirm", "readiness_confirm"):
        nested = value.get(key)
        if isinstance(nested, dict):
            return nested
    return value


def _message_kind(payload: dict[str, Any]) -> str:
    body = _body(payload)
    raw = body.get("type", body.get("message_type", body.get("command")))
    if raw is None:
        envelope_body = payload.get("body")
        for key in ("assistance_request", "assistanceRequest", "readiness_confirm", "readinessConfirm"):
            if key in payload or (isinstance(envelope_body, dict) and key in envelope_body):
                raw = key
                break
    if isinstance(raw, dict):
        raw = raw.get("type", raw.get("message_type"))
    return str(raw or "").replace("-", "_").replace(" ", "_").lower()


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise Task4CommunicationError(f"{name} must be a positive integer")
    return value


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Task4CommunicationError(f"{name} must be numeric")
    result = float(value)
    if result != result or result in (float("inf"), float("-inf")):
        raise Task4CommunicationError(f"{name} must be finite")
    return result


def _normalise_vehicle_type(value: Any) -> str:
    if isinstance(value, int) and not isinstance(value, bool):
        return {1: "USV", 2: "UUV", 3: "UAV"}.get(value, "UNKNOWN")
    text = str(value or "").upper()
    return {
        "TYPE_USV": "USV",
        "VEHICLE_TYPE_USV": "USV",
        "TYPE_UUV": "UUV",
        "VEHICLE_TYPE_UUV": "UUV",
        "TYPE_UAV": "UAV",
        "VEHICLE_TYPE_UAV": "UAV",
    }.get(text, text)


def _position(body: dict[str, Any]) -> dict[str, Any]:
    value = body.get("position", body.get("target", {}))
    if not isinstance(value, dict):
        raise Task4CommunicationError("assistance_request.position must be an object")
    return value


def parse_assistance_request(
    payload: dict[str, Any],
    *,
    expected_vehicle_id: str = "",
) -> AssistanceRequest:
    """Decode an official command or the equivalent JSON bridge envelope."""

    if not isinstance(payload, dict):
        raise Task4CommunicationError("command envelope must be an object")
    body = _body(payload)
    kind = _message_kind(payload)
    if kind not in {"assistance_request", "assistancerequest", "task4_begin"}:
        raise Task4CommunicationError(f"unsupported Task 4 command: {kind or 'missing type'}")

    command_seq = payload.get("seq", payload.get("command_seq", body.get("command_seq")))
    command_seq = _positive_int(command_seq, "command sequence")
    vehicle_id = str(
        payload.get("vehicle_id", payload.get("vehicleId", body.get("vehicle_id", body.get("vehicleId", ""))))
    )
    if expected_vehicle_id and vehicle_id and vehicle_id != expected_vehicle_id:
        raise Task4CommunicationError(
            f"command targets vehicle {vehicle_id!r}, expected {expected_vehicle_id!r}"
        )
    vehicle_type = _normalise_vehicle_type(
        body.get(
            "vehicle_type",
            body.get(
                "vehicleType",
                payload.get("target_vehicle", payload.get("vehicle_type", "USV")),
            ),
        )
    )
    position = _position(body)
    latitude = _number(position.get("latitude"), "position.latitude")
    longitude = _number(position.get("longitude"), "position.longitude")
    altitude = position.get("altitude", position.get("altitude_m"))
    return AssistanceRequest(
        command_seq=command_seq,
        latitude_deg=latitude,
        longitude_deg=longitude,
        altitude_m=None if altitude is None else _number(altitude, "position.altitude"),
        vehicle_type=vehicle_type,
        team_id=str(payload.get("team_id", payload.get("teamId", ""))),
        vehicle_id=vehicle_id,
    )


def parse_readiness_confirm(
    payload: dict[str, Any],
    *,
    expected_vehicle_id: str,
) -> ReadinessConfirm:
    """Decode official ``RxCommand.readiness_confirm``."""

    if not isinstance(payload, dict):
        raise Task4CommunicationError("command envelope must be an object")
    body = _body(payload)
    kind = _message_kind(payload)
    if kind not in {"readiness_confirm", "readinessconfirm", "resume"}:
        raise Task4CommunicationError(f"unsupported Task 4 confirmation: {kind or 'missing type'}")
    vehicle_id = str(
        body.get(
            "vehicle_id",
            body.get("vehicleId", payload.get("vehicle_id", payload.get("vehicleId", ""))),
        )
    )
    if vehicle_id != expected_vehicle_id:
        raise Task4CommunicationError(
            f"confirmation targets vehicle {vehicle_id!r}, expected {expected_vehicle_id!r}"
        )
    report_seq = body.get("report_seq", body.get("reportSeq", payload.get("report_seq", payload.get("reportSeq"))))
    report_seq = _positive_int(report_seq, "report sequence")
    command_seq = body.get(
        "command_seq",
        body.get("commandSeq", payload.get("command_seq", payload.get("commandSeq"))),
    )
    if command_seq is not None:
        command_seq = _positive_int(command_seq, "command sequence")
    return ReadinessConfirm(report_seq, vehicle_id, command_seq)


def normalize_ocs_command(payload: dict[str, Any]) -> dict[str, Any]:
    """Translate the team's internal OCS envelope to an RxCommand-shaped dict.

    The OCS remains the only component that talks to RoboCommand.  This small
    adapter lets the ROS node consume the already-routed ``task4_assistance``
    and ``task4_readiness_confirm`` commands without importing MQTT or protobuf
    code into the planner package.
    """

    if not isinstance(payload, dict) or payload.get("kind") != "command":
        return payload
    inner = payload.get("payload")
    if not isinstance(inner, dict):
        raise Task4CommunicationError("OCS command payload must be an object")
    command = str(inner.get("command", "")).lower()
    parameters = inner.get("parameters")
    if not isinstance(parameters, dict):
        parameters = {}
    command_seq = inner.get("command_seq", parameters.get("official_command_seq"))
    vehicle_id = str(payload.get("vehicle_id", parameters.get("vehicle_id", "")))
    if command == "task4_assistance":
        return {
            "team_id": payload.get("team_id", ""),
            "seq": command_seq,
            "vehicle_id": vehicle_id,
            "body": {
                "type": "assistance_request",
                "position": parameters.get("position", {}),
                "vehicle_type": parameters.get("vehicle_type", "USV"),
            },
        }
    if command == "task4_readiness_confirm":
        return {
            "team_id": payload.get("team_id", ""),
            "seq": inner.get("command_seq", 1),
            "vehicle_id": vehicle_id,
            "body": {
                "type": "readiness_confirm",
                "report_seq": parameters.get("report_seq"),
                "vehicle_id": parameters.get("vehicle_id", vehicle_id),
            },
        }
    raise Task4CommunicationError(f"unsupported OCS command: {command or 'missing command'}")


class Task4CommunicationSession:
    """Validate sequencing and build official-shaped OCS report envelopes."""

    def __init__(self, *, vehicle_id: str = "USV1", team_id: str = "") -> None:
        if not vehicle_id:
            raise ValueError("vehicle_id must not be empty")
        self.vehicle_id = vehicle_id
        self.team_id = team_id
        self._next_report_seq = 1
        self._last_command_seq = 0
        self._active_command_seq: Optional[int] = None
        self._active_report_seq: Optional[int] = None

    @property
    def active_command_seq(self) -> Optional[int]:
        return self._active_command_seq

    @property
    def active_report_seq(self) -> Optional[int]:
        return self._active_report_seq

    def accept_assistance(self, request: AssistanceRequest) -> CommunicationEvent:
        if request.vehicle_type != "USV":
            return CommunicationEvent(False, "assistance request is not addressed to a USV", assistance=request)
        if request.command_seq <= self._last_command_seq:
            return CommunicationEvent(False, "stale or duplicate assistance request", assistance=request)
        self._last_command_seq = request.command_seq
        self._active_command_seq = request.command_seq
        self._active_report_seq = None
        return CommunicationEvent(True, "assistance request accepted", assistance=request)

    def accept_readiness_confirm(self, confirm: ReadinessConfirm) -> CommunicationEvent:
        if confirm.report_seq != self._active_report_seq:
            return CommunicationEvent(False, "readiness confirmation does not match the active report", readiness_confirm=confirm)
        if confirm.command_seq is not None and confirm.command_seq != self._active_command_seq:
            return CommunicationEvent(False, "readiness confirmation does not match the active command", readiness_confirm=confirm)
        self._active_command_seq = None
        self._active_report_seq = None
        return CommunicationEvent(True, "readiness confirmation accepted", readiness_confirm=confirm)

    def incident_ack(self, command_seq: int) -> dict[str, Any]:
        return self._report(
            "incident_ack",
            {"command_seq": _positive_int(command_seq, "command sequence")},
        )

    def readiness_report(self, command_seq: int) -> dict[str, Any]:
        command_seq = _positive_int(command_seq, "command sequence")
        if command_seq != self._active_command_seq:
            raise Task4CommunicationError("cannot report readiness for an inactive command")
        self._active_report_seq = self._next_report_seq
        self._next_report_seq += 1
        return self._report(
            "readiness",
            {"command_seq": command_seq},
            seq=self._active_report_seq,
        )

    def _report(self, body_type: str, body: dict[str, Any], *, seq: Optional[int] = None) -> dict[str, Any]:
        report_seq = self._next_report_seq if seq is None else seq
        if seq is None or report_seq >= self._next_report_seq:
            self._next_report_seq += 1
        return {
            "team_id": self.team_id,
            "vehicle_id": self.vehicle_id,
            "seq": report_seq,
            "body": {"type": body_type, **body},
        }


__all__ = [
    "AssistanceRequest",
    "CommunicationEvent",
    "ReadinessConfirm",
    "Task4CommunicationError",
    "Task4CommunicationSession",
    "parse_assistance_request",
    "parse_readiness_confirm",
    "normalize_ocs_command",
]
