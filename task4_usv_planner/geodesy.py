"""Small-area GPS to local NED conversion for the Task 4 Core flow."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .types import NEDPoint


@dataclass(frozen=True)
class GpsReference:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float = 0.0

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (
            self.latitude_deg, self.longitude_deg, self.altitude_m,
        )):
            raise ValueError("GPS reference values must be finite")
        if not -90.0 <= self.latitude_deg <= 90.0:
            raise ValueError("latitude must be between -90 and 90 degrees")
        if not -180.0 <= self.longitude_deg <= 180.0:
            raise ValueError("longitude must be between -180 and 180 degrees")

    def to_ned(
        self,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float | None = None,
    ) -> NEDPoint:
        """Convert a nearby WGS84 point to metres relative to this reference."""

        if altitude_m is None:
            altitude_m = self.altitude_m
        if not all(math.isfinite(value) for value in (
            latitude_deg, longitude_deg, altitude_m,
        )):
            raise ValueError("GPS target values must be finite")
        if not -90.0 <= latitude_deg <= 90.0:
            raise ValueError("latitude must be between -90 and 90 degrees")
        if not -180.0 <= longitude_deg <= 180.0:
            raise ValueError("longitude must be between -180 and 180 degrees")

        earth_radius_m = 6_378_137.0
        reference_lat_rad = math.radians(self.latitude_deg)
        north_m = math.radians(latitude_deg - self.latitude_deg) * earth_radius_m
        east_m = (
            math.radians(longitude_deg - self.longitude_deg)
            * earth_radius_m
            * math.cos(reference_lat_rad)
        )
        down_m = -(altitude_m - self.altitude_m)
        return NEDPoint(north_m, east_m, down_m)


__all__ = ["GpsReference"]
