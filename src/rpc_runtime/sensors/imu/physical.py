"""Hardware-backed IMU implementations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .base import BaseIMU, IMUSample

try:
    from read_imu import IMU as LegacyIMU  # type: ignore
except ImportError:  # pragma: no cover - optional dependency on Pi
    LegacyIMU = None


@dataclass(slots=True)
class LegacyIMUConfig:
    joints: tuple[str, ...]
    segments: tuple[str, ...]
    timeout_ms: int = 500
    transform: Callable[[dict[str, Any]], IMUSample] | None = None


class LegacyIMUAdapter(BaseIMU):
    """Wrap the historic `read_imu.IMU` class with the new interface."""

    def __init__(self, config: LegacyIMUConfig):
        if LegacyIMU is None:
            raise RuntimeError(
                "read_imu.IMU import failed; install hardware dependencies before using"
            )
        self._config = config
        self._driver = LegacyIMU(timeout=config.timeout_ms)

    def start(self) -> None:
        self._driver.connect()

    def stop(self) -> None:
        self._driver.disconnect()

    def read(self) -> IMUSample:
        raw = self._driver.read()
        if self._config.transform is not None:
            return self._config.transform(raw)
        # Expect raw dict with joint and segment information similar to jose controller
        joint_angles = tuple(float(raw[f"{name}_angle"]) for name in self._config.joints)
        joint_velocities = tuple(float(raw[f"{name}_velocity"]) for name in self._config.joints)
        segment_angles = tuple(float(raw[f"{name}_angle"]) for name in self._config.segments)
        segment_velocities = tuple(float(raw[f"{name}_velocity"]) for name in self._config.segments)
        timestamp = float(raw.get("timestamp", 0.0))
        return IMUSample(
            timestamp=timestamp,
            joint_angles_rad=joint_angles,
            joint_velocities_rad_s=joint_velocities,
            segment_angles_rad=segment_angles,
            segment_velocities_rad_s=segment_velocities,
        )

    def reset(self) -> None:
        self._driver.zero()
