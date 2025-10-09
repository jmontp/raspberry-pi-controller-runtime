"""Hardware-backed IMU implementations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .base import BaseIMU, IMUSample

try:
    import read_imu  # type: ignore
except ImportError:  # pragma: no cover - optional dependency on Pi
    read_imu = None


@dataclass(slots=True)
class LegacyIMUConfig:
    """Configuration describing how to adapt the legacy IMU driver."""

    joints: tuple[str, ...]
    segments: tuple[str, ...]
    timeout_ms: int = 500
    transform: Callable[[dict[str, Any]], IMUSample] | None = None


class LegacyIMUAdapter(BaseIMU):
    """Wrap the historic `read_imu.IMU` class with the new interface."""

    def __init__(self, config: LegacyIMUConfig):
        """Create the adapter and instantiate the underlying driver.

        Args:
            config: Joint/segment names and optional transform for raw data.

        Raises:
            RuntimeError: When the legacy driver cannot be imported.
        """
        if read_imu is None:
            raise RuntimeError(
                "read_imu.IMU import failed; install hardware dependencies before using"
            )
        self._config = config
        self._driver = read_imu.IMU(timeout=config.timeout_ms)

    def start(self) -> None:
        """Open the hardware connection and begin streaming."""
        self._driver.connect()

    def stop(self) -> None:
        """Close the hardware connection."""
        self._driver.disconnect()

    def read(self) -> IMUSample:
        """Fetch the latest IMU sample."""
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
        """Zero the underlying driver orientation/velocity offsets."""
        self._driver.zero()
