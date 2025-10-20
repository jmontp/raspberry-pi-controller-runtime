"""HBK MicroStrain 3DM-GX5-AHRS adapter implemented with MSCL bindings."""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass, field, replace
from typing import Any, Dict, Tuple

import numpy as np

from .base import LOGGER, BaseIMU, BaseIMUConfig, IMUSample

if "/usr/share/python3-mscl/" not in sys.path:
    sys.path.append("/usr/share/python3-mscl/")

try:  # pragma: no cover - hardware dependency
    import mscl
except ImportError:  # pragma: no cover - optional dependency for development
    mscl = None


_DEFAULT_SEGMENTS: Tuple[str, ...] = (
    "thigh_r",
    "shank_r",
    "foot_r",
    "thigh_l",
    "shank_l",
    "foot_l",
)
_DEFAULT_JOINTS: Tuple[str, ...] = (
    "knee_r",
    "ankle_r",
    "foot_r",
    "knee_l",
    "ankle_l",
    "foot_l",
)


def _default_port_map() -> Dict[str, str]:
    """Placeholder port map – override in production deployments."""
    return {
        "thigh_r": "/dev/ttyIMU_thigh_r",
        "shank_r": "/dev/ttyIMU_shank_r",
        "foot_r": "/dev/ttyIMU_foot_r",
        "thigh_l": "/dev/ttyIMU_thigh_l",
        "shank_l": "/dev/ttyIMU_shank_l",
        "foot_l": "/dev/ttyIMU_foot_l",
    }


@dataclass(slots=True)
class Microstrain3DMGX5Config(BaseIMUConfig):
    """Configuration describing the HBK MicroStrain 3DM-GX5-AHRS."""

    port_map: Dict[str, str] = field(default_factory=_default_port_map)
    timeout_ms: int = 0
    calibration_samples: int = 500
    calibration_interval_s: float = 0.002


class _MicrostrainNode:
    """Minimal wrapper around an MSCL inertial node."""

    def __init__(self, port: str, timeout_ms: int) -> None:
        if mscl is None:  # pragma: no cover - checked earlier
            raise RuntimeError("mscl module not available; install MicroStrain MSCL")
        real_port = os.path.realpath(port)
        self._connection = mscl.Connection.Serial(real_port, 921600)
        self._node = mscl.InertialNode(self._connection)
        self._timeout = timeout_ms

    def read(self) -> dict[str, float] | None:
        packets = self._node.getDataPackets(self._timeout)
        if not packets:
            return None
        latest = packets[-1]
        data: dict[str, float] = {}
        for point in latest.data():
            try:
                data[point.channelName()] = point.as_float()
            except Exception:  # pragma: no cover - ignore malformed points
                continue
        return data if data else None

    def close(self) -> None:
        try:
            self._connection.disconnect()
        except Exception:  # pragma: no cover - best-effort cleanup
            pass


class Microstrain3DMGX5IMU(BaseIMU):
    """Adapter for HBK MicroStrain 3DM-GX5-AHRS using MSCL."""

    JOINT_NAMES = _DEFAULT_JOINTS
    SEGMENT_NAMES = _DEFAULT_SEGMENTS

    def __init__(
        self,
        config: Microstrain3DMGX5Config | None = None,
        **config_overrides: Any,
    ) -> None:
        """Initialise the MicroStrain adaptor and cache calibration metadata."""
        if config is None:
            cfg = (
                Microstrain3DMGX5Config(**config_overrides)
                if config_overrides
                else Microstrain3DMGX5Config()
            )
        else:
            cfg = replace(config, **config_overrides) if config_overrides else config
        super().__init__(cfg)
        self._timeout_ms = cfg.timeout_ms
        self._calibration_samples = cfg.calibration_samples
        self._calibration_interval = cfg.calibration_interval_s
        self._nodes: dict[str, _MicrostrainNode] = {}
        self._segment_indices: dict[str, int] = {}
        segment_len = len(self.segment_names)
        self._zero_offsets = np.zeros(2 * segment_len)
        self._last_segments = np.zeros(2 * segment_len)
        self._last_angles = np.zeros(segment_len)
        self._last_velocities = np.zeros(segment_len)

    # ---------------------------------------------------------------------
    # BaseIMU interface
    # ---------------------------------------------------------------------

    def start(self) -> None:
        """Open connections to each segment and perform calibration."""
        if mscl is None:  # pragma: no cover - checked during runtime
            raise RuntimeError(
                "mscl module not available; install MicroStrain MSCL before using the IMU"
            )
        if self._nodes:
            return
        self._segment_indices = {name: idx for idx, name in enumerate(self.segment_names)}
        for segment in self.segment_names:
            port = self.port_map.get(segment)
            if port is None:
                raise ValueError(f"No port mapping provided for segment '{segment}'")
            self._nodes[segment] = _MicrostrainNode(port, self._timeout_ms)
        self._zero_offsets = self._calibrate()

    def stop(self) -> None:
        """Close all active connections."""
        for node in self._nodes.values():
            node.close()
        self._nodes.clear()

    def read(self) -> IMUSample:
        """Fetch the latest IMU sample."""
        if not self._nodes:
            raise RuntimeError("IMU not started; call start() before read().")
        segments, fresh = self._read_segments()
        if segments is None:
            segments = self._last_segments
            if segments is None or not len(segments):
                return self._handle_sample(None, fresh=False)
        offset = segments - self._zero_offsets
        n_segments = len(self.segment_names)
        segment_angles = offset[:n_segments]
        segment_velocities = offset[n_segments:]
        joint_angles, joint_velocities = self._compute_joint_values(
            segment_angles, segment_velocities
        )
        timestamp = time.monotonic()
        sample = IMUSample(
            timestamp=timestamp,
            joint_angles_rad=tuple(joint_angles),
            joint_velocities_rad_s=tuple(joint_velocities),
            segment_angles_rad=tuple(segment_angles),
            segment_velocities_rad_s=tuple(segment_velocities),
        )
        return self._handle_sample(sample, fresh=fresh)

    def reset(self) -> None:
        """Re-calibrate zero offsets for the connected IMUs."""
        if not self._nodes:
            self._zero_offsets[:] = 0.0
            return
        self._zero_offsets = self._calibrate()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _calibrate(self) -> np.ndarray:
        """Collect samples and compute average offsets."""
        samples: list[np.ndarray] = []
        for _ in range(self._calibration_samples):
            measurement, fresh = self._read_segments()
            if measurement is not None and fresh:
                samples.append(measurement)
            time.sleep(self._calibration_interval)
        if not samples:
            return np.zeros_like(self._last_segments)
        stacked = np.stack(samples, axis=0)
        return stacked.mean(axis=0)

    def _read_segments(self) -> tuple[np.ndarray | None, bool]:
        """Read roll/gyro data for all segments; returns radians and freshness flag."""
        new_sample = False
        angles = np.array(self._last_angles, copy=True)
        velocities = np.array(self._last_velocities, copy=True)
        for idx, segment in enumerate(self.segment_names):
            node = self._nodes.get(segment)
            if node is None:
                continue
            try:
                packet = node.read()
            except Exception as exc:  # pragma: no cover - hardware failure
                LOGGER.warning("Error reading segment %s: %s", segment, exc)
                continue
            if not packet:
                continue
            roll = packet.get("roll")
            gyro = packet.get("scaledGyroX")
            if roll is None or gyro is None:
                continue
            new_sample = True
            if segment.endswith("_l"):
                roll = -roll
                gyro = -gyro
            angles[idx] = roll
            velocities[idx] = gyro
        if new_sample:
            combined = np.concatenate([angles, velocities])
            self._last_angles = angles
            self._last_velocities = velocities
            self._last_segments = combined
            return combined, True
        return None, False

    def _compute_joint_values(
        self, segment_angles: np.ndarray, segment_velocities: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        joint_angles = []
        joint_vels = []
        for name in self.joint_names:
            base, side = self._split_name(name)
            if base == "knee":
                shank = self._segment_value("shank", side, segment_angles)
                thigh = self._segment_value("thigh", side, segment_angles)
                shank_vel = self._segment_value("shank", side, segment_velocities)
                thigh_vel = self._segment_value("thigh", side, segment_velocities)
                joint_angles.append(self._difference(shank, thigh))
                joint_vels.append(self._difference(shank_vel, thigh_vel))
            elif base == "ankle":
                foot = self._segment_value("foot", side, segment_angles)
                shank = self._segment_value("shank", side, segment_angles)
                foot_vel = self._segment_value("foot", side, segment_velocities)
                shank_vel = self._segment_value("shank", side, segment_velocities)
                joint_angles.append(self._difference(foot, shank))
                joint_vels.append(self._difference(foot_vel, shank_vel))
            elif base == "hip":
                thigh = self._segment_value("thigh", side, segment_angles)
                trunk = self._segment_value("trunk", "", segment_angles)
                thigh_vel = self._segment_value("thigh", side, segment_velocities)
                trunk_vel = self._segment_value("trunk", "", segment_velocities)
                joint_angles.append(self._difference(thigh, trunk))
                joint_vels.append(self._difference(thigh_vel, trunk_vel))
            else:
                angle = self._segment_value(base, side, segment_angles)
                vel = self._segment_value(base, side, segment_velocities)
                joint_angles.append(angle if angle is not None else 0.0)
                joint_vels.append(vel if vel is not None else 0.0)
        return np.array(joint_angles, dtype=float), np.array(joint_vels, dtype=float)

    def _segment_value(self, base: str, side: str, values: np.ndarray) -> float | None:
        if side:
            key = f"{base}_{side}"
        else:
            key = base
        idx = self._segment_indices.get(key)
        if idx is None:
            return None
        return float(values[idx])

    @staticmethod
    def _difference(value_a: float | None, value_b: float | None) -> float:
        if value_a is None and value_b is None:
            return 0.0
        if value_a is None:
            return -float(value_b) if value_b is not None else 0.0
        if value_b is None:
            return float(value_a)
        return float(value_a) - float(value_b)

    @staticmethod
    def _split_name(name: str) -> Tuple[str, str]:
        if "_" in name:
            base, side = name.rsplit("_", 1)
            return base, side
        return name, ""
