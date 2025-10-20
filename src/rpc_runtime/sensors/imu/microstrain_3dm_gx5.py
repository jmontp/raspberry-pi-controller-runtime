"""HBK MicroStrain 3DM-GX5-AHRS adapter implemented with MSCL bindings."""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass, field, replace
from typing import Any, Dict

from .base import (
    LOGGER,
    BaseIMU,
    BaseIMUConfig,
    CanonicalFeatureSemantics,
    IMUSample,
)

if "/usr/share/python3-mscl/" not in sys.path:
    sys.path.append("/usr/share/python3-mscl/")

try:  # pragma: no cover - hardware dependency
    import mscl
except ImportError:  # pragma: no cover - optional dependency for development
    mscl = None


def _default_port_map() -> Dict[str, str]:
    """Canonical feature to port defaults matching a right/left leg rig."""
    return {
        "trunk_sagittal_angle_rad": "/dev/ttyIMU_trunk",
        "trunk_sagittal_velocity_rad_s": "/dev/ttyIMU_trunk",
        "thigh_sagittal_angle_ipsi_rad": "/dev/ttyIMU_thigh_r",
        "thigh_sagittal_velocity_ipsi_rad_s": "/dev/ttyIMU_thigh_r",
        "shank_sagittal_angle_ipsi_rad": "/dev/ttyIMU_shank_r",
        "shank_sagittal_velocity_ipsi_rad_s": "/dev/ttyIMU_shank_r",
        "foot_sagittal_angle_ipsi_rad": "/dev/ttyIMU_foot_r",
        "foot_sagittal_velocity_ipsi_rad_s": "/dev/ttyIMU_foot_r",
        "thigh_sagittal_angle_contra_rad": "/dev/ttyIMU_thigh_l",
        "thigh_sagittal_velocity_contra_rad_s": "/dev/ttyIMU_thigh_l",
        "shank_sagittal_angle_contra_rad": "/dev/ttyIMU_shank_l",
        "shank_sagittal_velocity_contra_rad_s": "/dev/ttyIMU_shank_l",
        "foot_sagittal_angle_contra_rad": "/dev/ttyIMU_foot_l",
        "foot_sagittal_velocity_contra_rad_s": "/dev/ttyIMU_foot_l",
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


@dataclass(slots=True)
class _SegmentEntry:
    port: str
    sign: float
    angle_feature: str | None = None
    velocity_feature: str | None = None


class Microstrain3DMGX5IMU(BaseIMU):
    """Adapter for HBK MicroStrain 3DM-GX5-AHRS using MSCL."""

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
        self._segments: dict[str, _SegmentEntry] = self._build_segment_entries()
        self._feature_to_segment: dict[str, str] = {}
        for segment, entry in self._segments.items():
            if entry.angle_feature:
                self._feature_to_segment[entry.angle_feature] = segment
            if entry.velocity_feature:
                self._feature_to_segment[entry.velocity_feature] = segment
        self._zero_offsets: dict[str, float] = dict.fromkeys(self.port_map, 0.0)
        self._last_values: dict[str, float] = {}

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
        for segment, entry in self._segments.items():
            self._nodes[segment] = _MicrostrainNode(entry.port, self._timeout_ms)
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
        measurement, fresh = self._read_segments()
        if measurement is None:
            if not self._last_values:
                return self._handle_sample(None, fresh=False)
            measurement = dict(self._last_values)
        canonical = {}
        for feature, value in measurement.items():
            offset = self._zero_offsets.get(feature, 0.0)
            canonical[feature] = value - offset
        self._last_values = canonical
        sample = IMUSample(timestamp=time.monotonic(), values=canonical)
        return self._handle_sample(sample, fresh=fresh)

    def reset(self) -> None:
        """Re-calibrate zero offsets for the connected IMUs."""
        if not self._nodes:
            self._zero_offsets = dict.fromkeys(self.port_map, 0.0)
            return
        self._zero_offsets = self._calibrate()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_segment_entries(self) -> dict[str, _SegmentEntry]:
        entries: dict[str, _SegmentEntry] = {}
        for feature, port in self.port_map.items():
            semantics = self.feature_semantics(feature)
            segment = self._resolve_segment_name(semantics, feature)
            sign = -1.0 if segment.endswith("_l") else 1.0
            entry = entries.get(segment)
            if entry is None:
                entry = _SegmentEntry(port=port, sign=sign)
                entries[segment] = entry
            elif entry.port != port:
                raise ValueError(
                    f"Canonical feature '{feature}' reuses segment '{segment}' with a different port"
                )
            if semantics.derivative >= 1:
                if entry.velocity_feature and entry.velocity_feature != feature:
                    raise ValueError(
                        f"Duplicate velocity feature mapping detected for segment '{segment}'"
                    )
                entry.velocity_feature = feature
            else:
                if entry.angle_feature and entry.angle_feature != feature:
                    raise ValueError(
                        f"Duplicate angle feature mapping detected for segment '{segment}'"
                    )
                entry.angle_feature = feature
        return entries

    @staticmethod
    def _resolve_segment_name(semantics: CanonicalFeatureSemantics, feature: str) -> str:
        base = semantics.basis
        side = semantics.side
        if base == "trunk":
            return "trunk"
        if base == "thigh":
            return "thigh_r" if side in {None, "ipsi"} else "thigh_l"
        if base == "shank":
            return "shank_r" if side in {None, "ipsi"} else "shank_l"
        if base == "foot":
            return "foot_r" if side in {None, "ipsi"} else "foot_l"
        raise ValueError(
            f"MicroStrain driver cannot map canonical feature '{feature}' to a known segment"
        )

    def _calibrate(self) -> dict[str, float]:
        """Collect samples and compute average offsets."""
        aggregates: dict[str, list[float]] = {}
        for _ in range(self._calibration_samples):
            measurement, fresh = self._read_segments()
            if measurement is None or not fresh:
                time.sleep(self._calibration_interval)
                continue
            for feature, value in measurement.items():
                aggregates.setdefault(feature, []).append(value)
            time.sleep(self._calibration_interval)
        offsets: dict[str, float] = {}
        for feature in self.port_map:
            values = aggregates.get(feature, [])
            if values:
                offsets[feature] = sum(values) / len(values)
            else:
                offsets[feature] = 0.0
        return offsets

    def _read_segments(self) -> tuple[dict[str, float] | None, bool]:
        """Read roll/gyro data for all segments; returns values and freshness flag."""
        measurements: dict[str, float] = {}
        new_sample = False
        for segment, entry in self._segments.items():
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
            angle = packet.get("roll")
            velocity = packet.get("scaledGyroX")
            if angle is not None and entry.angle_feature:
                measurements[entry.angle_feature] = entry.sign * float(angle)
                new_sample = True
            if velocity is not None and entry.velocity_feature:
                measurements[entry.velocity_feature] = entry.sign * float(velocity)
                new_sample = True
        return (measurements, True) if new_sample else (None, False)
