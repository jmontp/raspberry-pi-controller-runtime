"""Diagnostics sinks for the controller runtime.

Provides an abstract interface and two implementations:
- NoOpDiagnosticsSink: drops diagnostics.
- InMemoryDiagnosticsSink: stores rows in memory for tests.
- CSVDiagnosticsSink: appends rows to a CSV with a dynamic header.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, TextIO

from ..actuators.base import TorqueCommand
from ..sensors.combinators import ControlInputs
from ..sensors.grf.base import VerticalGRFSample
from ..sensors.imu.base import IMUSample


class DiagnosticsSink:
    """Abstract sink API consumed by the runtime loop."""

    def log_tick(
        self,
        *,
        timestamp: float,
        features: Mapping[str, float],
        feature_packet: ControlInputs,
        torque_command_raw: TorqueCommand,
        torque_command_safe: TorqueCommand,
        scheduler: Mapping[str, float] | None = None,
    ) -> None:
        """Record a single control tick worth of diagnostics."""

    def flush(self) -> None:  # pragma: no cover - trivial default
        """Persist any buffered diagnostics to disk (optional)."""
        return None


class NoOpDiagnosticsSink(DiagnosticsSink):
    """A sink that discards all diagnostics events."""

    pass


@dataclass(slots=True)
class InMemoryDiagnosticsSink(DiagnosticsSink):
    """Lightweight sink that stores diagnostics rows in memory."""

    capacity: int = 10_000
    rows: list[dict] = field(default_factory=list, init=False)

    def log_tick(
        self,
        *,
        timestamp: float,
        features: Mapping[str, float],
        feature_packet: ControlInputs,
        torque_command_raw: TorqueCommand,
        torque_command_safe: TorqueCommand,
        scheduler: Mapping[str, float] | None = None,
    ) -> None:
        """Append a diagnostics row for a single control tick."""
        if len(self.rows) >= self.capacity:
            self.rows.pop(0)

        imu = _select_imu_sample(feature_packet)
        grf = _select_grf_sample(feature_packet)
        row = {
            "timestamp": float(timestamp),
            "imu_joint_angles": tuple(float(x) for x in getattr(imu, "joint_angles_rad", ())) if imu else (),
            "imu_joint_vel": tuple(float(x) for x in getattr(imu, "joint_velocities_rad_s", ())) if imu else (),
            "grf_forces": tuple(float(x) for x in getattr(grf, "forces_newton", ())) if grf else (),
            "torque_raw": {k: float(v) for k, v in torque_command_raw.torques_nm.items()},
            "torque_safe": {k: float(v) for k, v in torque_command_safe.torques_nm.items()},
            "features": dict(features),
        }
        if scheduler:
            row.update({f"scheduler_{k}": float(v) for k, v in scheduler.items()})
        self.rows.append(row)

    def flush(self) -> None:
        """Persist buffered rows (no-op for in-memory sink)."""
        return None


class CSVDiagnosticsSink(DiagnosticsSink):
    """Append control tick diagnostics to a CSV file.

    The header is derived from the first tick and includes joint/segment/GRF
    channels (when present), raw/safe torques, and scheduler metrics.
    """

    def __init__(
        self,
        path: str | Path,
        *,
        include_segments: bool = False,
    ) -> None:
        """Create a CSV diagnostics sink."""
        self._path = Path(path)
        self._include_segments = include_segments
        self._writer: csv.DictWriter | None = None
        self._file: TextIO | None = None
        self._columns: list[str] | None = None
        self._feature_keys: tuple[str, ...] | None = None
        self._torque_keys: tuple[str, ...] | None = None
        self._scheduler_keys: tuple[str, ...] | None = None

    def _ensure_writer(
        self,
        *,
        features: Mapping[str, float],
        torque_command_raw: TorqueCommand,
        torque_command_safe: TorqueCommand,
        scheduler: Mapping[str, float] | None,
    ) -> None:
        if self._writer is not None:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)

        feature_keys = sorted(features.keys())
        torque_keys = sorted(set(torque_command_raw.torques_nm) | set(torque_command_safe.torques_nm))
        scheduler_keys = sorted((scheduler or {}).keys())

        columns: list[str] = ["timestamp"] + feature_keys
        for k in torque_keys:
            columns.append(f"torque_raw_{k}")
        for k in torque_keys:
            columns.append(f"torque_safe_{k}")
        for k in scheduler_keys:
            columns.append(f"scheduler_{k}")

        self._columns = columns
        self._feature_keys = tuple(feature_keys)
        self._torque_keys = tuple(torque_keys)
        self._scheduler_keys = tuple(scheduler_keys)
        self._file = self._path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=columns)
        self._writer.writeheader()

    def log_tick(
        self,
        *,
        timestamp: float,
        features: Mapping[str, float],
        feature_packet: ControlInputs,
        torque_command_raw: TorqueCommand,
        torque_command_safe: TorqueCommand,
        scheduler: Mapping[str, float] | None = None,
    ) -> None:
        """Write a single row for the current control tick."""
        self._ensure_writer(
            features=features,
            torque_command_raw=torque_command_raw,
            torque_command_safe=torque_command_safe,
            scheduler=scheduler,
        )
        assert self._writer is not None and self._columns is not None

        row: dict[str, float] = {"timestamp": float(timestamp)}
        for key in self._feature_keys or ():
            value = features.get(key)
            if value is not None:
                row[key] = float(value)
        for k in self._torque_keys or ():
            row[f"torque_raw_{k}"] = float(torque_command_raw.torques_nm.get(k, 0.0))
        for k in self._torque_keys or ():
            row[f"torque_safe_{k}"] = float(torque_command_safe.torques_nm.get(k, 0.0))
        for k in self._scheduler_keys or ():
            if scheduler and k in scheduler:
                row[f"scheduler_{k}"] = float(scheduler[k])

        self._writer.writerow(row)
        if self._file is not None:
            self._file.flush()

    def flush(self) -> None:
        """Flush buffered data and close the CSV handle."""
        if self._file is not None:
            self._file.flush()

    def __del__(self) -> None:  # pragma: no cover - best-effort cleanup
        """Close the CSV file handle if still open."""
        try:
            if self._file is not None:
                self._file.close()
        except Exception:
            pass


def _select_imu_sample(feature_packet: ControlInputs) -> IMUSample | None:
    primary = feature_packet.imu
    if isinstance(primary, IMUSample):
        return primary
    for candidate in feature_packet.as_dict().values():
        if isinstance(candidate, IMUSample):
            return candidate
        if hasattr(candidate, "joint_angles_rad"):
            try:
                angles = candidate.joint_angles_rad  # type: ignore[attr-defined]
            except Exception:
                continue
            if angles is not None:
                return candidate  # type: ignore[return-value]
    return None


def _select_grf_sample(feature_packet: ControlInputs) -> VerticalGRFSample | None:
    primary = feature_packet.vertical_grf
    if isinstance(primary, VerticalGRFSample):
        return primary
    for candidate in feature_packet.as_dict().values():
        if isinstance(candidate, VerticalGRFSample):
            return candidate
        if hasattr(candidate, "forces_newton"):
            return candidate  # type: ignore[return-value]
    return None
