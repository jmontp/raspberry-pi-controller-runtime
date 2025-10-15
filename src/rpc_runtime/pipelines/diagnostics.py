"""Diagnostics sinks for the controller runtime.

This module provides a minimal diagnostics interface used by the runtime loop.
It intentionally avoids heavy dependencies so unit tests can run in a constrained
environment. Implementations can later be extended to persist Parquet/Feather
logs or integrate with pandas without changing the `RuntimeLoop` contract.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping

from ..actuators.base import TorqueCommand
from ..sensors.combinators import ControlInputs


class DiagnosticsSink:
    """Abstract sink API consumed by the runtime loop.

    Concrete sinks should override :meth:`log_tick` and :meth:`flush`.
    """

    def log_tick(
        self,
        *,
        timestamp: float,
        feature_packet: ControlInputs,
        torque_command_raw: TorqueCommand,
        torque_command_safe: TorqueCommand,
        scheduler: Mapping[str, float] | None = None,
    ) -> None:
        """Write a single row for the current control tick.

        Args:
            timestamp: Monotonic timestamp of the IMU sample.
            feature_packet: Raw inputs used by the controller (for context).
            torque_command_raw: Controller output before safety clamping.
            torque_command_safe: Command after safety enforcement.
            scheduler: Optional scheduler metrics to include in the row.
        """
        """Record a single control tick worth of diagnostics.

        The default implementation is a no-op to keep the loop robust when no
        diagnostics are desired.
        """

    def flush(self) -> None:  # pragma: no cover - trivial default
        """Persist any buffered diagnostics to disk (optional)."""
        return None


class NoOpDiagnosticsSink(DiagnosticsSink):
    """A sink that discards all diagnostics events."""

    pass


@dataclass(slots=True)
class InMemoryDiagnosticsSink(DiagnosticsSink):
    """Lightweight sink that stores diagnostics rows in memory.

    This implementation aims to be dependency-free and suitable for tests. It
    keeps a bounded list of rows with select fields extracted from inputs and
    commands. Consumers can access :attr:`rows` for inspection.
    """

    capacity: int = 10_000
    rows: list[dict] = field(default_factory=list, init=False)

    def log_tick(
        self,
        *,
        timestamp: float,
        feature_packet: ControlInputs,
        torque_command_raw: TorqueCommand,
        torque_command_safe: TorqueCommand,
        scheduler: Mapping[str, float] | None = None,
    ) -> None:
        """Append a diagnostics row for a single control tick.

        Args:
            timestamp: Monotonic timestamp of the IMU sample.
            feature_packet: Raw inputs used by the controller (for context).
            torque_command_raw: Controller output before safety clamping.
            torque_command_safe: Command after safety enforcement.
            scheduler: Optional scheduler metrics to include in the row.
        """
        if len(self.rows) >= self.capacity:
            # Drop oldest to keep memory bounded
            self.rows.pop(0)

        imu = feature_packet.imu
        grf = feature_packet.vertical_grf
        row = {
            "timestamp": float(timestamp),
            # Minimal IMU snapshot
            "imu_joint_angles": tuple(float(x) for x in imu.joint_angles_rad),
            "imu_joint_vel": tuple(float(x) for x in imu.joint_velocities_rad_s),
            # Optional GRF
            "grf_forces": tuple(float(x) for x in (grf.forces_newton if grf else ())),
            # Torques
            "torque_raw": {k: float(v) for k, v in torque_command_raw.torques_nm.items()},
            "torque_safe": {k: float(v) for k, v in torque_command_safe.torques_nm.items()},
        }
        if scheduler:
            row.update({f"scheduler_{k}": float(v) for k, v in scheduler.items()})
        self.rows.append(row)

    def flush(self) -> None:
        """Persist buffered rows (no-op for in-memory sink)."""
        return None


class CSVDiagnosticsSink(DiagnosticsSink):
    """Append control tick diagnostics to a CSV file.

    The header is derived from the first tick and includes:
    - ``timestamp``
    - ``imu_joint_angle_{i}`` and ``imu_joint_vel_{i}``
    - optional ``seg_angle_{i}``, ``seg_vel_{i}`` when enabled
    - optional ``grf_force_{i}`` when GRF is present
    - ``torque_raw_{joint}`` and ``torque_safe_{joint}``
    - ``scheduler_{key}`` for provided scheduler metrics
    """

    def __init__(
        self,
        path: str | Path,
        *,
        include_segments: bool = False,
    ) -> None:
        """Create a CSV diagnostics sink.

        Args:
            path: Destination CSV filepath for the run log.
            include_segments: Include segment angles/velocities in the log.
        """
        self._path = Path(path)
        self._include_segments = include_segments
        self._writer: csv.DictWriter | None = None
        self._file = None
        self._columns: list[str] | None = None
        self._torque_keys: tuple[str, ...] | None = None
        self._scheduler_keys: tuple[str, ...] | None = None

    def _ensure_writer(
        self,
        *,
        feature_packet: ControlInputs,
        torque_command_raw: TorqueCommand,
        torque_command_safe: TorqueCommand,
        scheduler: Mapping[str, float] | None,
    ) -> None:
        if self._writer is not None:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)

        imu = feature_packet.imu
        grf = feature_packet.vertical_grf
        n_joints = len(imu.joint_angles_rad)
        n_segments = len(imu.segment_angles_rad)
        n_grf = len(grf.forces_newton) if grf is not None else 0

        torque_keys = sorted(
            set(torque_command_raw.torques_nm) | set(torque_command_safe.torques_nm)
        )
        scheduler_keys = sorted((scheduler or {}).keys())

        columns: list[str] = ["timestamp"]
        columns += [f"imu_joint_angle_{i}" for i in range(n_joints)]
        columns += [f"imu_joint_vel_{i}" for i in range(n_joints)]
        if self._include_segments:
            columns += [f"seg_angle_{i}" for i in range(n_segments)]
            columns += [f"seg_vel_{i}" for i in range(n_segments)]
        if n_grf:
            columns += [f"grf_force_{i}" for i in range(n_grf)]
        for k in torque_keys:
            columns.append(f"torque_raw_{k}")
        for k in torque_keys:
            columns.append(f"torque_safe_{k}")
        for k in scheduler_keys:
            columns.append(f"scheduler_{k}")

        self._columns = columns
        self._torque_keys = tuple(torque_keys)
        self._scheduler_keys = tuple(scheduler_keys)
        self._file = self._path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=columns)
        self._writer.writeheader()

    def log_tick(
        self,
        *,
        timestamp: float,
        feature_packet: ControlInputs,
        torque_command_raw: TorqueCommand,
        torque_command_safe: TorqueCommand,
        scheduler: Mapping[str, float] | None = None,
    ) -> None:
        """Write a single row for the current control tick.

        Args:
            timestamp: Monotonic timestamp of the IMU sample.
            feature_packet: Raw inputs used by the controller (for context).
            torque_command_raw: Controller output before safety clamping.
            torque_command_safe: Command after safety enforcement.
            scheduler: Optional scheduler metrics to include in the row.
        """
        self._ensure_writer(
            feature_packet=feature_packet,
            torque_command_raw=torque_command_raw,
            torque_command_safe=torque_command_safe,
            scheduler=scheduler,
        )
        assert self._writer is not None and self._columns is not None

        imu = feature_packet.imu
        grf = feature_packet.vertical_grf
        row: dict[str, float] = {"timestamp": float(timestamp)}

        for i, v in enumerate(imu.joint_angles_rad):
            row[f"imu_joint_angle_{i}"] = float(v)
        for i, v in enumerate(imu.joint_velocities_rad_s):
            row[f"imu_joint_vel_{i}"] = float(v)
        if self._include_segments:
            for i, v in enumerate(imu.segment_angles_rad):
                row[f"seg_angle_{i}"] = float(v)
            for i, v in enumerate(imu.segment_velocities_rad_s):
                row[f"seg_vel_{i}"] = float(v)
        if grf is not None:
            for i, v in enumerate(grf.forces_newton):
                row[f"grf_force_{i}"] = float(v)

        # Torques: fill missing keys with 0.0 for a dense CSV.
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
