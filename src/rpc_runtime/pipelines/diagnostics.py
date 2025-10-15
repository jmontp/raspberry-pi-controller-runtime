"""Diagnostics sinks for the controller runtime.

This module provides a minimal diagnostics interface used by the runtime loop.
It intentionally avoids heavy dependencies so unit tests can run in a constrained
environment. Implementations can later be extended to persist Parquet/Feather
logs or integrate with pandas without changing the `RuntimeLoop` contract.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Mapping

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
        # Nothing to do for an in-memory sink. Hook provided for API parity.
        return None

