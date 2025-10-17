"""Safety management utilities for clamping and fault policy."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from ..actuators.base import TorqueCommand


@dataclass(slots=True)
class SafetyConfig:
    """Configuration for :class:`SafetyManager`."""

    torque_limits_nm: Mapping[str, float] | None = None
    diagnostics_path: Path | None = None


@dataclass(slots=True)
class SafetyMetrics:
    """Runtime metrics surfaced by :class:`SafetyManager`."""

    clamp_events: int = 0
    fault_count: int = 0
    last_fault_reason: str | None = None


class SafetyManager:
    """Clamp torque commands and collect simple diagnostics."""

    def __init__(self, config: SafetyConfig | None = None) -> None:
        """Create a new safety manager with optional torque limits.

        Args:
            config: Safety configuration including optional per-joint limits.
        """
        self._config = config or SafetyConfig()
        self._metrics = SafetyMetrics()
        self._diagnostics_path = self._config.diagnostics_path

    @property
    def metrics(self) -> SafetyMetrics:
        """Return runtime safety metrics (e.g., clamp event count)."""
        return self._metrics

    @property
    def limits(self) -> Mapping[str, float] | None:
        """Expose the configured torque limits when available."""
        return self._config.torque_limits_nm

    @property
    def diagnostics_path(self) -> Path | None:
        """Path used for safety diagnostics artefacts when configured."""
        return self._diagnostics_path

    def enforce(self, command: TorqueCommand) -> TorqueCommand:
        """Clamp torque commands to configured limits.

        Args:
            command: Candidate torque command to validate and clamp.

        Returns:
            TorqueCommand: A new command with torques clipped to limits.
        """
        limits = self._config.torque_limits_nm
        if not limits:
            return command
        clamped = False
        sanitized: dict[str, float] = {}
        for joint, torque in command.torques_nm.items():
            limit = limits.get(joint)
            if limit is None:
                sanitized[joint] = float(torque)
                continue
            limit = float(limit)
            t = float(torque)
            if abs(t) > limit:
                clamped = True
                t = max(-limit, min(limit, t))
            sanitized[joint] = t
        if clamped:
            self._metrics.clamp_events += 1
        return TorqueCommand(timestamp=command.timestamp, torques_nm=sanitized)

    def record_fault(self, reason: str) -> None:
        """Record a safety fault for diagnostics and reporting."""
        self._metrics.fault_count += 1
        self._metrics.last_fault_reason = reason
