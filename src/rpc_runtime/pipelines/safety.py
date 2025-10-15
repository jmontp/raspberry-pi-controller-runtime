"""Safety management utilities for clamping and fault policy.

This is a minimal implementation that clamps torques using simple per-joint
limits. Actuators already enforce limits at apply-time; the safety layer exists
to make the clamping explicit in the runtime loop and to surface counters for
diagnostics.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

from ..actuators.base import TorqueCommand


@dataclass(slots=True)
class SafetyConfig:
    """Configuration for :class:`SafetyManager`."""

    torque_limits_nm: Mapping[str, float] | None = None


@dataclass(slots=True)
class SafetyMetrics:
    """Runtime metrics surfaced by :class:`SafetyManager`."""

    clamp_events: int = 0


class SafetyManager:
    """Clamp torque commands and collect simple diagnostics."""

    def __init__(self, config: SafetyConfig | None = None) -> None:
        self._config = config or SafetyConfig()
        self._metrics = SafetyMetrics()

    @property
    def metrics(self) -> SafetyMetrics:
        return self._metrics

    def enforce(self, command: TorqueCommand) -> TorqueCommand:
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

