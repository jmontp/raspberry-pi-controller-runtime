"""Minimal configuration models for runtime profiles.

This layer remains declarative; canonical signal resolution lives in
`rpc_runtime.runtime.wrangler` to keep configuration free of runtime deps.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable


class HardwareAvailabilityError(RuntimeError):
    """Raised when required sensors or signals are missing for a profile."""


@dataclass(slots=True)
class SensorBinding:
    """Describe how a logical sensor should be instantiated."""

    name: str
    driver: str
    provides: tuple[str, ...]
    config: dict[str, Any] = field(default_factory=dict)
    required: bool = True

    def available_signals(self) -> set[str]:
        """Return the signals exposed by this binding."""
        return set(self.provides)


@dataclass(slots=True)
class ActuatorBinding:
    """Describe how to instantiate the actuator stack for a profile."""

    driver: str
    config: dict[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class ControllerManifest:
    """Controller IO manifest capturing schema + actuator contract."""

    name: str
    input_schema: Any
    output_schema: Any | None
    joints: tuple[str, ...]
    description: str | None = None

    def validate_hardware(self, available_signals: Iterable[str]) -> None:
        """Assert the declared hardware exposes the required signals."""
        self.input_schema.validate_signals(available_signals)


@dataclass(slots=True)
class ControllerBundle:
    """Controller configuration bundle linking manifests and implementation."""

    name: str
    implementation: str
    manifest: ControllerManifest
    config: dict[str, Any] = field(default_factory=dict)
    torque_model: dict[str, Any] | None = None


@dataclass(slots=True)
class RuntimeProfile:
    """Top-level runtime configuration assembled from declarative profiles."""

    name: str
    sensors: tuple[SensorBinding, ...]
    actuator: ActuatorBinding
    controller: ControllerBundle

    def available_signals(self) -> set[str]:
        """Return the union of signals exposed by all configured sensors."""
        signals: set[str] = set()
        for sensor in self.sensors:
            signals.update(sensor.available_signals())
        return signals

    def validate(self) -> None:
        """Ensure the declared hardware satisfies controller requirements."""
        available = self.available_signals()
        self.controller.manifest.validate_hardware(available)

