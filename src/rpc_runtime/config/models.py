"""Minimal configuration models for runtime profiles.

This layer remains declarative; canonical signal routing stays with the runtime
so configuration objects avoid hardware dependencies.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping


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
class SignalRoute:
    """Ordered mapping from canonical signal to a hardware provider."""

    name: str
    provider: str | None
    default: float = 0.0

    @property
    def derived(self) -> bool:
        """Signal is derived by software rather than hardware."""
        return self.provider is None


@dataclass(slots=True)
class RuntimeProfile:
    """Top-level runtime configuration assembled from declarative profiles."""

    name: str
    sensors: tuple[SensorBinding, ...]
    actuator: ActuatorBinding
    controller: ControllerBundle
    signal_routes: tuple[SignalRoute, ...]

    def available_signals(self) -> set[str]:
        """Return the union of signals exposed by all configured sensors."""
        signals: set[str] = set()
        for sensor in self.sensors:
            signals.update(sensor.available_signals())
        return signals

    def sensor_bindings(self) -> Mapping[str, SensorBinding]:
        """Return sensor bindings keyed by alias."""
        return {sensor.name: sensor for sensor in self.sensors}

    def validate(self) -> None:
        """Ensure the declared hardware satisfies controller requirements."""
        available = self.available_signals()
        self.controller.manifest.validate_hardware(available)
        bindings = self.sensor_bindings()
        required_signals = self.controller.manifest.input_schema.required_signals()
        for route in self.signal_routes:
            if route.provider is None:
                continue
            binding = bindings.get(route.provider)
            if binding is None:
                if route.name in required_signals:
                    raise HardwareAvailabilityError(
                        f"Signal '{route.name}' references unknown sensor '{route.provider}'"
                    )
                continue
            if route.name not in binding.provides:
                raise HardwareAvailabilityError(
                    f"Sensor '{route.provider}' does not provide signal '{route.name}'"
                )
