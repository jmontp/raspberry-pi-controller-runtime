"""Declarative configuration models for runtime manifests and schemas."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Mapping

from rpc_runtime.sensors.combinators import ControlInputs


class HardwareAvailabilityError(RuntimeError):
    """Raised when required sensors or signals are missing for a profile."""


# ---------------------------------------------------------------------------
# Canonical signal registry
# ---------------------------------------------------------------------------

#: Metadata describing the canonical signals recognised by the runtime.
#:
#: Keys are canonical signal names; values include default fill value and a
#: human-readable description covering semantics and units.
CANONICAL_SIGNAL_REGISTRY: dict[str, dict[str, Any]] = {
    "knee_angle": {
        "default": 0.0,
        "description": "Sagittal-plane knee joint angle in radians.",
    },
    "knee_velocity": {
        "default": 0.0,
        "description": "Sagittal-plane knee joint angular velocity in rad/s.",
    },
    "ankle_angle": {
        "default": 0.0,
        "description": "Sagittal-plane ankle joint angle in radians.",
    },
    "ankle_velocity": {
        "default": 0.0,
        "description": "Sagittal-plane ankle joint angular velocity in rad/s.",
    },
    "grf_total": {
        "default": 0.0,
        "description": "Total vertical ground reaction force in Newtons.",
    },
    "knee_torque": {
        "default": 0.0,
        "description": "Controller torque command for the knee joint (Nm).",
    },
    "ankle_torque": {
        "default": 0.0,
        "description": "Controller torque command for the ankle joint (Nm).",
    },
}


@dataclass(slots=True)
class SchemaSignal:
    """Describe a canonical signal expected by a controller model."""

    name: str
    required: bool = True
    default: float = 0.0
    description: str | None = None

    def resolve(self, inputs: ControlInputs) -> float | None:
        """Resolve the signal from canonical runtime inputs."""
        resolver = SIGNAL_RESOLVERS.get(self.name)
        if resolver is None:
            return None
        return resolver(inputs)

    @classmethod
    def from_registry(
        cls,
        name: str,
        *,
        required: bool = True,
        default: float | None = None,
        description: str | None = None,
    ) -> "SchemaSignal":
        """Create a `SchemaSignal`, filling defaults from the registry."""
        metadata = CANONICAL_SIGNAL_REGISTRY.get(name)
        if metadata is None:
            raise ValueError(
                f"Unknown canonical signal '{name}'. Update CANONICAL_SIGNAL_REGISTRY."
            )
        return cls(
            name=name,
            required=required,
            default=metadata["default"] if default is None else float(default),
            description=description or metadata.get("description"),
        )


JOINT_INDEX: dict[str, int] = {
    "knee": 0,
    "ankle": 1,
}


def _resolve_joint_signal(
    inputs: ControlInputs,
    joint: str,
    attribute: str,
) -> float | None:
    """Helper extracting joint-based metrics from IMU samples."""
    index = JOINT_INDEX.get(joint)
    if index is None:
        return None
    if attribute == "angle":
        try:
            return float(inputs.imu.joint_angles_rad[index])
        except (IndexError, TypeError):
            return None
    if attribute == "velocity":
        try:
            return float(inputs.imu.joint_velocities_rad_s[index])
        except (IndexError, TypeError):
            return None
    return None


def _resolve_grf_signal(inputs: ControlInputs, index: int) -> float | None:
    """Helper extracting GRF channels."""
    if inputs.vertical_grf is None:
        return None
    try:
        return float(inputs.vertical_grf.forces_newton[index])
    except (IndexError, TypeError):
        return None


def _resolve_torque_signal(_: ControlInputs) -> float | None:
    """Placeholder resolver for controller outputs (resolved elsewhere)."""
    return None


SIGNAL_RESOLVERS: dict[str, Callable[[ControlInputs], float | None]] = {
    "knee_angle": lambda inputs: _resolve_joint_signal(inputs, "knee", "angle"),
    "knee_velocity": lambda inputs: _resolve_joint_signal(inputs, "knee", "velocity"),
    "ankle_angle": lambda inputs: _resolve_joint_signal(inputs, "ankle", "angle"),
    "ankle_velocity": lambda inputs: _resolve_joint_signal(inputs, "ankle", "velocity"),
    "grf_total": lambda inputs: _resolve_grf_signal(inputs, 0),
    # Output placeholders for validation; actual values come from controller.
    "knee_torque": _resolve_torque_signal,
    "ankle_torque": _resolve_torque_signal,
}


@dataclass(slots=True)
class InputSchema:
    """Ordered collection of input channels backing a controller manifest."""

    name: str
    signals: tuple[SchemaSignal, ...]
    description: str | None = None

    def required_sensor_names(self) -> set[str]:
        """Return the set of sensors that must be present."""
        return set()

    def required_signals(self) -> set[str]:
        """Return the set of fully-qualified signals required by the schema."""
        return {
            signal.name
            for signal in self.signals
            if signal.required
        }

    def all_signals(self) -> set[str]:
        """Return the set of all signals referenced by the schema."""
        return {signal.name for signal in self.signals}

    def build_features(self, inputs: ControlInputs) -> dict[str, float]:
        """Return a dense feature dictionary following the declared ordering."""
        features: dict[str, float] = {}
        missing_required: list[str] = []
        for signal in self.signals:
            value = signal.resolve(inputs)
            if value is None:
                features[signal.name] = signal.default
                if signal.required:
                    missing_required.append(signal.name)
            else:
                features[signal.name] = value
        if missing_required:
            raise HardwareAvailabilityError(
                "Missing required signals while building features: "
                + ", ".join(sorted(missing_required))
            )
        return features

    def validate_signals(self, available_signals: Iterable[str]) -> None:
        """Ensure all required signals are available from configured sensors."""
        available = set(available_signals)
        missing = self.required_signals() - available
        if missing:
            raise HardwareAvailabilityError(
                "Configured hardware does not expose required signals: "
                + ", ".join(sorted(missing))
            )


@dataclass(slots=True)
class SensorBinding:
    """Describe how a logical sensor should be instantiated."""

    name: str
    driver: str
    provides: tuple[str, ...]
    config: Mapping[str, Any] = field(default_factory=dict)
    required: bool = True

    def available_signals(self) -> set[str]:
        """Return the signals exposed by this binding."""
        return set(self.provides)


@dataclass(slots=True)
class ActuatorBinding:
    """Describe how to instantiate the actuator stack for a profile."""

    driver: str
    config: Mapping[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class ControllerManifest:
    """Controller IO manifest capturing schema + actuator contract."""

    name: str
    input_schema: InputSchema
    output_schema: InputSchema | None
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
    config: Mapping[str, Any] = field(default_factory=dict)
    torque_model: Mapping[str, Any] | None = None


@dataclass(slots=True)
class RuntimeProfile:
    """Top-level runtime configuration assembled from declarative profiles."""

    name: str
    sensors: tuple[SensorBinding, ...]
    actuator: ActuatorBinding
    controller: ControllerBundle

    def available_signals(self) -> set[str]:
        """Return the union of signals exposed by all configured sensors."""
        signals = set()
        for sensor in self.sensors:
            signals.update(sensor.available_signals())
        return signals

    def validate(self) -> None:
        """Ensure the declared hardware satisfies controller requirements."""
        available = self.available_signals()
        self.controller.manifest.validate_hardware(available)
