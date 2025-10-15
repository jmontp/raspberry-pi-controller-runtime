"""Profile loader and runtime factory rolled into a single minimal module."""

from __future__ import annotations

import importlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping

import yaml

from rpc_runtime.actuators.base import BaseActuator
from rpc_runtime.controllers.pi_controller import (
    PIController,
    PIControllerConfig,
    PIControllerGains,
)
from rpc_runtime.controllers.torque_models.base import TorqueModel
from rpc_runtime.runtime.wrangler import (
    InputSchema,
    SchemaSignal,
)
from rpc_runtime.sensors.base import BaseSensor
from rpc_runtime.sensors.grf.base import BaseVerticalGRF
from rpc_runtime.sensors.imu.base import BaseIMU

# ---------------------------------------------------------------------------
# Data models
# ---------------------------------------------------------------------------


@dataclass(slots=True)
class SensorBinding:
    """Binding between a logical sensor name and a concrete driver."""

    name: str
    driver: str
    provides: tuple[str, ...]
    config: dict[str, Any] = field(default_factory=dict)
    required: bool = True

    def available_signals(self) -> set[str]:
        """Signals exposed by this sensor binding."""
        return set(self.provides)


@dataclass(slots=True)
class ActuatorBinding:
    """Driver path and config payload for the actuator stack."""

    driver: str
    config: dict[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class ControllerManifest:
    """Controller IO manifest describing schemas and joints."""

    name: str
    input_schema: InputSchema
    output_schema: InputSchema | None
    joints: tuple[str, ...]
    description: str | None = None

    def validate_hardware(self, available_signals: Iterable[str]) -> None:
        """Ensure required signals are offered by the hardware."""
        self.input_schema.validate_signals(available_signals)


@dataclass(slots=True)
class ControllerBundle:
    """Controller implementation reference plus configuration payloads."""

    name: str
    implementation: str
    manifest: ControllerManifest
    config: dict[str, Any] = field(default_factory=dict)
    torque_model: dict[str, Any] | None = None


@dataclass(slots=True)
class RuntimeProfile:
    """In-memory representation of a declarative runtime profile."""

    name: str
    sensors: tuple[SensorBinding, ...]
    actuator: ActuatorBinding
    controller: ControllerBundle

    def available_signals(self) -> set[str]:
        """Union of signals provided by all configured sensors."""
        signals: set[str] = set()
        for sensor in self.sensors:
            signals.update(sensor.available_signals())
        return signals

    def validate(self) -> None:
        """Validate that sensors can satisfy the controller manifest."""
        available = self.available_signals()
        self.controller.manifest.validate_hardware(available)


@dataclass(slots=True)
class RuntimeComponents:
    """Concrete runtime objects instantiated from a profile."""

    profile: RuntimeProfile
    imu: BaseIMU
    actuator: BaseActuator
    controller: PIController
    torque_model: TorqueModel
    vertical_grf: BaseVerticalGRF | None
    sensors: Dict[str, BaseSensor]


# ---------------------------------------------------------------------------
# Profile loading
# ---------------------------------------------------------------------------


def load_runtime_profile(path: str | Path) -> RuntimeProfile:
    """Load and validate a YAML runtime profile."""
    profile_path = Path(path)
    if not profile_path.exists():
        raise FileNotFoundError(profile_path)
    with profile_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    profile_section = _require_mapping(raw.get("profile"), "profile")

    schemas = _parse_schemas(raw.get("schemas", {}))
    controllers = _parse_controllers(raw.get("controllers", {}), schemas)
    sensors = _parse_sensors(raw.get("sensors", {}))
    actuators = _parse_actuators(raw.get("actuators", {}))

    profile_name = str(profile_section.get("name", profile_path.stem))
    controller_key = profile_section.get("controller")
    if controller_key not in controllers:
        raise ValueError(f"Unknown controller bundle '{controller_key}' referenced by profile")
    controller_bundle = controllers[str(controller_key)]

    actuator_key = profile_section.get("actuator")
    if actuator_key not in actuators:
        raise ValueError(f"Unknown actuator binding '{actuator_key}' referenced by profile")
    actuator_binding = actuators[str(actuator_key)]

    sensor_entries = profile_section.get("sensors", [])
    if not isinstance(sensor_entries, list):
        raise ValueError("'sensors' entry in profile must be a list")
    sensor_bindings: list[SensorBinding] = []
    for entry in sensor_entries:
        if not isinstance(entry, Mapping):
            raise ValueError("Sensor entry must be a mapping with 'name' and 'binding'")
        logical_name = entry.get("name")
        binding_key = entry.get("binding", logical_name)
        if logical_name is None:
            raise ValueError("Sensor entry missing 'name'")
        if binding_key not in sensors:
            raise ValueError(f"Sensor binding '{binding_key}' not defined")
        template = sensors[str(binding_key)]
        sensor_bindings.append(
            SensorBinding(
                name=str(logical_name),
                driver=template.driver,
                provides=template.provides,
                config=dict(template.config),
                required=bool(entry.get("required", template.required)),
            )
        )

    profile = RuntimeProfile(
        name=profile_name,
        sensors=tuple(sensor_bindings),
        actuator=actuator_binding,
        controller=controller_bundle,
    )
    profile.validate()
    return profile


# ---------------------------------------------------------------------------
# Runtime instantiation
# ---------------------------------------------------------------------------


def build_runtime_components(profile: RuntimeProfile) -> RuntimeComponents:
    """Instantiate hardware/controller objects described by a profile."""
    sensor_instances: Dict[str, BaseSensor] = {}
    for binding in profile.sensors:
        sensor_cls = _import_symbol(binding.driver)
        instance = sensor_cls(**binding.config)
        if not isinstance(instance, BaseSensor):
            raise TypeError(
                f"Sensor binding '{binding.name}' produced non-sensor instance {instance!r}"
            )
        sensor_instances[binding.name] = instance

    imu = _resolve_single(sensor_instances, BaseIMU, preferred_name="imu")
    vertical_grf = _resolve_optional(
        sensor_instances,
        BaseVerticalGRF,
        preferred_name="vertical_grf",
    )

    actuator_cls = _import_symbol(profile.actuator.driver)
    actuator = actuator_cls(**profile.actuator.config)
    if not isinstance(actuator, BaseActuator):
        raise TypeError(f"Actuator binding produced unexpected instance {actuator!r}")

    controller, torque_model = _build_controller(profile.controller)

    return RuntimeComponents(
        profile=profile,
        imu=imu,
        actuator=actuator,
        controller=controller,
        torque_model=torque_model,
        vertical_grf=vertical_grf,
        sensors=sensor_instances,
    )


def load_components(path: str | Path) -> RuntimeComponents:
    """Load a profile and build runtime components in one step."""
    return build_runtime_components(load_runtime_profile(path))


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"Profile YAML missing '{label}' mapping")
    return value


def _parse_schemas(raw: Mapping[str, Any]) -> dict[str, InputSchema]:
    schemas: dict[str, InputSchema] = {}
    for name, payload in raw.items():
        mapping = _require_mapping(payload, f"schemas['{name}']")
        channels_payload = mapping.get("channels", [])
        if not isinstance(channels_payload, list) or not channels_payload:
            raise ValueError(f"Schema '{name}' must define a non-empty 'channels' list")
        channels: list[SchemaSignal] = []
        for idx, definition in enumerate(channels_payload):
            if isinstance(definition, str):
                channels.append(SchemaSignal(name=str(definition)))
                continue
            if not isinstance(definition, Mapping):
                raise ValueError(f"Schema '{name}' channel #{idx} must be a mapping or string")
            signal_name = definition.get("signal", definition.get("name"))
            if not signal_name:
                raise ValueError(f"Schema '{name}' channel #{idx} missing 'signal'")
            default = definition.get("default")
            channels.append(
                SchemaSignal(
                    name=str(signal_name),
                    required=bool(definition.get("required", True)),
                    default=float(default) if default is not None else 0.0,
                )
            )
        schemas[str(name)] = InputSchema(
            name=str(name),
            signals=tuple(channels),
            description=(mapping.get("description") or None),
        )
    return schemas


def _parse_controllers(
    raw: Mapping[str, Any],
    schemas: Mapping[str, InputSchema],
) -> dict[str, ControllerBundle]:
    controllers: dict[str, ControllerBundle] = {}
    for name, payload in raw.items():
        mapping = _require_mapping(payload, f"controllers['{name}']")
        implementation = mapping.get("implementation")
        if implementation is None:
            raise ValueError(f"Controller '{name}' missing 'implementation'")
        input_schema_name = mapping.get("input_schema", mapping.get("schema"))
        if input_schema_name not in schemas:
            raise ValueError(
                f"Controller '{name}' references unknown input schema '{input_schema_name}'"
            )
        output_schema_name = mapping.get("output_schema")
        output_schema = None
        if output_schema_name is not None:
            if output_schema_name not in schemas:
                raise ValueError(
                    f"Controller '{name}' references unknown output schema '{output_schema_name}'"
                )
            output_schema = schemas[str(output_schema_name)]
        joints = mapping.get("joints")
        if not isinstance(joints, list) or not joints:
            raise ValueError(f"Controller '{name}' must define non-empty 'joints'")
        manifest = ControllerManifest(
            name=str(name),
            input_schema=schemas[str(input_schema_name)],
            output_schema=output_schema,
            joints=tuple(str(joint) for joint in joints),
            description=(mapping.get("description") or None),
        )
        config_payload = mapping.get("config", {})
        if not isinstance(config_payload, Mapping):
            raise ValueError(f"Controller '{name}' config must be a mapping")
        torque_model_payload = mapping.get("torque_model")
        if torque_model_payload is not None and not isinstance(torque_model_payload, Mapping):
            raise ValueError(f"Controller '{name}' torque_model must be a mapping")
        controllers[str(name)] = ControllerBundle(
            name=str(name),
            implementation=str(implementation),
            manifest=manifest,
            config=dict(config_payload),
            torque_model=dict(torque_model_payload) if torque_model_payload else None,
        )
    return controllers


def _parse_sensors(raw: Mapping[str, Any]) -> dict[str, SensorBinding]:
    sensors: dict[str, SensorBinding] = {}
    for name, payload in raw.items():
        mapping = _require_mapping(payload, f"sensors['{name}']")
        driver = mapping.get("driver")
        if driver is None:
            raise ValueError(f"Sensor '{name}' missing 'driver'")
        provides = mapping.get("provides", [])
        if not isinstance(provides, list) or not provides:
            raise ValueError(f"Sensor '{name}' must define a non-empty 'provides' list")
        config_payload = mapping.get("config", {})
        if not isinstance(config_payload, Mapping):
            raise ValueError(f"Sensor '{name}' config must be a mapping")
        sensors[str(name)] = SensorBinding(
            name=str(name),
            driver=str(driver),
            provides=tuple(str(signal) for signal in provides),
            config=dict(config_payload),
            required=bool(mapping.get("required", True)),
        )
    return sensors


def _parse_actuators(raw: Mapping[str, Any]) -> dict[str, ActuatorBinding]:
    actuators: dict[str, ActuatorBinding] = {}
    for name, payload in raw.items():
        mapping = _require_mapping(payload, f"actuators['{name}']")
        driver = mapping.get("driver")
        if driver is None:
            raise ValueError(f"Actuator '{name}' missing 'driver'")
        config_payload = mapping.get("config", {})
        if not isinstance(config_payload, Mapping):
            raise ValueError(f"Actuator '{name}' config must be a mapping")
        actuators[str(name)] = ActuatorBinding(
            driver=str(driver),
            config=dict(config_payload),
        )
    return actuators


def _build_controller(bundle: ControllerBundle) -> tuple[PIController, TorqueModel]:
    controller_cls = _import_symbol(bundle.implementation)
    if controller_cls is not PIController:
        raise NotImplementedError(
            f"Controller implementation '{bundle.implementation}' is not supported yet"
        )

    config_payload = dict(bundle.config)
    try:
        dt = float(config_payload.pop("dt"))
        torque_scale = float(config_payload.pop("torque_scale"))
        torque_limit_nm = float(config_payload.pop("torque_limit_nm"))
    except KeyError as exc:  # pragma: no cover - defensive
        raise ValueError(f"Controller bundle '{bundle.name}' missing field {exc}") from exc
    velocity_alpha = float(config_payload.pop("velocity_filter_alpha", 0.1))
    torque_alpha = float(config_payload.pop("torque_filter_alpha", 0.1))
    gains_payload = config_payload.pop("gains", {})
    if not isinstance(gains_payload, dict):
        raise ValueError("Controller gains configuration must be a mapping")
    kp = gains_payload.get("kp", {})
    ki = gains_payload.get("ki", {})

    controller_config = PIControllerConfig(
        dt=dt,
        torque_scale=torque_scale,
        torque_limit_nm=torque_limit_nm,
        joints=bundle.manifest.joints,
        velocity_filter_alpha=velocity_alpha,
        torque_filter_alpha=torque_alpha,
    )
    controller_gains = PIControllerGains(kp=dict(kp), ki=dict(ki))

    torque_model_info = bundle.torque_model or {}
    torque_model_impl = torque_model_info.get("implementation")
    if torque_model_impl is None:
        raise ValueError(f"Controller '{bundle.name}' missing torque model implementation")
    torque_model_cls = _import_symbol(str(torque_model_impl))
    torque_model_kwargs = dict(torque_model_info.get("config", {}))
    torque_model = torque_model_cls(**torque_model_kwargs)
    if not isinstance(torque_model, TorqueModel):
        raise TypeError(
            f"Torque model implementation '{torque_model_impl}' did not return a TorqueModel"
        )

    controller = controller_cls(
        config=controller_config,
        gains=controller_gains,
        torque_model=torque_model,
        input_schema=bundle.manifest.input_schema,
    )
    return controller, torque_model


def _import_symbol(path: str):
    module_name, _, attr = path.rpartition(".")
    if not module_name:
        raise ValueError(f"Invalid import path '{path}'")
    module = importlib.import_module(module_name)
    try:
        return getattr(module, attr)
    except AttributeError as exc:  # pragma: no cover - defensive
        raise AttributeError(f"{path} is not a valid attribute") from exc


def _resolve_single(
    sensors: Dict[str, BaseSensor],
    sensor_type: type,
    *,
    preferred_name: str,
) -> Any:
    instance = sensors.get(preferred_name)
    if instance is not None:
        if not isinstance(instance, sensor_type):
            raise TypeError(
                f"Sensor '{preferred_name}' is not of expected type {sensor_type.__name__}"
            )
        return instance
    matches = [sensor for sensor in sensors.values() if isinstance(sensor, sensor_type)]
    if len(matches) != 1:
        raise ValueError(
            f"Expected exactly one {sensor_type.__name__} instance, found {len(matches)}"
        )
    return matches[0]


def _resolve_optional(
    sensors: Dict[str, BaseSensor],
    sensor_type: type,
    *,
    preferred_name: str,
) -> Any | None:
    instance = sensors.get(preferred_name)
    if instance is not None:
        if not isinstance(instance, sensor_type):
            raise TypeError(
                f"Sensor '{preferred_name}' is not of expected type {sensor_type.__name__}"
            )
        return instance
    matches = [sensor for sensor in sensors.values() if isinstance(sensor, sensor_type)]
    if not matches:
        return None
    if len(matches) > 1:
        raise ValueError(
            f"Expected at most one {sensor_type.__name__} instance, found {len(matches)}"
        )
    return matches[0]
