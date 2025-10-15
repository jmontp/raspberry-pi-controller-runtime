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

    profile_name = str(profile_section.get("name", profile_path.stem))
    profile = _load_structured_profile(raw, profile_section, profile_name)
    profile.validate()
    return profile


# ---------------------------------------------------------------------------
# Profile parsing helpers
# ---------------------------------------------------------------------------


def _load_structured_profile(
    raw: Mapping[str, Any],
    profile_section: Mapping[str, Any],
    profile_name: str,
) -> RuntimeProfile:
    """Load profiles that follow the documented input/output/hardware layout."""
    hardware_section = _require_mapping(raw.get("hardware"), "hardware")
    hardware_sensors = _require_mapping(hardware_section.get("sensors"), "hardware['sensors']")
    hardware_actuators = _require_mapping(
        hardware_section.get("actuators"), "hardware['actuators']"
    )

    input_signals_raw = raw.get("input_signals")
    if not isinstance(input_signals_raw, list) or not input_signals_raw:
        raise ValueError("Profile YAML must define a non-empty 'input_signals' list")

    sensor_signal_map: dict[str, list[str]] = {str(name): [] for name in hardware_sensors}
    sensor_required_map: dict[str, bool] = {str(name): False for name in hardware_sensors}
    input_schema_signals: list[SchemaSignal] = []
    for idx, entry in enumerate(input_signals_raw):
        if not isinstance(entry, Mapping):
            raise ValueError(f"'input_signals' entry #{idx} must be a mapping")
        signal_name = entry.get("name") or entry.get("signal")
        hardware_alias = entry.get("hardware")
        if signal_name is None:
            raise ValueError(f"'input_signals' entry #{idx} missing 'name'")
        if hardware_alias is None:
            raise ValueError(f"'input_signals' entry #{idx} missing 'hardware'")
        hardware_alias = str(hardware_alias)
        if hardware_alias not in sensor_signal_map:
            raise ValueError(
                f"'input_signals' entry #{idx} references unknown sensor alias '{hardware_alias}'"
            )
        required = bool(entry.get("required", True))
        default_value = entry.get("default", 0.0)
        if default_value is None:
            default_value = 0.0
        try:
            default_numeric = float(default_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"'input_signals' entry '{signal_name}' has invalid default '{default_value}'"
            ) from exc
        input_schema_signals.append(
            SchemaSignal(
                name=str(signal_name),
                required=required,
                default=default_numeric,
            )
        )
        sensor_signal_map[hardware_alias].append(str(signal_name))
        if required:
            sensor_required_map[hardware_alias] = True

    input_schema_name = f"{profile_name}_inputs"
    input_schema = InputSchema(name=input_schema_name, signals=tuple(input_schema_signals))

    output_signals_raw = raw.get("output_signals")
    if not isinstance(output_signals_raw, list) or not output_signals_raw:
        raise ValueError("Profile YAML must define a non-empty 'output_signals' list")
    actuator_signal_map: dict[str, list[str]] = {str(name): [] for name in hardware_actuators}
    output_schema_signals: list[SchemaSignal] = []
    for idx, entry in enumerate(output_signals_raw):
        if not isinstance(entry, Mapping):
            raise ValueError(f"'output_signals' entry #{idx} must be a mapping")
        signal_name = entry.get("name") or entry.get("signal")
        hardware_alias = entry.get("hardware")
        if signal_name is None:
            raise ValueError(f"'output_signals' entry #{idx} missing 'name'")
        if hardware_alias is None:
            raise ValueError(f"'output_signals' entry #{idx} missing 'hardware'")
        hardware_alias = str(hardware_alias)
        if hardware_alias not in actuator_signal_map:
            raise ValueError(
                f"'output_signals' entry #{idx} references unknown actuator alias '{hardware_alias}'"
            )
        output_schema_signals.append(SchemaSignal(name=str(signal_name), required=True))
        actuator_signal_map[hardware_alias].append(str(signal_name))

    output_schema_name = f"{profile_name}_outputs"
    output_schema = InputSchema(name=output_schema_name, signals=tuple(output_schema_signals))

    schemas: dict[str, InputSchema] = {
        input_schema_name: input_schema,
        output_schema_name: output_schema,
    }
    controllers = _parse_controllers(
        raw.get("controllers", {}),
        schemas,
        default_input_schema=input_schema_name,
        default_output_schema=output_schema_name,
    )

    controller_key = profile_section.get("controller")
    if controller_key not in controllers:
        raise ValueError(f"Unknown controller bundle '{controller_key}' referenced by profile")
    controller_bundle = controllers[str(controller_key)]

    used_sensor_aliases = [alias for alias, signals in sensor_signal_map.items() if signals]
    if not used_sensor_aliases:
        raise ValueError("No sensors referenced by 'input_signals'")

    sensor_bindings: list[SensorBinding] = []
    for alias in used_sensor_aliases:
        payload = _require_mapping(
            hardware_sensors.get(alias), f"hardware['sensors']['{alias}']"
        )
        driver = payload.get("class") or payload.get("driver")
        if driver is None:
            raise ValueError(f"Sensor '{alias}' missing 'class' entry")
        config_payload = payload.get("config", {})
        if config_payload is None:
            config_payload = {}
        if not isinstance(config_payload, Mapping):
            raise ValueError(f"Sensor '{alias}' config must be a mapping")
        config: dict[str, Any] = dict(config_payload)
        for key, value in payload.items():
            if key in {"class", "driver", "config"}:
                continue
            config[key] = value
        provides = tuple(sensor_signal_map[alias])
        if not provides:
            raise ValueError(f"Sensor '{alias}' does not provide any signals")
        sensor_bindings.append(
            SensorBinding(
                name=str(alias),
                driver=str(driver),
                provides=provides,
                config=config,
                required=sensor_required_map.get(alias, True),
            )
        )

    used_actuator_aliases = [alias for alias, signals in actuator_signal_map.items() if signals]
    if not used_actuator_aliases:
        raise ValueError("No actuators referenced by 'output_signals'")
    if len(set(used_actuator_aliases)) != 1:
        raise NotImplementedError(
            "Multiple actuator aliases referenced; the minimal runtime supports a single actuator"
        )
    actuator_alias = used_actuator_aliases[0]
    actuator_payload = _require_mapping(
        hardware_actuators.get(actuator_alias), f"hardware['actuators']['{actuator_alias}']"
    )
    actuator_driver = actuator_payload.get("class") or actuator_payload.get("driver")
    if actuator_driver is None:
        raise ValueError(f"Actuator '{actuator_alias}' missing 'class' entry")
    actuator_config_payload = actuator_payload.get("config", {})
    if actuator_config_payload is None:
        actuator_config_payload = {}
    if not isinstance(actuator_config_payload, Mapping):
        raise ValueError(f"Actuator '{actuator_alias}' config must be a mapping")
    actuator_config: dict[str, Any] = dict(actuator_config_payload)
    for key, value in actuator_payload.items():
        if key in {"class", "driver", "config"}:
            continue
        actuator_config[key] = value
    actuator_binding = ActuatorBinding(
        driver=str(actuator_driver),
        config=actuator_config,
    )

    return RuntimeProfile(
        name=profile_name,
        sensors=tuple(sensor_bindings),
        actuator=actuator_binding,
        controller=controller_bundle,
    )


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


def _parse_controllers(
    raw: Mapping[str, Any],
    schemas: Mapping[str, InputSchema],
    *,
    default_input_schema: str | None = None,
    default_output_schema: str | None = None,
) -> dict[str, ControllerBundle]:
    controllers: dict[str, ControllerBundle] = {}
    for name, payload in raw.items():
        mapping = _require_mapping(payload, f"controllers['{name}']")
        implementation = mapping.get("implementation")
        if implementation is None:
            raise ValueError(f"Controller '{name}' missing 'implementation'")
        input_schema_name = mapping.get("input_schema", mapping.get("schema"))
        if input_schema_name is None:
            if default_input_schema is None:
                raise ValueError(
                    f"Controller '{name}' missing 'input_schema' and no default schema available"
                )
            if default_input_schema not in schemas:
                raise ValueError(
                    f"Default input schema '{default_input_schema}' not defined for controller '{name}'"
                )
            input_schema = schemas[str(default_input_schema)]
        else:
            if input_schema_name not in schemas:
                raise ValueError(
                    f"Controller '{name}' references unknown input schema '{input_schema_name}'"
                )
            input_schema = schemas[str(input_schema_name)]
        output_schema_name = mapping.get("output_schema")
        output_schema = None
        if output_schema_name is None:
            if default_output_schema is not None and default_output_schema in schemas:
                output_schema = schemas[str(default_output_schema)]
        else:
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
            input_schema=input_schema,
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
