"""Load declarative runtime profiles from YAML files."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

from .models import (
    ActuatorBinding,
    ControllerBundle,
    ControllerManifest,
    HardwareAvailabilityError,
    InputSchema,
    RuntimeProfile,
    SchemaSignal,
    SensorBinding,
)


def load_runtime_profile(path: str | Path) -> RuntimeProfile:
    """Parse a YAML profile describing sensors, controllers, and actuators.

    Args:
        path: Filesystem path to the profile YAML file.

    Returns:
        RuntimeProfile: Parsed profile ready for hardware instantiation.

    Raises:
        FileNotFoundError: If the profile path does not exist.
        ValueError: When required sections or fields are missing.
        HardwareAvailabilityError: If declared hardware cannot satisfy the
            controller manifest.
    """
    profile_path = Path(path)
    if not profile_path.exists():
        raise FileNotFoundError(profile_path)
    with profile_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    profile_section = raw.get("profile")
    if not isinstance(profile_section, Mapping):
        raise ValueError("Profile YAML missing top-level 'profile' mapping")

    schemas = _parse_schemas(raw.get("schemas", {}))
    controllers = _parse_controllers(raw.get("controllers", {}), schemas)
    sensors = _parse_sensors(raw.get("sensors", {}))
    actuators = _parse_actuators(raw.get("actuators", {}))

    profile_name = str(profile_section.get("name", profile_path.stem))
    controller_key = profile_section.get("controller")
    if controller_key is None:
        raise ValueError("Profile mapping must include a 'controller' entry")
    if controller_key not in controllers:
        raise ValueError(f"Unknown controller bundle '{controller_key}' referenced by profile")
    controller_bundle = controllers[controller_key]

    actuator_key = profile_section.get("actuator")
    if actuator_key is None:
        raise ValueError("Profile mapping must include an 'actuator' entry")
    if actuator_key not in actuators:
        raise ValueError(f"Unknown actuator binding '{actuator_key}' referenced by profile")
    actuator_binding = actuators[actuator_key]

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
        binding_template = sensors[binding_key]
        sensor_bindings.append(
            SensorBinding(
                name=str(logical_name),
                driver=binding_template.driver,
                provides=binding_template.provides,
                config=binding_template.config,
                required=bool(entry.get("required", binding_template.required)),
            )
        )

    profile = RuntimeProfile(
        name=profile_name,
        sensors=tuple(sensor_bindings),
        actuator=actuator_binding,
        controller=controller_bundle,
    )
    try:
        profile.validate()
    except HardwareAvailabilityError as exc:
        raise HardwareAvailabilityError(f"Profile '{profile_name}' invalid: {exc}") from exc
    return profile


def _parse_schemas(raw_schemas: Mapping[str, Any]) -> dict[str, InputSchema]:
    schemas: dict[str, InputSchema] = {}
    for name, payload in raw_schemas.items():
        if not isinstance(payload, Mapping):
            raise ValueError(f"Schema '{name}' must be a mapping")
        channels_payload = payload.get("channels", [])
        if not isinstance(channels_payload, list) or not channels_payload:
            raise ValueError(f"Schema '{name}' must define a non-empty 'channels' list")
        channels: list[SchemaSignal] = []
        for idx, definition in enumerate(channels_payload):
            if isinstance(definition, str):
                channels.append(SchemaSignal.from_registry(str(definition)))
                continue
            if not isinstance(definition, Mapping):
                raise ValueError(f"Schema '{name}' channel #{idx} must be a mapping or string")
            signal_name = definition.get("signal", definition.get("name"))
            if not signal_name:
                raise ValueError(f"Schema '{name}' channel #{idx} missing 'signal'")
            channel = SchemaSignal.from_registry(
                str(signal_name),
                required=bool(definition.get("required", True)),
                default=definition.get("default"),
                description=(definition.get("description") or None),
            )
            channels.append(channel)
        schema = InputSchema(
            name=str(name),
            signals=tuple(channels),
            description=(payload.get("description") or None),
        )
        schemas[str(name)] = schema
    return schemas


def _parse_controllers(
    raw_controllers: Mapping[str, Any],
    schemas: Mapping[str, InputSchema],
) -> dict[str, ControllerBundle]:
    controllers: dict[str, ControllerBundle] = {}
    for name, payload in raw_controllers.items():
        if not isinstance(payload, Mapping):
            raise ValueError(f"Controller '{name}' must be a mapping")
        implementation = payload.get("implementation")
        if implementation is None:
            raise ValueError(f"Controller '{name}' missing 'implementation'")
        input_schema_name = payload.get("input_schema", payload.get("schema"))
        if input_schema_name not in schemas:
            raise ValueError(
                f"Controller '{name}' references unknown input schema '{input_schema_name}'"
            )
        output_schema_name = payload.get("output_schema")
        output_schema = None
        if output_schema_name is not None:
            if output_schema_name not in schemas:
                raise ValueError(
                    f"Controller '{name}' references unknown output schema '{output_schema_name}'"
                )
            output_schema = schemas[str(output_schema_name)]
        joints = payload.get("joints")
        if not isinstance(joints, list) or not joints:
            raise ValueError(f"Controller '{name}' must define non-empty 'joints'")
        manifest = ControllerManifest(
            name=str(name),
            input_schema=schemas[str(input_schema_name)],
            output_schema=output_schema,
            joints=tuple(str(joint) for joint in joints),
            description=(payload.get("description") or None),
        )
        config_payload = payload.get("config", {})
        if not isinstance(config_payload, Mapping):
            raise ValueError(f"Controller '{name}' config must be a mapping")
        torque_model_payload = payload.get("torque_model")
        if torque_model_payload is not None and not isinstance(
            torque_model_payload, Mapping
        ):
            raise ValueError(f"Controller '{name}' torque_model must be a mapping")
        controllers[str(name)] = ControllerBundle(
            name=str(name),
            implementation=str(implementation),
            manifest=manifest,
            config=dict(config_payload),
            torque_model=dict(torque_model_payload) if torque_model_payload else None,
        )
    return controllers


def _parse_sensors(raw_sensors: Mapping[str, Any]) -> dict[str, SensorBinding]:
    sensors: dict[str, SensorBinding] = {}
    for name, payload in raw_sensors.items():
        if not isinstance(payload, Mapping):
            raise ValueError(f"Sensor '{name}' must be a mapping")
        driver = payload.get("driver")
        if driver is None:
            raise ValueError(f"Sensor '{name}' missing 'driver'")
        provides = payload.get("provides", [])
        if not isinstance(provides, list) or not provides:
            raise ValueError(f"Sensor '{name}' must define a non-empty 'provides' list")
        config_payload = payload.get("config", {})
        if not isinstance(config_payload, Mapping):
            raise ValueError(f"Sensor '{name}' config must be a mapping")
        sensors[str(name)] = SensorBinding(
            name=str(name),
            driver=str(driver),
            provides=tuple(str(signal) for signal in provides),
            config=dict(config_payload),
            required=bool(payload.get("required", True)),
        )
    return sensors


def _parse_actuators(raw_actuators: Mapping[str, Any]) -> dict[str, ActuatorBinding]:
    actuators: dict[str, ActuatorBinding] = {}
    for name, payload in raw_actuators.items():
        if not isinstance(payload, Mapping):
            raise ValueError(f"Actuator '{name}' must be a mapping")
        driver = payload.get("driver")
        if driver is None:
            raise ValueError(f"Actuator '{name}' missing 'driver'")
        config_payload = payload.get("config", {})
        if not isinstance(config_payload, Mapping):
            raise ValueError(f"Actuator '{name}' config must be a mapping")
        actuators[str(name)] = ActuatorBinding(
            driver=str(driver),
            config=dict(config_payload),
        )
    return actuators
