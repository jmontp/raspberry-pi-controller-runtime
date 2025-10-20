"""Profile loader and runtime factory rolled into a single minimal module."""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping

import yaml  # type: ignore[import-untyped]

from rpc_runtime.actuators.base import BaseActuator
from rpc_runtime.controllers.pi_controller import PIController, PIControllerConfig
from rpc_runtime.controllers.torque_models.base import TorqueModel
from rpc_runtime.runtime.wrangler import InputSchema, SchemaSignal
from rpc_runtime.sensors.base import BaseSensor

from .models import (
    ActuatorBinding,
    ControllerBundle,
    ControllerManifest,
    DiagnosticsConfig,
    SensorBinding,
    SignalRoute,
)
from .models import (
    RuntimeProfile as ProfileModel,
)

RuntimeProfile = ProfileModel

# ---------------------------------------------------------------------------
# Data models
# ---------------------------------------------------------------------------


def _canonicalize_side_name(name: str) -> str:
    """Map *_right/*_left aliases to canonical ipsi/contra names."""
    if not isinstance(name, str):
        return name
    replacements = [
        ("_right_", "_ipsi_"),
        ("_left_", "_contra_"),
        ("_right", "_ipsi"),
        ("_left", "_contra"),
    ]
    canonical = name
    for old, new in replacements:
        canonical = canonical.replace(old, new)
    return canonical


@dataclass(slots=True)
class RuntimeComponents:
    """Concrete runtime objects instantiated from a profile."""

    profile: RuntimeProfile
    actuator: BaseActuator
    controller: PIController
    torque_model: TorqueModel
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
    signal_routes: list[SignalRoute] = []
    input_schema_signals: list[SchemaSignal] = []
    for idx, entry in enumerate(input_signals_raw):
        if not isinstance(entry, Mapping):
            raise ValueError(f"'input_signals' entry #{idx} must be a mapping")
        signal_name_raw = entry.get("name") or entry.get("signal")
        signal_name = _canonicalize_side_name(str(signal_name_raw)) if signal_name_raw else None
        if signal_name is None:
            raise ValueError(f"'input_signals' entry #{idx} missing 'name'")
        hardware_alias_raw = entry.get("hardware")
        provider: str | None
        if hardware_alias_raw is None:
            provider = None
        else:
            provider = str(hardware_alias_raw)
            if provider not in sensor_signal_map:
                raise ValueError(
                    f"'input_signals' entry #{idx} references unknown sensor alias '{provider}'"
                )
        required_flag = entry.get("required")
        required = bool(required_flag) if required_flag is not None else True
        input_schema_signals.append(
            SchemaSignal(
                name=str(signal_name),
                required=required,
            )
        )
        signal_routes.append(
            SignalRoute(
                name=signal_name,
                provider=provider,
            )
        )
        if provider is not None:
            sensor_signal_map[provider].append(signal_name)
            if required:
                sensor_required_map[provider] = True

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
        signal_name_raw = entry.get("name") or entry.get("signal")
        hardware_alias = entry.get("hardware")
        if signal_name_raw is None:
            raise ValueError(f"'output_signals' entry #{idx} missing 'name'")
        signal_name = _canonicalize_side_name(str(signal_name_raw))
        if hardware_alias is None:
            raise ValueError(f"'output_signals' entry #{idx} missing 'hardware'")
        hardware_alias = str(hardware_alias)
        if hardware_alias not in actuator_signal_map:
            raise ValueError(
                f"'output_signals' entry #{idx} references unknown actuator alias '{hardware_alias}'"
            )
        output_schema_signals.append(SchemaSignal(name=signal_name, required=True))
        actuator_signal_map[hardware_alias].append(signal_name)

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
        payload = _require_mapping(hardware_sensors.get(alias), f"hardware['sensors']['{alias}']")
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

    diagnostics_config = _parse_diagnostics_config(raw.get("diagnostics"))

    return RuntimeProfile(
        name=profile_name,
        sensors=tuple(sensor_bindings),
        actuator=actuator_binding,
        controller=controller_bundle,
        signal_routes=tuple(signal_routes),
        diagnostics=diagnostics_config,
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

    actuator_cls = _import_symbol(profile.actuator.driver)
    actuator = actuator_cls(**profile.actuator.config)
    if not isinstance(actuator, BaseActuator):
        raise TypeError(f"Actuator binding produced unexpected instance {actuator!r}")

    controller, torque_model = _build_controller(profile.controller)

    return RuntimeComponents(
        profile=profile,
        actuator=actuator,
        controller=controller,
        torque_model=torque_model,
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


def _parse_diagnostics_config(raw: Any) -> DiagnosticsConfig | None:
    """Normalise optional diagnostics configuration from profile YAML."""
    if raw is None:
        return None
    mapping = _require_mapping(raw, "diagnostics")
    artifacts_root = mapping.get("artifacts_root")
    if artifacts_root is None and "root" in mapping:
        artifacts_root = mapping["root"]
    artifacts_root_str = str(artifacts_root).strip() if artifacts_root is not None else None
    if artifacts_root_str:
        artifacts_root_value = artifacts_root_str
    else:
        artifacts_root_value = None
    rtplot_enabled_raw = mapping.get("rtplot_enabled")
    if rtplot_enabled_raw is None and "rtplot" in mapping:
        rtplot_enabled_raw = mapping["rtplot"]
    rtplot_enabled_value: bool | None
    if rtplot_enabled_raw is None:
        rtplot_enabled_value = None
    else:
        rtplot_enabled_value = bool(rtplot_enabled_raw)
    rtplot_host_raw = mapping.get("rtplot_host")
    rtplot_host_value = None
    if rtplot_host_raw is not None:
        rtplot_host_value = str(rtplot_host_raw).strip()
        if not rtplot_host_value:
            rtplot_host_value = None
    if rtplot_enabled_value is None and rtplot_host_value is not None:
        rtplot_enabled_value = True
    if rtplot_enabled_value is None:
        rtplot_enabled_value = False
    return DiagnosticsConfig(
        artifacts_root=artifacts_root_value,
        rtplot_enabled=rtplot_enabled_value,
        rtplot_host=rtplot_host_value,
    )


def _normalise_artifacts_root(value: str | Path | None) -> Path | None:
    if value is None:
        return None
    raw = str(value).strip()
    if not raw or raw.lower() == "none":
        return None
    return Path(raw).expanduser().resolve()


def _normalise_rtplot_host(value: str | None) -> str | None:
    if value is None:
        return None
    host = value.strip()
    return host or None


def resolve_diagnostics_settings(
    profile: RuntimeProfile,
    *,
    cli_root: str | Path | None,
    default_root: Path | None,
    cli_rtplot_host: str | None,
    rtplot_requested: bool,
) -> tuple[Path | None, str | None]:
    """Resolve diagnostics artifacts directory and rtplot host for runtime execution."""
    cli_root_path = _normalise_artifacts_root(cli_root)
    cli_root_disabled = False
    if cli_root is not None and cli_root_path is None:
        raw = str(cli_root).strip()
        if not raw or raw.lower() == "none":
            cli_root_disabled = True

    profile_root = _normalise_artifacts_root(
        getattr(profile.diagnostics, "artifacts_root", None) if profile.diagnostics else None
    )
    default_root_path = _normalise_artifacts_root(default_root)

    if cli_root_disabled:
        resolved_root: Path | None = None
    elif cli_root_path is not None:
        resolved_root = cli_root_path
    else:
        resolved_root = profile_root or default_root_path

    cli_host = _normalise_rtplot_host(cli_rtplot_host)
    profile_host = _normalise_rtplot_host(
        getattr(profile.diagnostics, "rtplot_host", None) if profile.diagnostics else None
    )

    rtplot_enabled = rtplot_requested or bool(cli_host)
    if not rtplot_enabled and profile.diagnostics is not None:
        if profile.diagnostics.rtplot_enabled or profile_host:
            rtplot_enabled = True

    resolved_host = cli_host or profile_host
    if rtplot_enabled and not resolved_host:
        resolved_host = "local"
    if not rtplot_enabled:
        resolved_host = None

    return resolved_root, resolved_host


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
        joints_raw = mapping.get("joints")
        if not isinstance(joints_raw, list) or not joints_raw:
            raise ValueError(f"Controller '{name}' must define non-empty 'joints'")
        joints = tuple(_canonicalize_side_name(str(joint)) for joint in joints_raw)
        manifest = ControllerManifest(
            name=str(name),
            input_schema=input_schema,
            output_schema=output_schema,
            joints=joints,
            description=(mapping.get("description") or None),
        )
        config_payload = mapping.get("config", {})
        if not isinstance(config_payload, Mapping):
            raise ValueError(f"Controller '{name}' config must be a mapping")
        config_dict: dict[str, Any] = dict(config_payload)
        torque_model_payload = mapping.get("torque_model")
        if torque_model_payload is not None and not isinstance(torque_model_payload, Mapping):
            raise ValueError(f"Controller '{name}' torque_model must be a mapping")
        torque_model_dict = None
        if torque_model_payload:
            torque_model_dict = dict(torque_model_payload)
            outputs_payload = torque_model_dict.get("outputs")
            if isinstance(outputs_payload, Mapping):
                torque_model_dict["outputs"] = {
                    _canonicalize_side_name(str(key)): value
                    for key, value in outputs_payload.items()
                }
        controllers[str(name)] = ControllerBundle(
            name=str(name),
            implementation=str(implementation),
            manifest=manifest,
            config=config_dict,
            torque_model=torque_model_dict,
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
    controller_config = PIControllerConfig(
        dt=dt,
        torque_scale=torque_scale,
        torque_limit_nm=torque_limit_nm,
        joints=bundle.manifest.joints,
    )

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
