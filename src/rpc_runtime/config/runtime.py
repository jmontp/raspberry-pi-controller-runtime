"""Factory helpers for instantiating runtime components from profiles."""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from typing import Any, Dict

from rpc_runtime.actuators.base import BaseActuator
from rpc_runtime.controllers.pi_controller import (
    PIController,
    PIControllerConfig,
    PIControllerGains,
)
from rpc_runtime.controllers.torque_models.base import TorqueModel
from rpc_runtime.sensors.base import BaseSensor
from rpc_runtime.sensors.grf.base import BaseVerticalGRF
from rpc_runtime.sensors.imu.base import BaseIMU

from .models import ControllerBundle, RuntimeProfile


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


def build_runtime_components(profile: RuntimeProfile) -> RuntimeComponents:
    """Instantiate sensors, controller, and actuator defined by a profile."""
    sensor_instances: Dict[str, BaseSensor] = {}
    for sensor in profile.sensors:
        sensor_cls = _import_symbol(sensor.driver)
        kwargs = dict(sensor.config)
        instance = sensor_cls(**kwargs)
        if not isinstance(instance, BaseSensor):
            raise TypeError(
                f"Sensor binding '{sensor.name}' produced non-sensor instance {instance!r}"
            )
        sensor_instances[sensor.name] = instance

    imu = _resolve_single(sensor_instances, BaseIMU, preferred_name="imu")
    vertical_grf = _resolve_optional(sensor_instances, BaseVerticalGRF, preferred_name="vertical_grf")

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
    controller_gains = PIControllerGains(
        kp=dict(kp),
        ki=dict(ki),
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
