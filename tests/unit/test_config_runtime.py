"""Tests covering runtime configuration schema and profile loading."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from rpc_runtime.config import build_runtime_components, load_runtime_profile
from rpc_runtime.config.models import SignalRoute
from rpc_runtime.runtime.loop import RuntimeLoop, RuntimeLoopConfig
from rpc_runtime.runtime.wrangler import (
    DataWrangler,
    HardwareAvailabilityError,
    InputSchema,
    SchemaSignal,
)
from rpc_runtime.sensors.imu.base import IMUSample
from rpc_runtime.sensors.imu.mock import MockIMU


def _mock_imu_sample() -> IMUSample:
    return IMUSample(
        timestamp=0.0,
        joint_angles_rad=(0.1, -0.05),
        joint_velocities_rad_s=(0.2, -0.1),
        segment_angles_rad=(0.0, 0.0, 0.0),
        segment_velocities_rad_s=(0.0, 0.0, 0.0),
    )


def test_wrangler_optional_signal_zero_fill() -> None:
    """Optional channels should fall back to the declared default value."""
    schema = InputSchema(
        name="test",
        signals=(
            SchemaSignal(name="knee_flexion_angle_ipsi_rad", required=True),
            SchemaSignal(name="vertical_grf_ipsi_N", required=False, default=0.0),
        ),
    )
    routes = (
        SignalRoute(
            name="knee_flexion_angle_ipsi_rad",
            provider="imu",
            default=0.0,
        ),
        SignalRoute(
            name="vertical_grf_ipsi_N",
            provider="vertical_grf",
            default=0.0,
        ),
    )
    imu = MockIMU(samples=[_mock_imu_sample()], loop=False)
    wrangler = DataWrangler(schema, routes, sensors={"imu": imu})
    with wrangler:
        view, _, _ = wrangler.get_sensor_data()
    assert view["knee_flexion_angle_ipsi_rad"] == pytest.approx(0.1)
    assert view["vertical_grf_ipsi_N"] == 0.0


def test_wrangler_missing_required_sensor_raises() -> None:
    """Missing sensors for required channels should raise during setup."""
    schema = InputSchema(
        name="test",
        signals=(
            SchemaSignal(name="knee_flexion_angle_ipsi_rad", required=True),
            SchemaSignal(name="vertical_grf_ipsi_N", required=True),
        ),
    )
    routes = (
        SignalRoute(
            name="knee_flexion_angle_ipsi_rad",
            provider="imu",
            default=0.0,
        ),
        SignalRoute(
            name="vertical_grf_ipsi_N",
            provider="vertical_grf",
            default=0.0,
        ),
    )
    imu = MockIMU(samples=[_mock_imu_sample()], loop=False)
    with pytest.raises(HardwareAvailabilityError):
        DataWrangler(schema, routes, sensors={"imu": imu})


def test_runtime_profile_default_loop_executes() -> None:
    """Default YAML profile should load and drive a mock runtime loop."""
    profile_path = Path("src/rpc_runtime/config/hardware_config.yaml")
    profile = load_runtime_profile(profile_path)
    # Drop the optional GRF sensor to exercise zero-filling in the schema.
    sensors_wo_grf = tuple(sensor for sensor in profile.sensors if sensor.name != "vertical_grf")
    profile_wo_grf = replace(profile, sensors=sensors_wo_grf)
    profile_wo_grf.validate()
    components = build_runtime_components(profile_wo_grf)
    loop = RuntimeLoop(
        RuntimeLoopConfig(frequency_hz=100.0),
        sensors=components.sensors,
        actuator=components.actuator,
        controller=components.controller,
        signal_routes=profile_wo_grf.signal_routes,
    )
    with loop:
        for _ in loop.run(duration_s=0.05):
            pass
    actuator = components.actuator
    assert hasattr(actuator, "commanded")
    assert actuator.commanded, "Expected actuator commands to be recorded"
