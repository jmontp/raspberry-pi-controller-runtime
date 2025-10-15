"""Tests covering runtime configuration schema and profile loading."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from rpc_runtime.config import load_runtime_profile
from rpc_runtime.config.models import (
    HardwareAvailabilityError,
    InputSchema,
    SchemaSignal,
)
from rpc_runtime.config.runtime import build_runtime_components
from rpc_runtime.runtime.loop import RuntimeLoop, RuntimeLoopConfig
from rpc_runtime.sensors.combinators import ControlInputs
from rpc_runtime.sensors.imu.base import IMUSample


def _mock_imu_sample() -> IMUSample:
    return IMUSample(
        timestamp=0.0,
        joint_angles_rad=(0.1, -0.05),
        joint_velocities_rad_s=(0.2, -0.1),
        segment_angles_rad=(0.0, 0.0, 0.0),
        segment_velocities_rad_s=(0.0, 0.0, 0.0),
    )


def test_input_schema_optional_zero_fill() -> None:
    """Optional channels should fall back to the declared default value."""
    schema = InputSchema(
        name="test",
        signals=(
            SchemaSignal.from_registry("knee_angle", required=True),
            SchemaSignal.from_registry("grf_total", required=False, default=0.0),
        ),
    )
    inputs = ControlInputs(imu=_mock_imu_sample(), vertical_grf=None)
    features = schema.build_features(inputs)
    assert features["knee_angle"] == pytest.approx(0.1)
    assert features["grf_total"] == 0.0


def test_input_schema_missing_required_signal_raises() -> None:
    """Missing data for a required channel should surface an error."""
    schema = InputSchema(
        name="test",
        signals=(
            SchemaSignal.from_registry("knee_angle", required=True),
            SchemaSignal.from_registry("grf_total", required=True),
        ),
    )
    inputs = ControlInputs(imu=_mock_imu_sample(), vertical_grf=None)
    with pytest.raises(HardwareAvailabilityError):
        schema.build_features(inputs)


def test_runtime_profile_default_loop_executes() -> None:
    """Default YAML profile should load and drive a mock runtime loop."""
    profile_path = Path("src/rpc_runtime/config/default.yaml")
    profile = load_runtime_profile(profile_path)
    # Drop the optional GRF sensor to exercise zero-filling in the schema.
    sensors_wo_grf = tuple(sensor for sensor in profile.sensors if sensor.name != "vertical_grf")
    profile_wo_grf = replace(profile, sensors=sensors_wo_grf)
    profile_wo_grf.validate()
    components = build_runtime_components(profile_wo_grf)
    assert components.vertical_grf is None
    loop = RuntimeLoop(
        RuntimeLoopConfig(frequency_hz=100.0),
        imu=components.imu,
        actuator=components.actuator,
        controller=components.controller,
        vertical_grf=None,
    )
    with loop:
        for _ in loop.run(duration_s=0.05):
            pass
    actuator = components.actuator
    assert hasattr(actuator, "commanded")
    assert actuator.commanded, "Expected actuator commands to be recorded"
