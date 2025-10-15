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
            provider="imu_mock",
            default=0.0,
        ),
        SignalRoute(
            name="vertical_grf_ipsi_N",
            provider="vertical_grf_mock",
            default=0.0,
        ),
    )
    imu = MockIMU(samples=[_mock_imu_sample()], loop=False)
    wrangler = DataWrangler(schema, routes, sensors={"imu_mock": imu})
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
            provider="imu_mock",
            default=0.0,
        ),
        SignalRoute(
            name="vertical_grf_ipsi_N",
            provider="vertical_grf_mock",
            default=0.0,
        ),
    )
    imu = MockIMU(samples=[_mock_imu_sample()], loop=False)
    with pytest.raises(HardwareAvailabilityError):
        DataWrangler(schema, routes, sensors={"imu_mock": imu})


def test_runtime_profile_default_loop_executes() -> None:
    """Default YAML profile should load and drive a mock runtime loop."""
    profile_path = Path("src/rpc_runtime/config/hardware_config.yaml")
    profile = load_runtime_profile(profile_path)
    profile.validate()
    components = build_runtime_components(profile)
    loop = RuntimeLoop(
        RuntimeLoopConfig(frequency_hz=100.0),
        sensors=components.sensors,
        actuator=components.actuator,
        controller=components.controller,
        signal_routes=profile.signal_routes,
    )
    with loop:
        for _ in loop.run(duration_s=0.05):
            pass
    actuator = components.actuator
    assert hasattr(actuator, "commanded")
    assert actuator.commanded, "Expected actuator commands to be recorded"


def test_profile_side_alias_canonicalisation(tmp_path: Path) -> None:
    """Profiles using *_right/*_left aliases should canonicalise to ipsi/contra."""
    alias_yaml = tmp_path / "side_alias.yaml"
    alias_yaml.write_text(
        """
profile:
  name: alias_example
  controller: pi_alias

input_signals:
  - { name: knee_flexion_angle_right_rad, hardware: imu_mock }
  - { name: knee_flexion_velocity_right_rad_s, hardware: imu_mock }
  - { name: ankle_dorsiflexion_angle_right_rad, hardware: imu_mock }
  - { name: ankle_dorsiflexion_velocity_right_rad_s, hardware: imu_mock }

output_signals:
  - { name: knee_flexion_moment_right_Nm, hardware: actuator_mock }
  - { name: ankle_dorsiflexion_moment_right_Nm, hardware: actuator_mock }

hardware:
  sensors:
    imu_mock:
      class: rpc_runtime.sensors.imu.mock.MockIMU
      config: {}
  actuators:
    actuator_mock:
      class: rpc_runtime.actuators.mock.MockActuator
      config: {}

controllers:
  pi_alias:
    implementation: rpc_runtime.controllers.pi_controller.PIController
    joints: ["knee_flexion_moment_right_Nm", "ankle_dorsiflexion_moment_right_Nm"]
    config:
      dt: 0.01
      torque_scale: 0.1
      torque_limit_nm: 10.0
      gains:
        kp:
          knee_flexion_moment_right_Nm: 0.0
          ankle_dorsiflexion_moment_right_Nm: 0.0
        ki:
          knee_flexion_moment_right_Nm: 0.0
          ankle_dorsiflexion_moment_right_Nm: 0.0
    torque_model:
      implementation: rpc_runtime.controllers.torque_models.mock.MockTorqueModel
      config:
        outputs:
          knee_flexion_moment_right_Nm: 0.0
          ankle_dorsiflexion_moment_right_Nm: 0.0
"""
    )
    profile = load_runtime_profile(alias_yaml)
    route_names = {route.name for route in profile.signal_routes}
    assert "knee_flexion_angle_ipsi_rad" in route_names
    assert "ankle_dorsiflexion_angle_ipsi_rad" in route_names
    assert profile.controller.manifest.joints == (
        "knee_flexion_moment_ipsi_Nm",
        "ankle_dorsiflexion_moment_ipsi_Nm",
    )
    gains = profile.controller.config.get("gains", {})
    assert gains["kp"]["knee_flexion_moment_ipsi_Nm"] == 0.0
