"""Unit tests covering controller+sensor interactions with mock devices."""

import pytest

from rpc_runtime.actuators.mock import MockActuator
from rpc_runtime.config.models import SignalRoute
from rpc_runtime.controllers.pi_controller import PIController, PIControllerConfig
from rpc_runtime.controllers.torque_models.mock import MockTorqueModel
from rpc_runtime.runtime.loop import RuntimeLoop, RuntimeLoopConfig
from rpc_runtime.runtime.wrangler import InputSchema, SchemaSignal
from rpc_runtime.sensors.grf.mock import MockVerticalGRF
from rpc_runtime.sensors.imu.base import BaseIMUConfig, IMUSample, IMUStaleDataError
from rpc_runtime.sensors.imu.mock import MockIMU

KNEE_TORQUE = "knee_flexion_moment_ipsi_Nm"
ANKLE_TORQUE = "ankle_dorsiflexion_moment_ipsi_Nm"


def _canonical_sample() -> IMUSample:
    """Construct a canonical IMU sample with consistent limb kinematics."""
    values = {
        "trunk_sagittal_angle_rad": 0.3,
        "thigh_sagittal_angle_ipsi_rad": 0.2,
        "shank_sagittal_angle_ipsi_rad": 0.1,
        "foot_sagittal_angle_ipsi_rad": 0.05,
        "thigh_sagittal_angle_contra_rad": -0.2,
        "shank_sagittal_angle_contra_rad": -0.1,
        "foot_sagittal_angle_contra_rad": -0.04,
        "trunk_sagittal_velocity_rad_s": 0.0,
        "thigh_sagittal_velocity_ipsi_rad_s": 0.05,
        "shank_sagittal_velocity_ipsi_rad_s": 0.02,
        "foot_sagittal_velocity_ipsi_rad_s": 0.01,
        "thigh_sagittal_velocity_contra_rad_s": -0.04,
        "shank_sagittal_velocity_contra_rad_s": -0.015,
        "foot_sagittal_velocity_contra_rad_s": -0.005,
    }
    return IMUSample(timestamp=0.0, values=values)


def test_controller_with_mock_devices() -> None:
    """Integration-style check covering controller + scheduling with mocks."""
    imu_sample = _canonical_sample()
    mock_imu = MockIMU(samples=[imu_sample], loop=True)
    assert mock_imu.port_map, "Mock IMU should expose a default port map"
    assert "thigh_sagittal_angle_ipsi_rad" in mock_imu.port_map
    mock_grf = MockVerticalGRF()
    mock_actuator = MockActuator()
    torque_model = MockTorqueModel(outputs={KNEE_TORQUE: 0.5, ANKLE_TORQUE: -0.5})

    config = PIControllerConfig(
        dt=1 / 100,
        torque_scale=0.1,
        torque_limit_nm=10.0,
        joints=(KNEE_TORQUE, ANKLE_TORQUE),
    )
    # Provide a schema so the runtime uses the DataWrangler path
    schema = InputSchema(
        name="pi_inputs",
        signals=(
            SchemaSignal(name="knee_flexion_angle_ipsi_rad"),
            SchemaSignal(name="knee_flexion_velocity_ipsi_rad_s"),
            SchemaSignal(name="ankle_dorsiflexion_angle_ipsi_rad"),
            SchemaSignal(name="ankle_dorsiflexion_velocity_ipsi_rad_s"),
            SchemaSignal(name="vertical_grf_ipsi_N", required=False),
        ),
    )
    controller = PIController(
        config=config,
        torque_model=torque_model,
        input_schema=schema,
    )
    routes = (
        SignalRoute(name="knee_flexion_angle_ipsi_rad", provider="imu_mock"),
        SignalRoute(name="knee_flexion_velocity_ipsi_rad_s", provider="imu_mock"),
        SignalRoute(name="ankle_dorsiflexion_angle_ipsi_rad", provider="imu_mock"),
        SignalRoute(name="ankle_dorsiflexion_velocity_ipsi_rad_s", provider="imu_mock"),
        SignalRoute(name="vertical_grf_ipsi_N", provider="vertical_grf_mock"),
    )
    loop = RuntimeLoop(
        RuntimeLoopConfig(frequency_hz=100.0),
        sensors={"imu_mock": mock_imu, "vertical_grf_mock": mock_grf},
        actuator=mock_actuator,
        controller=controller,
        signal_routes=routes,
    )

    with loop:
        for _ in loop.run(duration_s=0.02):
            pass

    assert mock_actuator.commanded, "Expected commands to be recorded"
    last_command = mock_actuator.last_command()
    assert last_command is not None
    assert last_command.torques_nm[KNEE_TORQUE] == pytest.approx(0.05, rel=1e-6)
    assert last_command.torques_nm[ANKLE_TORQUE] == pytest.approx(-0.05, rel=1e-6)


def test_mock_imu_stale_fallback() -> None:
    """Ensure IMU fallback strategy returns zeroed samples and records diagnostics."""
    config = BaseIMUConfig(max_stale_samples=1, fault_strategy="fallback")
    mock = MockIMU(samples=None, loop=True, config_override=config)
    mock.read()
    mock.read()
    diag = mock.diagnostics
    assert diag.hz_estimate is not None
    fallback = mock._handle_sample(None, fresh=False)
    assert all(value == 0.0 for value in fallback.values.values())


def test_mock_imu_stale_raise() -> None:
    """Verify IMU stale data raises when configured for strict handling."""
    config = BaseIMUConfig(max_stale_samples=1, fault_strategy="raise")
    mock = MockIMU(samples=None, loop=True, config_override=config)
    mock.read()
    mock.read()
    with pytest.raises(IMUStaleDataError):
        mock._handle_sample(None, fresh=False)


def test_mock_imu_stale_warn_pass_through() -> None:
    """Warn strategy should preserve latest sample while recording diagnostics."""
    config = BaseIMUConfig(max_stale_samples=1, fault_strategy="warn")
    mock = MockIMU(samples=None, loop=True, config_override=config)
    fresh_sample = mock.read()
    mock.read()
    returned = mock._handle_sample(fresh_sample, fresh=False)
    diag = mock.diagnostics
    assert returned is fresh_sample
    assert diag.stale_samples == 0
    assert diag.total_samples >= 1
