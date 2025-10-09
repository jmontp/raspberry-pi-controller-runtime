import pytest

from rpc_runtime.actuators.mock import MockActuator
from rpc_runtime.controllers.pi_controller import PIController, PIControllerConfig, PIControllerGains
from rpc_runtime.controllers.torque_models.mock import MockTorqueModel
from rpc_runtime.pipelines.runtime_loop import RuntimeLoop, RuntimeLoopConfig
from rpc_runtime.sensors.grf.mock import MockVerticalGRF
from rpc_runtime.sensors.imu.base import IMUSample
from rpc_runtime.sensors.imu.mock import MockIMU


def test_controller_with_mock_devices():
    imu_sample = IMUSample(
        timestamp=0.0,
        joint_angles_rad=(0.1, -0.05),
        joint_velocities_rad_s=(0.0, 0.0),
        segment_angles_rad=(0.0, 0.0, 0.0),
        segment_velocities_rad_s=(0.0, 0.0, 0.0),
    )
    mock_imu = MockIMU(samples=[imu_sample], loop=True)
    mock_grf = MockVerticalGRF()
    mock_actuator = MockActuator()
    torque_model = MockTorqueModel(outputs={"knee": 0.5, "ankle": -0.5})

    config = PIControllerConfig(
        dt=1 / 100,
        torque_scale=0.1,
        torque_limit_nm=10.0,
        joints=("knee", "ankle"),
    )
    gains = PIControllerGains(kp={"knee": 0.0, "ankle": 0.0}, ki={"knee": 0.0, "ankle": 0.0})
    controller = PIController(config=config, gains=gains, torque_model=torque_model)
    loop = RuntimeLoop(
        RuntimeLoopConfig(frequency_hz=100.0),
        imu=mock_imu,
        actuator=mock_actuator,
        controller=controller,
        vertical_grf=mock_grf,
    )

    with loop:
        for _ in loop.run(duration_s=0.02):
            pass

    assert mock_actuator.commanded, "Expected commands to be recorded"
    last_command = mock_actuator.last_command()
    assert last_command is not None
    assert last_command.torques_nm["knee"] == pytest.approx(0.05, rel=1e-6)
    assert last_command.torques_nm["ankle"] == pytest.approx(-0.05, rel=1e-6)
