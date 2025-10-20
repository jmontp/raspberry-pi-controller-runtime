"""Tests verifying IMU-derived signals are exposed through the runtime."""

from __future__ import annotations

import pytest

from rpc_runtime.actuators.base import TorqueCommand
from rpc_runtime.actuators.mock import MockActuator
from rpc_runtime.config.models import SignalRoute
from rpc_runtime.runtime.loop import RuntimeLoop, RuntimeLoopConfig
from rpc_runtime.runtime.wrangler import InputSchema, SchemaSignal
from rpc_runtime.sensors.imu.base import BaseIMUConfig, IMUSample
from rpc_runtime.sensors.imu.mock import MockIMU


class _MockKneeAngleController:
    """Controller stub that records the requested knee flexion angle."""

    def __init__(self, *, schema: InputSchema) -> None:
        """Store the schema and prepare joint outputs."""
        self._input_schema = schema
        self._joints: tuple[str, ...] = ("knee_flexion_moment_ipsi_Nm",)
        self.recorded_angles: list[float] = []

    def reset(self) -> None:
        """Clear recorded angles before a new runtime session."""
        self.recorded_angles.clear()

    def compute_torque(self, features: dict[str, float], *, timestamp: float) -> TorqueCommand:
        """Capture the knee flexion angle and emit zero torque."""
        angle = features["knee_flexion_angle_ipsi_rad"]
        self.recorded_angles.append(angle)
        torques = dict.fromkeys(self._joints, 0.0)
        return TorqueCommand(timestamp=timestamp, torques_nm=torques)

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Return the actuator joint ordering used by this controller."""
        return self._joints


def test_mock_imu_accepts_canonical_port_names() -> None:
    """Ensure IMU configs accept canonical feature names in port maps."""
    config = BaseIMUConfig(
        port_map={
            "shank_sagittal_angle_ipsi_rad": "/dev/null",
            "foot_sagittal_angle_ipsi_rad": "/dev/null",
        },
    )
    imu = MockIMU(config_override=config, loop=True)
    assert "shank_sagittal_angle_ipsi_rad" in imu.port_map
    assert imu.port_map["shank_sagittal_angle_ipsi_rad"] == "/dev/null"


def test_runtime_loop_derives_knee_angle_from_segments() -> None:
    """Ensure the runtime derives knee flexion from thigh and shank segments."""
    thigh_angle = 0.60
    shank_angle = 0.25
    values = {
        "trunk_sagittal_angle_rad": 0.05,
        "thigh_sagittal_angle_ipsi_rad": thigh_angle,
        "shank_sagittal_angle_ipsi_rad": shank_angle,
        "foot_sagittal_angle_ipsi_rad": 0.10,
        "thigh_sagittal_angle_contra_rad": -0.30,
        "shank_sagittal_angle_contra_rad": -0.45,
        "foot_sagittal_angle_contra_rad": -0.15,
        "trunk_sagittal_velocity_rad_s": 0.0,
        "thigh_sagittal_velocity_ipsi_rad_s": 0.0,
        "shank_sagittal_velocity_ipsi_rad_s": 0.0,
        "foot_sagittal_velocity_ipsi_rad_s": 0.0,
        "thigh_sagittal_velocity_contra_rad_s": 0.0,
        "shank_sagittal_velocity_contra_rad_s": 0.0,
        "foot_sagittal_velocity_contra_rad_s": 0.0,
    }
    expected_knee_angle = thigh_angle - shank_angle
    imu_sample = IMUSample(timestamp=1.0, values=values)
    imu = MockIMU(samples=[imu_sample], loop=False)
    schema = InputSchema(
        name="knee_angle_only",
        signals=(SchemaSignal(name="knee_flexion_angle_ipsi_rad"),),
    )
    controller = _MockKneeAngleController(schema=schema)
    actuator = MockActuator()
    routes = (SignalRoute(name="knee_flexion_angle_ipsi_rad", provider="imu_mock"),)
    loop = RuntimeLoop(
        RuntimeLoopConfig(frequency_hz=100.0),
        sensors={"imu_mock": imu},
        actuator=actuator,
        controller=controller,
        signal_routes=routes,
    )

    with loop:
        for _ in loop.run(duration_s=0.0):
            pass

    assert controller.recorded_angles, "Controller should have observed at least one angle"
    assert controller.recorded_angles[0] == pytest.approx(expected_knee_angle)
    last_command = actuator.last_command()
    assert last_command is not None
    assert last_command.torques_nm["knee_flexion_moment_ipsi_Nm"] == pytest.approx(0.0)
