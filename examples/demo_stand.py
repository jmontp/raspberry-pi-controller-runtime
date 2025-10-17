"""Minimal demo wiring IMU + FSR + controller."""

from __future__ import annotations

from pathlib import Path

from rpc_runtime.actuators.osl_actuator import OSLActuator, OSLJoint, OSLLegConfig
from rpc_runtime.controllers.pi_controller import PIController, PIControllerConfig
from rpc_runtime.controllers.torque_models.onnx_runtime import ONNXTorqueModel
from rpc_runtime.runtime.loop import RuntimeLoop, RuntimeLoopConfig
from rpc_runtime.sensors.grf.fsr import BluetoothFSR, BluetoothFSRConfig
from rpc_runtime.sensors.imu.microstrain_3dm_gx5 import (
    Microstrain3DMGX5Config,
    Microstrain3DMGX5IMU,
)


def main() -> None:
    """Demonstrate wiring the runtime loop with hardware-backed adapters."""
    bundle = Path("/path/to/export/onnx")
    config = PIControllerConfig(
        dt=1 / 500,
        torque_scale=0.1,
        torque_limit_nm=25,
        joints=("knee", "ankle"),
    )
    torque_model = ONNXTorqueModel(bundle)
    imu = Microstrain3DMGX5IMU(
        Microstrain3DMGX5Config(
            joint_names=("knee", "ankle"),
            segment_names=("thigh", "shank", "foot"),
        )
    )
    fsr = BluetoothFSR(BluetoothFSRConfig(address="E8:EA:71:E8:37:D1"))
    actuator = OSLActuator(
        OSLLegConfig(
            controller_hz=500,
            joints=(
                OSLJoint(name="knee", gear_ratio=9.0, port="/dev/ttyActPackRightKnee"),
                OSLJoint(name="ankle", gear_ratio=9.0, port="/dev/ttyActPackRightAnkle"),
            ),
        )
    )
    loop = RuntimeLoop(
        RuntimeLoopConfig(frequency_hz=500),
        imu=imu,
        actuator=actuator,
        controller=PIController(config, torque_model),
        vertical_grf=fsr,
    )

    with loop:
        for _ in loop.run(duration_s=5.0):
            pass


if __name__ == "__main__":
    main()
