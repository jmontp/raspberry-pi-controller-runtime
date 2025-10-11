"""Simple CLI to monitor sensor and actuator diagnostics using mock devices."""

from __future__ import annotations

import time

from rpc_runtime.actuators.base import BaseActuatorConfig, TorqueCommand
from rpc_runtime.actuators.mock import MockActuator
from rpc_runtime.sensors.grf.base import BaseVerticalGRFConfig
from rpc_runtime.sensors.grf.mock import MockVerticalGRF
from rpc_runtime.sensors.imu.base import BaseIMUConfig
from rpc_runtime.sensors.imu.mock import MockIMU


def main(duration_s: float = 1.0, frequency_hz: float = 20.0) -> None:
    """Stream mock data and print diagnostics for demonstration purposes."""
    imu = MockIMU(config_override=BaseIMUConfig(max_stale_samples=3, fault_strategy="warn"))
    grf = MockVerticalGRF(
        config_override=BaseVerticalGRFConfig(max_stale_samples=3, fault_strategy="warn")
    )
    actuator = MockActuator(
        config_override=BaseActuatorConfig(joint_names=("knee", "ankle"), clamp_torque=True)
    )

    dt = 1.0 / frequency_hz
    start = time.monotonic()
    while time.monotonic() - start < duration_s:
        imu.read()
        grf.read()
        command = TorqueCommand(timestamp=time.monotonic(), torques_nm={"knee": 1.0, "ankle": -1.0})
        actuator.apply(command)
        print("IMU diagnostics:", imu.diagnostics)
        print("GRF diagnostics:", grf.diagnostics)
        print("Actuator diagnostics:", actuator.diagnostics)
        print("-" * 40)
        time.sleep(dt)


if __name__ == "__main__":
    main()
