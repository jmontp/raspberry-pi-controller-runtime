#!/usr/bin/env python3
"""Replay recorded sensor data through the controller for offline analysis."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pandas as pd

from rpc_runtime.controllers.pi_controller import (
    PIController,
    PIControllerConfig,
    PIControllerGains,
)
from rpc_runtime.controllers.torque_models.onnx_runtime import ONNXTorqueModel
from rpc_runtime.sensors.combinators import ControlInputs
from rpc_runtime.sensors.grf.base import VerticalGRFSample
from rpc_runtime.sensors.imu.base import IMUSample


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay recorded CSV through the controller")
    parser.add_argument("csv", type=Path)
    parser.add_argument("bundle", type=Path)
    args = parser.parse_args()

    data = pd.read_csv(args.csv)
    config = PIControllerConfig(
        dt=1 / 500,
        torque_scale=0.1,
        torque_limit_nm=25,
        joints=("knee", "ankle"),
    )
    gains = PIControllerGains(
        kp={"knee": 120, "ankle": 80},
        ki={"knee": 5, "ankle": 3},
    )
    model = ONNXTorqueModel(args.bundle)
    controller = PIController(config, gains, model)

    outputs = []
    for row in data.itertuples(index=False):
        imu_sample = IMUSample(
            timestamp=float(row.timestamp),
            joint_angles_rad=(float(row.knee_angle), float(row.ankle_angle)),
            joint_velocities_rad_s=(float(row.knee_velocity), float(row.ankle_velocity)),
            segment_angles_rad=(
                float(row.thigh_angle),
                float(row.shank_angle),
                float(row.foot_angle),
            ),
            segment_velocities_rad_s=(
                float(row.thigh_velocity),
                float(row.shank_velocity),
                float(row.foot_velocity),
            ),
        )
        grf_sample = VerticalGRFSample(
            timestamp=float(row.timestamp),
            forces_newton=(float(row.grf),),
        )
        inputs = ControlInputs(imu=imu_sample, vertical_grf=grf_sample)
        command = controller.tick(inputs)
        outputs.append({"timestamp": imu_sample.timestamp, **command.torques_nm})

    print(json.dumps(outputs[:5], indent=2))


if __name__ == "__main__":
    main()
