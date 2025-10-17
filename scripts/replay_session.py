#!/usr/bin/env python3
"""Replay recorded sensor data through the controller for offline analysis.

CSV Expectations
----------------
The input CSV must expose canonical LocoHub feature names that match the
runtime configuration:

Required columns:
    - `timestamp`
    - `knee_flexion_angle_ipsi_rad`
    - `knee_flexion_velocity_ipsi_rad_s`
    - `ankle_dorsiflexion_angle_ipsi_rad`
    - `ankle_dorsiflexion_velocity_ipsi_rad_s`
    - `vertical_grf_ipsi_N`

Optional columns:
    - `knee_flexion_angle_desired_ipsi_rad`
    - `ankle_dorsiflexion_angle_desired_ipsi_rad`

Additional columns are ignored.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pandas as pd

from rpc_runtime.controllers.pi_controller import PIController, PIControllerConfig
from rpc_runtime.controllers.torque_models.onnx_runtime import ONNXTorqueModel
from rpc_runtime.sensors.imu.base import IMUSample

REQUIRED_COLUMNS = {
    "timestamp",
    "knee_flexion_angle_ipsi_rad",
    "knee_flexion_velocity_ipsi_rad_s",
    "ankle_dorsiflexion_angle_ipsi_rad",
    "ankle_dorsiflexion_velocity_ipsi_rad_s",
    "vertical_grf_ipsi_N",
}

OPTIONAL_COLUMNS = {
    "knee_flexion_angle_desired_ipsi_rad",
    "ankle_dorsiflexion_angle_desired_ipsi_rad",
}


def main() -> None:
    """Replay a CSV of features through the feed-forward controller and print sample output."""
    parser = argparse.ArgumentParser(description="Replay recorded CSV through the controller")
    parser.add_argument("csv", type=Path)
    parser.add_argument("bundle", type=Path)
    args = parser.parse_args()

    data = pd.read_csv(args.csv)
    missing = REQUIRED_COLUMNS - set(data.columns)
    if missing:
        raise ValueError(
            "CSV is missing required canonical columns: "
            + ", ".join(sorted(missing))
        )

    config = PIControllerConfig(
        dt=1 / 500,
        torque_scale=0.1,
        torque_limit_nm=25,
        joints=(KNEE_TORQUE, ANKLE_TORQUE),
    )
    model = ONNXTorqueModel(args.bundle)
    controller = PIController(config, model)

    outputs = []
    for row in data.itertuples(index=False):
        imu_sample = IMUSample(
            timestamp=float(row.timestamp),
            joint_angles_rad=(
                float(row.knee_flexion_angle_ipsi_rad),
                float(row.ankle_dorsiflexion_angle_ipsi_rad),
            ),
            joint_velocities_rad_s=(
                float(row.knee_flexion_velocity_ipsi_rad_s),
                float(row.ankle_dorsiflexion_velocity_ipsi_rad_s),
            ),
            segment_angles_rad=(
                float(row.thigh_sagittal_angle_ipsi_rad),
                float(row.shank_sagittal_angle_ipsi_rad),
                float(row.foot_sagittal_angle_ipsi_rad),
            ),
            segment_velocities_rad_s=(
                float(row.thigh_sagittal_velocity_ipsi_rad_s),
                float(row.shank_sagittal_velocity_ipsi_rad_s),
                float(row.foot_sagittal_velocity_ipsi_rad_s),
            ),
        )
        # GRF is consumed directly as a canonical feature below.
        features = {
            "knee_flexion_angle_ipsi_rad": float(row.knee_flexion_angle_ipsi_rad),
            "knee_flexion_velocity_ipsi_rad_s": float(row.knee_flexion_velocity_ipsi_rad_s),
            "ankle_dorsiflexion_angle_ipsi_rad": float(row.ankle_dorsiflexion_angle_ipsi_rad),
            "ankle_dorsiflexion_velocity_ipsi_rad_s": float(
                row.ankle_dorsiflexion_velocity_ipsi_rad_s
            ),
            "vertical_grf_ipsi_N": float(row.vertical_grf_ipsi_N),
        }
        if hasattr(row, "knee_flexion_angle_desired_ipsi_rad"):
            features["knee_flexion_angle_desired_ipsi_rad"] = float(
                row.knee_flexion_angle_desired_ipsi_rad
            )
        if hasattr(row, "ankle_dorsiflexion_angle_desired_ipsi_rad"):
            features["ankle_dorsiflexion_angle_desired_ipsi_rad"] = float(
                row.ankle_dorsiflexion_angle_desired_ipsi_rad
            )
        command = controller.compute_torque(features, timestamp=imu_sample.timestamp)
        outputs.append({"timestamp": imu_sample.timestamp, **command.torques_nm})

    print(json.dumps(outputs[:5], indent=2))


if __name__ == "__main__":
    main()
KNEE_TORQUE = "knee_flexion_moment_ipsi_Nm"
ANKLE_TORQUE = "ankle_dorsiflexion_moment_ipsi_Nm"
