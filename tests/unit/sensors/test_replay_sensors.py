from __future__ import annotations

from pathlib import Path

import pandas as pd
import pytest

from rpc_runtime.sensors.imu.replay import ReplayIMU
from rpc_runtime.sensors.grf.replay import ReplayVerticalGRF
from rpc_runtime.sensors.imu.base import BaseIMUConfig
from rpc_runtime.sensors.grf.base import BaseVerticalGRFConfig


def _write_csv(tmp_path: Path, name: str, frame: pd.DataFrame) -> Path:
    path = tmp_path / name
    frame.to_csv(path, index=False)
    return path


def test_replay_imu_reads_filtered_sequence(tmp_path):
    frame = pd.DataFrame(
        {
            "subject": ["S1", "S1", "S2"],
            "task": ["walk", "run", "walk"],
            "hip_flexion_angle_ipsi_rad": [0.1, 0.2, 0.3],
            "knee_flexion_angle_ipsi_rad": [0.4, 0.5, 0.6],
            "ankle_dorsiflexion_angle_ipsi_rad": [0.7, 0.8, 0.9],
            "hip_flexion_angle_contra_rad": [1.0, 1.1, 1.2],
            "knee_flexion_angle_contra_rad": [1.3, 1.4, 1.5],
            "ankle_dorsiflexion_angle_contra_rad": [1.6, 1.7, 1.8],
            "hip_flexion_velocity_ipsi_rad_s": [0.01, 0.02, 0.03],
            "knee_flexion_velocity_ipsi_rad_s": [0.04, 0.05, 0.06],
            "ankle_dorsiflexion_velocity_ipsi_rad_s": [0.07, 0.08, 0.09],
            "hip_flexion_velocity_contra_rad_s": [0.10, 0.11, 0.12],
            "knee_flexion_velocity_contra_rad_s": [0.13, 0.14, 0.15],
            "ankle_dorsiflexion_velocity_contra_rad_s": [0.16, 0.17, 0.18],
            "trunk_sagittal_angle_rad": [0.0, 0.0, 0.0],
            "trunk_sagittal_velocity_rad_s": [0.0, 0.0, 0.0],
            "thigh_sagittal_angle_ipsi_rad": [0.21, 0.22, 0.23],
            "shank_sagittal_angle_ipsi_rad": [0.24, 0.25, 0.26],
            "foot_sagittal_angle_ipsi_rad": [0.27, 0.28, 0.29],
            "thigh_sagittal_angle_contra_rad": [0.31, 0.32, 0.33],
            "shank_sagittal_angle_contra_rad": [0.34, 0.35, 0.36],
            "foot_sagittal_angle_contra_rad": [0.37, 0.38, 0.39],
            "thigh_sagittal_velocity_ipsi_rad_s": [0.41, 0.42, 0.43],
            "shank_sagittal_velocity_ipsi_rad_s": [0.44, 0.45, 0.46],
            "foot_sagittal_velocity_ipsi_rad_s": [0.47, 0.48, 0.49],
            "thigh_sagittal_velocity_contra_rad_s": [0.51, 0.52, 0.53],
            "shank_sagittal_velocity_contra_rad_s": [0.54, 0.55, 0.56],
            "foot_sagittal_velocity_contra_rad_s": [0.57, 0.58, 0.59],
        }
    )
    path = _write_csv(tmp_path, "imu.csv", frame)

    imu = ReplayIMU(
        path=str(path),
        subject="S1",
        tasks=["walk"],
        max_samples=1,
        loop=False,
        dt=0.01,
        config_override=BaseIMUConfig(),
    )
    imu.probe()
    imu.start()

    sample = imu.read()
    assert sample.joint_angles_rad[0] == pytest.approx(0.1)
    assert sample.joint_angles_rad[3] == pytest.approx(1.0)
    assert sample.joint_velocities_rad_s[1] == pytest.approx(0.04)

    with pytest.raises(RuntimeError):
        imu.read()


def test_replay_imu_probe_fails_for_missing_file(tmp_path):
    missing = tmp_path / "missing.csv"
    imu = ReplayIMU(path=str(missing))
    with pytest.raises(FileNotFoundError):
        imu.probe()


def test_replay_grf_respects_mass_and_loop(tmp_path):
    frame = pd.DataFrame(
        {
            "subject": ["S1", "S1"],
            "task": ["walk", "walk"],
            "vertical_grf_ipsi_BW": [0.5, 0.6],
            "vertical_grf_contra_BW": [0.2, 0.3],
        }
    )
    path = _write_csv(tmp_path, "grf.csv", frame)
    grf = ReplayVerticalGRF(
        path=str(path),
        subject="S1",
        loop=True,
        dt=0.01,
        body_mass_kg=70.0,
        config_override=BaseVerticalGRFConfig(channel_names=("vertical_grf_ipsi_N", "vertical_grf_contra_N")),
    )
    grf.probe()
    grf.start()

    sample = grf.read()
    scale = 70.0 * 9.80665
    assert sample.forces_newton[0] == pytest.approx(0.5 * scale)
    assert sample.forces_newton[1] == pytest.approx(0.2 * scale)

    sample_looped = grf.read()
    assert sample_looped.forces_newton[0] == pytest.approx(0.6 * scale)

    # Should loop without raising after exhausting max_samples
    for _ in range(2):
        grf.read()
