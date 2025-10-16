"""Tests for sensor diagnostics helpers and faulty mock sensor."""

from __future__ import annotations

from typing import Iterable

from rpc_runtime.sensors.imu.base import BaseIMUConfig
from rpc_runtime.sensors.imu.mock import MockIMU
from rpc_runtime.sensors.imu.mock_faulty import MockFaultyIMU


def _collect_angles(samples: Iterable[float]) -> tuple[float, ...]:
    return tuple(samples)


def test_sensor_histogram_reports_rates() -> None:
    imu = MockIMU(loop=True)
    for _ in range(20):
        imu.read()
    diag = imu.diagnostics
    edges, counts = imu.sample_rate_histogram(bins=5)
    assert diag.recent_sample_periods
    assert diag.recent_sample_rates
    assert edges
    assert sum(counts) == len(diag.recent_sample_rates)


def test_faulty_imu_injects_frame_drop() -> None:
    imu = MockFaultyIMU(
        loop=True,
        config_override=BaseIMUConfig(fault_strategy="fallback", max_stale_samples=100),
    )
    imu.start()
    imu.force_drop_frames(3)
    dropped = 0
    produced = 0
    for _ in range(6):
        sample = imu.read()
        if all(value == 0.0 for value in sample.joint_angles_rad):
            dropped += 1
        else:
            produced += 1
    assert dropped >= 3
    assert produced >= 1


def test_faulty_imu_trigger_dropout() -> None:
    imu = MockFaultyIMU(
        loop=True,
        seed=1,
        config_override=BaseIMUConfig(fault_strategy="fallback", max_stale_samples=100),
    )
    imu.start()
    imu.trigger_dropout(frames=3)
    dropped = 0
    for _ in range(3):
        sample = imu.read()
        assert all(value == 0.0 for value in sample.joint_angles_rad)
        dropped += 1
    imu.restore()
    sample = imu.read()
    assert any(abs(value) > 0.0 for value in sample.joint_angles_rad)
    assert dropped == 3
