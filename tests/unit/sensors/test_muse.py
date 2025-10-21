import math

import pytest

from rpc_runtime.sensors.imu import MuseDeviceConfig, MuseIMU, MuseIMUConfig


class _StubSample:
    def __init__(self, pitch_deg: float, gyro_deg_s: float) -> None:
        self.euler = [0.0, pitch_deg, 0.0]
        self.gyr = [0.0, gyro_deg_s, 0.0]


def _make_imu() -> MuseIMU:
    config = MuseIMUConfig(
        port_map={
            "trunk_sagittal_angle_rad": "trunk",
            "trunk_sagittal_velocity_rad_s": "trunk",
            "thigh_sagittal_angle_contra_rad": "left",
            "thigh_sagittal_velocity_contra_rad_s": "left",
        },
        devices={
            "trunk": MuseDeviceConfig(name="muse_trunk"),
            "left": MuseDeviceConfig(name="muse_left"),
        },
        calibration_samples=0,
    )
    imu = MuseIMU(config)
    imu._started = True  # bypass BLE startup for unit testing
    imu._calibration_done = True
    return imu


def test_handle_muse_sample_generates_canonical_values():
    imu = _make_imu()

    imu._handle_muse_sample("trunk", _StubSample(pitch_deg=10.0, gyro_deg_s=90.0))
    imu._handle_muse_sample("left", _StubSample(pitch_deg=5.0, gyro_deg_s=60.0))

    sample_trunk = imu.read()
    assert imu.last_sample_fresh is True
    assert math.isclose(sample_trunk.values["trunk_sagittal_angle_rad"], math.radians(10.0), rel_tol=1e-6)
    assert math.isclose(sample_trunk.values["trunk_sagittal_velocity_rad_s"], math.radians(90.0), rel_tol=1e-6)

    sample_left = imu.read()
    assert imu.last_sample_fresh is True
    assert math.isclose(
        sample_left.values["thigh_sagittal_angle_contra_rad"],
        -math.radians(5.0),
        rel_tol=1e-6,
    )
    assert math.isclose(
        sample_left.values["thigh_sagittal_velocity_contra_rad_s"],
        -math.radians(60.0),
        rel_tol=1e-6,
    )

    stale = imu.read()
    assert imu.last_sample_fresh is False
    assert math.isclose(stale.values["trunk_sagittal_angle_rad"], math.radians(10.0), rel_tol=1e-6)
    assert math.isclose(stale.values["thigh_sagittal_angle_contra_rad"], -math.radians(5.0), rel_tol=1e-6)


def test_reset_updates_zero_offsets():
    imu = _make_imu()
    imu._handle_muse_sample("trunk", _StubSample(pitch_deg=12.0, gyro_deg_s=0.0))
    _ = imu.read()
    imu.reset()
    imu._handle_muse_sample("trunk", _StubSample(pitch_deg=14.0, gyro_deg_s=30.0))
    sample = imu.read()
    assert math.isclose(sample.values["trunk_sagittal_angle_rad"], math.radians(2.0), rel_tol=1e-6)


def test_invalid_device_reference_raises():
    config = MuseIMUConfig(
        port_map={"trunk_sagittal_angle_rad": "trunk"},
        devices={"other": MuseDeviceConfig(name="muse_other")},
    )
    with pytest.raises(ValueError):
        MuseIMU(config)
