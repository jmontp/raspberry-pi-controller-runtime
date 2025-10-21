# IMU Adapter Responsibilities

Every concrete IMU implementation in this package must follow the contract
defined by `BaseIMU`. When adding a new adapter, make sure the following
requirements are satisfied:

See `sensors/README.md` for the shared configuration, lifecycle, and staleness
handling expectations inherited from `BaseSensor`. IMU adapters add the
following responsibilities:

1. **Canonical feature mapping**
   - Hardware samples must be expressed directly in the canonical feature
     namespace (e.g., `thigh_sagittal_angle_ipsi_rad`,
     `thigh_sagittal_velocity_ipsi_rad_s`). `BaseIMU` handles downstream joint
     derivations, so adapters should only populate the features their hardware
     produces.
   - Use `feature_semantics()` or `split_canonical_feature_name()` to parse a
     canonical identifier into side/segment/quantity metadata when routing data
     to device channels.

2. **Calibration/reset hooks**
   - Override `reset()` (or expose `zero()`) when the hardware supports explicit
     realignment to controller coordinates.
   - Offsets must be tracked per canonical feature so that calibration affects
     both angles and velocities consistently.

3. **Port mapping**
   - `BaseIMUConfig.port_map` now lists canonical feature names instead of
     segment identifiers. The adapter is responsible for coalescing duplicate
     entries that map to the same physical device (e.g., an angle/velocity pair
     that shares a serial port).
   - When the config omits ports, provide sensible defaults or raise a helpful
     error describing the required canonical features.

Following these guidelines keeps IMU adapters aligned with the shared sensor
contract while documenting the modality-specific expectations.

## Muse Bluetooth IMU

The `MuseIMU` adapter streams BLE telemetry from Muse inertial units shipped
with the reference `muse_api` repository. It requires `bleak` and the
`Muse_Utils`, `Muse_HW`, and `Muse_Data` modules on `PYTHONPATH`.

```python
from rpc_runtime.sensors.imu import MuseIMU, MuseIMUConfig, MuseDeviceConfig

config = MuseIMUConfig(
    port_map={
        "trunk_sagittal_angle_rad": "trunk",
        "trunk_sagittal_velocity_rad_s": "trunk",
    },
    devices={
        "trunk": MuseDeviceConfig(name="muse_trunk", adapter="hci0"),
    },
)
imu = MuseIMU(config)
```

Each entry in `port_map` declares which device supplies a canonical feature.
Device keys must appear in `MuseIMUConfig.devices`, which specifies BLE names
or addresses plus optional per-device stream overrides. By default the driver
starts IMU+orientation streaming at 200 Hz and automatically calibrates zero
offsets from the first few samples. Use `reset()` at runtime to capture a new
neutral pose.
