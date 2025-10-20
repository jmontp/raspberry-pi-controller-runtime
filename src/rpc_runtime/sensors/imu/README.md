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
