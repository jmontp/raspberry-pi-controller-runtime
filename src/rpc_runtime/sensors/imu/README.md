# IMU Adapter Responsibilities

Every concrete IMU implementation in this package must follow the contract
defined by `BaseIMU`. When adding a new adapter, make sure the following
requirements are satisfied:

See `sensors/README.md` for the shared configuration, lifecycle, and staleness
handling expectations inherited from `BaseSensor`. IMU adapters add the
following responsibilities:

1. **Joint/segment mapping**
   - Ensure joint angle computations only reference available segments. The base
     class already validates this, but adapters should keep the mapping clear
     when translating raw sensor data into controller joints.

2. **Calibration/reset hooks**
   - Override `reset()` (or expose `zero()`) when the hardware supports explicit
     realignment to controller coordinates.

3. **Port mapping**
   - Ensure `BaseIMUConfig.port_map` covers every segment required by the
     adapter. This mapping bridges logical segments to physical transport
     identifiers or device handles.

Following these guidelines keeps IMU adapters aligned with the shared sensor
contract while documenting the modality-specific expectations.
