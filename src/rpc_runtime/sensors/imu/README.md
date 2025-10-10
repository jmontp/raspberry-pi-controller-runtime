# IMU Adapter Responsibilities

Every concrete IMU implementation in this package must follow the contract
defined by `BaseIMU`. When adding a new adapter, make sure the following
requirements are satisfied:

1. **Initialisation**
   - Accept a `BaseIMUConfig` (or subclass) and pass it to
     `BaseIMU.__init__`. This validates joint/segment names, port mappings, and
     staleness policy.

2. **Lifecycle hooks**
   - Implement `start()` to open hardware connections, perform calibration, and
     populate any internal buffers.
   - Implement `stop()` to release all resources even on error paths.

3. **Reading samples**
   - `read()` must supply the latest `IMUSample`. If no fresh data is available,
     call `self._handle_sample(None, fresh=False)` so the base class can enforce
     the configured staleness policy.
   - When a packet is received, build an `IMUSample` and return
     `self._handle_sample(sample, fresh=True)` so counters and timestamps stay
     in sync.

4. **Error handling**
   - Wrap hardware reads in `try/except` and log recoverable errors. When a
     hard failure occurs, let the exception surface (it will be caught by the
     runtime loop).

5. **Joint/segment mapping**
   - Ensure joint angle computations only reference available segments. The base
     class already validates this, but adapters should keep the mapping clear
     when translating raw sensor data into controller joints.

Following these guidelines keeps every IMU adapter compatible with the runtime
staleness/fallback logic and the controller’s expectations.
