# Actuator Adapter Responsibilities

Actuator implementations (e.g., `OSLActuator`, hardware mocks) should follow the
contract defined by `BaseActuator`. When contributing a new adapter:

1. **Configuration**
   - Accept a `BaseActuatorConfig` (or subclass) and pass it to
     `BaseActuator.__init__`. This enables joint validation, torque limit
     enforcement, and diagnostics tracking.
   - Override `_validate_config` only when additional modality-specific checks
     are required (e.g., custom limit semantics).

2. **Lifecycle hooks**
   - Implement `start()` to initialise any hardware links or background threads.
   - Implement `stop()` to ensure the device returns to a safe state even on
     error paths.

3. **Applying commands**
   - Do not override `apply()` directly. Instead, implement `_apply_command`
     which receives a validated (and optionally clamped) `TorqueCommand`. The
     base class handles joint membership checks, torque limits, and diagnostic
     bookkeeping.

4. **Diagnostics**
   - Consume `self.diagnostics` to expose telemetry such as last command,
     command counts, clamping events, and faults. The base class already
     populates this data—adapters only need to raise `ActuatorError` when
     transport-level faults occur.

5. **Fault handling**
   - Override `fault_if_needed()` for periodic health checks specific to the
     hardware. Raise `ActuatorError` when a fault is detected so the runtime can
     react.

Following these guidelines keeps actuator adapters aligned with the shared
staleness, validation, and telemetry behaviour expected by the controller
runtime.
