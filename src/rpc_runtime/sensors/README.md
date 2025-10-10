# Base Sensor Responsibilities

Common functionality for every sensor adapter lives in `BaseSensor`
(`src/rpc_runtime/sensors/base.py`). When implementing a new modality (or
refactoring an existing one), keep the following expectations in mind:

1. **Configuration**
   - Accept a `BaseSensorConfig` (or subclass) and pass it to
     `BaseSensor.__init__`. Call `super()._validate_config()` from any override to
     reuse the generic fault-strategy checks (`raise`, `fallback`, `warn`) and
     diagnostics window validation.
   - Access the normalised configuration through `self.config`. Subclasses should
     avoid keeping duplicated copies of the same data.

2. **Lifecycle**
   - `BaseSensor` implements context manager hooks (`__enter__`, `__exit__`) that
     call `start()`/`stop()`; subclasses should simply implement those abstract
     methods.

3. **Sampling & staleness**
   - All adapters must route every read through `_handle_sample(sample, fresh=...)`.
     The subclass provides a small wrapper to convert modality-specific fallback
     data into the shared `SensorDiagnostics` flow.
   - When a new packet arrives, pass `fresh=True`. If `read()` finds no update,
     call `_handle_sample(None, fresh=False)` so the configured staleness policy
     can raise, warn, or serve fallback data.

4. **Diagnostics**
   - `BaseSensor` automatically tracks `last_dt`, instantaneous `hz_estimate`, and
     a rolling `recent_sample_periods` tuple. Adapters do not need bespoke logging
     for cadence metrics; consumers can read `sensor.diagnostics`.

5. **Logging & fault strategy**
   - When `fault_strategy` is `fallback` or `warn`, the base class emits a warning
     via the module logger. Subclasses should log only modality-specific hardware
     failures; routine stale handling is centralised already.

Following this contract keeps all sensor adapters aligned with the shared
staleness management and telemetry expected by the runtime controller.
