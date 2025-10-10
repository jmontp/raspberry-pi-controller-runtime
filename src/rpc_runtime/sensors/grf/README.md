# GRF Adapter Responsibilities

Implementations of `BaseVerticalGRF` should adhere to the following contract:

1. **Initialisation**
   - Accept a `BaseVerticalGRFConfig` (or subclass) and pass it to
     `BaseVerticalGRF.__init__` to validate channel names and staleness policy.

2. **Lifecycle hooks**
   - Implement `start()` to initialise hardware connections or threads.
   - Implement `stop()` to release resources and join worker threads.

3. **Reading samples**
   - When fresh data is available, build a `VerticalGRFSample` and return
     `self._handle_sample(sample, fresh=True)`.
   - If no new data arrives, call `self._handle_sample(None, fresh=False)` so the
     base class can apply the configured fault strategy (`raise`, `fallback`,
     `warn`).

4. **Zeroing/baselines**
   - Override `zero()` when the hardware supports in-place baseline updates.

5. **Error handling**
   - Catch recoverable hardware errors and log them; re-raise fatal errors so the
     runtime loop can respond appropriately.

Following these guidelines ensures all GRF adapters integrate cleanly with the
runtime’s staleness handling and schema expectations.
