# GRF Adapter Responsibilities

Implementations of `BaseVerticalGRF` should adhere to the following contract:

See `sensors/README.md` for the shared configuration, lifecycle, and staleness
handling expectations inherited from `BaseSensor`. GRF adapters add the
following responsibilities:

1. **Zeroing/baselines**
   - Override `zero()` when the hardware supports in-place baseline updates.

2. **Channel conventions**
   - Validate that `BaseVerticalGRFConfig.channel_names` aligns with the
     adapter’s supported channel list (`CHANNEL_NAMES`) and document any sign or
     scaling conventions via `FORCE_CONVENTIONS`.

3. **Hardware-specific fallback**
   - Implement `_handle_sample` wrappers that build meaningful fallback samples
     (e.g., zero force vectors) when the upstream transport returns stale data.

Following these guidelines keeps GRF adapters aligned with the shared sensor
contract while documenting the modality-specific expectations.
