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

## Bluetooth FSR

`BluetoothFSR` wraps the Nordic UART Service exposed by the lab-built BLE insoles.
Provide the peripheral MAC address and optional sampling rate (25/100/200 Hz).
The adapter speaks the same command set as the original MBLUE scripts and keeps
the zero offset configurable via `zero()`.

## Wired FSR

`WiredFSR` re-implements the USB/serial ActiSense reader that streams 39-byte
frames at 230400 baud. Configure the serial port, optional clip direction
(`forward` or `reverse`), and whether raw counts should be inverted (the legacy
implementation used 1/value).

Both adapters expose the shared `channel_names` tuple and honour the staleness
contract described above.
