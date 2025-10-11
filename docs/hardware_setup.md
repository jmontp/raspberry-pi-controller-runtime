# Hardware Bring-up

1. **IMUs** — Mount devices following the joint order defined in `config/default.yaml`. Run `CalibrationRoutine` to zero offsets before starting the loop.
   - Software adapters expose their connection details via `BaseIMUConfig`. Override `port_map` (placeholders default to `/dev/ttyIMU_<segment>`) when USB device paths differ from your deployed symlinks. Configure `max_stale_samples`, `max_stale_time_s`, and `fault_strategy` to control how the runtime reacts to stale or missing IMU packets. The MicroStrain implementation requires the HBK MSCL Python bindings (`mscl`).
2. **FSRs** — Pair the Bluetooth FSR board (see `sensors/grf/fsr.py`). Update the MAC address in the configuration profile.
   - GRF adapters use `BaseVerticalGRFConfig` for channel naming and staleness policies. Set `fault_strategy` to `fallback` if you prefer neutral force samples when packets drop.
3. **Actuators** — Connect ActPack devices and verify they appear under `/dev/ttyActPack*`. The runtime sets current-control mode during `OSLActuator.start()`.
   - All actuator adapters inherit from `BaseActuator`, which validates joint membership and optional torque limits supplied via `BaseActuatorConfig`. If your hardware requires torque clamping rather than raising on limit violations, set `clamp_torque=True`. Diagnostics such as `command_count`, `last_command`, and any `ActuatorError` messages can be retrieved from `actuator.diagnostics` to aid in bring-up and fault analysis.
4. **Runtime Loop** — Launch using a profile-specific script (see `examples/demo_stand.py`). Monitor logs for fault flags reported by `OSLActuator.fault_if_needed()`.

## Calibration & Diagnostics Checklist

1. Run `CalibrationRoutine` prior to entering the control loop. This calls `zero()` on any IMU/GRF adapters that expose the hook.
2. Verify sampling stability by inspecting `sensor.diagnostics.recent_sample_periods` to ensure the hardware is streaming at the expected rate.
3. During early bring-up, enable sensor `fault_strategy="warn"` to keep the controller running while capturing stale packet warnings.
4. After commanding torques, check `actuator.diagnostics` for `command_count`, `last_command_clamped`, and `last_fault` to confirm limits and safety policies are respected.
