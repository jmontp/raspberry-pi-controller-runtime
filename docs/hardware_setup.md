# Hardware Bring-up

1. **IMUs** — Mount devices following the joint order defined in `config/default.yaml`. Run `CalibrationRoutine` to zero offsets before starting the loop.
   - Software adapters expose their connection details via `BaseIMUConfig`. Override `port_map` (placeholders default to `/dev/ttyIMU_<segment>`) when USB device paths differ from your deployed symlinks. The MicroStrain implementation requires the HBK MSCL Python bindings (`mscl`).
2. **FSRs** — Pair the Bluetooth FSR board (see `sensors/grf/fsr.py`). Update the MAC address in the configuration profile.
3. **Actuators** — Connect ActPack devices and verify they appear under `/dev/ttyActPack*`. The runtime sets current-control mode during `OSLActuator.start()`.
4. **Runtime Loop** — Launch using a profile-specific script (see `examples/demo_stand.py`). Monitor logs for fault flags reported by `OSLActuator.fault_if_needed()`.
