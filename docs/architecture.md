# Architecture Overview

The runtime splits responsibilities into four layers:

1. **Sensors** — `rpc_runtime.sensors` holds abstract base classes for IMUs and vertical GRF sources. Hardware adapters (e.g. Bluetooth FSRs) translate vendor payloads into controller-ready measurements. `BaseIMU` publishes canonical joint/segment names and angle conventions, and `BaseIMUConfig` lets adapters expose connection metadata (e.g. `port_map`) so controllers consume a consistent schema.
2. **Controllers** — `rpc_runtime.controllers` implements the PI controller skeleton inspired by `energy_shaping_ML/pi_code/jose_pi_controller.py`, with replaceable torque models (`onnxruntime` or `torchscript`).
3. **Actuators** — `rpc_runtime.actuators` wraps the `osl` API so joints are referenced by semantic names. Safety hooks surface hardware faults to the pipeline.
4. **Pipelines** — `rpc_runtime.pipelines.RuntimeLoop` orchestrates the read→compute→actuate loop, while `rpc_runtime.pipelines.scheduler` abstracts timing (simple loop for CI, NeuroLocoMiddleware’s `SoftRealtimeLoop` for hardware). Calibration helpers live alongside the scheduler layer.

Configuration lives in `src/rpc_runtime/config/` using YAML profiles. See `examples/demo_stand.py` for basic wiring.
