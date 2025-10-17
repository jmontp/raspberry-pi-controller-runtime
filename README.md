# Raspberry Pi Controller Runtime

`raspberry-pi-controller-runtime`: Make it easy to swap between models, sensors, and actuators so we can evaluate different exoskeleton control stacks without rewriting the runtime each time.

## Repository goals
- Provide consistent abstractions for IMUs, ground reaction force sensing, and future modalities.
- Offer a reusable feed-forward controller skeleton derived from `energy_shaping_ML/pi_code/jose_pi_controller.py` with dependency injection for sensors/actuators.
- Standardise torque model packaging per `torque-modeling/docs/raspberry_pi.md`.
- Ship deployment scripts, documentation, and CI guardrails for rapid iteration.

## Getting started
```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .[dev]
```

Then explore `examples/demo_stand.py` for a simple runtime loop wiring IMU + FSR data into the controller.

## Profile-driven configuration
The runtime now supports declarative hardware/controller manifests. Define your desired hardware in a YAML profile (see `src/rpc_runtime/config/default.yaml` for a mock example), then load and instantiate components programmatically:

```python
from pathlib import Path

from rpc_runtime.config import load_runtime_profile
from rpc_runtime.config import build_runtime_components
from rpc_runtime.runtime.loop import RuntimeLoop, RuntimeLoopConfig

profile = load_runtime_profile(Path("src/rpc_runtime/config/hardware_config.yaml"))
components = build_runtime_components(profile)

loop = RuntimeLoop(
    RuntimeLoopConfig(frequency_hz=500),
    sensors=components.sensors,
    actuator=components.actuator,
    controller=components.controller,
    signal_routes=profile.signal_routes,
)
```

The controller manifest guarantees that sensors emit a consistent feature schema across training and evaluation. Required channels trigger validation errors when the profile omits compatible hardware, while optional channels fall back to zeroed defaults.

Read `docs/architecture.md` for module structure and `src/rpc_runtime/config/default.yaml` for a profile example.

## Canonical signal naming
Feature IDs follow the canonical definitions in `LocoHub/src/locohub/feature_constants.py`. Profiles, runtime schemas, and tool scripts reference names such as `knee_flexion_angle_ipsi_rad` and `vertical_grf_ipsi_N`. When replaying recorded sessions (see `scripts/replay_session.py`), ensure CSV/log columns use these canonical identifiers; legacy column names like `knee_angle` are no longer accepted.

## Repository layout
See `docs/architecture.md` for a module-level overview and `AGENTS.md` for contributor expectations.
