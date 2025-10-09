# Raspberry Pi Controller Runtime

`raspberry-pi-controller-runtime` consolidates our Raspberry Pi exoskeleton controller runtime: modular sensor abstractions, actuator interfaces powered by `osl`, torque model loaders, and orchestration utilities to run closed-loop control on embedded Linux.

## Repository goals
- Provide consistent abstractions for IMUs, ground reaction force sensing, and future modalities.
- Offer a reusable PI controller skeleton derived from `energy_shaping_ML/pi_code/jose_pi_controller.py` with dependency injection for sensors/actuators.
- Standardise torque model packaging per `torque-modeling/docs/raspberry_pi.md`.
- Ship deployment scripts, documentation, and CI guardrails for rapid iteration.

## Getting started
```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .[dev]
```

Then explore `examples/demo_stand.py` for a simple runtime loop wiring IMU + FSR data into the controller.

## Repository layout
See `docs/architecture.md` for a module-level overview and `Agents.md` for contributor expectations.
