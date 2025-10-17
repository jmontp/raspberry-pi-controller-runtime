# Agent Guidelines

This document captures everything Codex (and other agents) need to contribute to
`raspberry-pi-controller-runtime`. Treat it as the single source of truth when
starting a new chat or automation session.

---

## 1. Repository setup
- Python version: **3.11** (ensure virtual environments respect this).
- Install in editable mode with dev extras:
  ```bash
  python -m venv .venv
  source .venv/bin/activate
  pip install -e .[dev]
  ```
- Hardware-only dependencies (`read_imu`, `osl`, Bluetooth FSR thread) are
  optional during local development; code must guard imports.
- `SoftRealtimeLoop` from NeuroLocoMiddleware is not required for unit tests,
  but production code should support it via the scheduler abstraction.

## 2. Coding standards
- Follow [PEP 8](https://peps.python.org/pep-0008/) plus our overrides:
  - Type annotate all public functions and class attributes.
  - Prefer composition via dependency injection; avoid singletons.
  - Avoid blocking I/O in control loops; wrap hardware interaction in adapters.
  - Keep modules importable on Raspberry Pi: no heavy desktop-only packages on
    hot paths.
- Controllers and sensor adapters must describe coordinate frames, units, and
  expectations in docstrings.
- IMU adapters accept typed configs (`BaseIMUConfig` and subclasses) exposing
  joint/segment ordering and `port_map` overrides. Staleness handling is
  configured via `max_stale_samples`, `max_stale_time_s`, and `fault_strategy`
  (`raise`, `fallback`, `warn`). The MicroStrain driver uses the MSCL Python
  bindings (`mscl`); ensure the library is installed on the Pi when deploying
  hardware builds.
  - See `src/rpc_runtime/sensors/imu/README.md` for adapter implementation
    guidelines.
- GRF adapters follow the same pattern: use `BaseVerticalGRFConfig`, honour
  staleness strategies, and implement the responsibilities described in
  `src/rpc_runtime/sensors/grf/README.md`.
- Use mocks/adapters for hardware in tests (`MockIMU`, `MockVerticalGRF`,
  `MockActuator`, `MockTorqueModel`) instead of stubbing low-level libraries.

## 3. Docstring standards
- Every public module, class, function, and method has a docstring.
- Use **Google-style** sections exactly as shown:
  ```python
  def example(arg_one: float, arg_two: int) -> float:
      """Short summary line.

      Args:
          arg_one: Description with units where applicable.
          arg_two: Description including valid ranges.

      Returns:
          float: Meaning of the returned value and units.

      Raises:
          ValueError: Circumstances that trigger the error.
      """
  ```
- Keep summaries to one sentence; include hardware context (frames, units,
  resource ownership, side effects).
- For context managers or async APIs, document what resources are acquired and
  how they are released.
- When behaviour is hardware-dependent, document graceful degradation paths
  (e.g., raising `RuntimeError` when no samples are ready).

## 4. Linting & formatting
- Tools: `ruff` for lint + format, `mypy` for type checking.
- Before committing, run:
  ```bash
  ruff check --fix
  ruff format
  mypy src
  pytest
  ```
- `ruff` configuration lives in `pyproject.toml`; line length is 100 chars.
- Mark optional hardware imports with `# pragma: no cover` or `# type: ignore`
  as needed, but prefer feature flags/configuration over unconditional import
  errors.

## 5. Testing philosophy
- Unit tests use the mock hardware stack; never require real devices.
- Put fast tests under `tests/unit/`; hardware/integration tests should be
  clearly marked with `@pytest.mark.hardware`.
- Prefer deterministic fixtures; add mock generators for new sensor types.
- When introducing new controller logic, add regression tests that assert on
  torque outputs given fixed inputs.

## 6. Runtime & scheduler usage
- `RuntimeLoop` accepts a `scheduler_factory`:
  - Use `SimpleScheduler` for CI/local tests.
  - Wrap NeuroLocoMiddleware’s `SoftRealtimeLoop` via `SoftRealtimeScheduler`
    for deployment.
- The runtime always uses `DataWrangler` to build canonical features each tick.
  Controllers implement `compute_torque(features, *, timestamp)` and remain
  agnostic to hardware specifics.
- If sensors support alignment, call `imu.as_resettable()?.zero()` or `imu.reset()`
  prior to starting the loop.

## 7. Commit & branching strategy
- Create a feature branch per task; keep commits focused.
- Write descriptive commit messages in the imperative mood, e.g.,
  `Add mock hardware stack for feed-forward controller`.
- Never commit interpreter artefacts (`__pycache__`, `.pytest_cache`, etc.);
  `.gitignore` already covers these.
- When working alongside user changes, avoid undoing their work without
  explicit approval.

## 8. Review checklist (use this before submitting PRs)
- [ ] Tests cover new behaviour or regressions and pass locally.
- [ ] `ruff check`, `ruff format`, and `mypy` succeed.
- [ ] Control loop remains real-time safe (no blocking I/O or large
      allocations in tight loops).
- [ ] Sensor/actuator adapters gracefully handle disconnected hardware.
- [ ] Docstrings and docs updated for new/changed APIs.
- [ ] Configuration changes documented in `docs/architecture.md` or inline
      YAML comments if necessary.

## 9. Documentation & examples
- Update `docs/` whenever architecture or configuration changes.
- Key references:
  - `docs/architecture.md` — high-level module layout and data flow.
  - `docs/torque_models.md` — integrating exported ONNX/TorchScript bundles.
  - `docs/hardware_setup.md` — IMU/FSR bring-up and deployment notes.
- Keep `examples/` runnable against the mock stack; hardware-specific examples
  should clearly note required dependencies.
- Provide quick-start notes in `README.md` when altering setup steps.

## 10. Communication expectations
- Surface assumptions, TODOs, and risks in PR descriptions.
- When blocking on hardware or external dependencies, document fallback plans.
- For breaking changes, add migration notes in commit messages and docs.

Keeping this guidance up to date ensures Codex can bootstrap new sessions
without additional context. Update this file whenever the project workflow
changes.***
