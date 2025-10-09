# Agent Guidelines

## Coding standards
- Target Python 3.11.
- Follow the [PEP 8](https://peps.python.org/pep-0008/) style guide; prefer type hints for public APIs.
- Keep modules importable on Raspberry Pi: avoid heavy dependencies on the critical path.
- Sensor/actuator interfaces must expose async-safe methods when they perform I/O.
- Document assumptions and units directly in docstrings; controllers should state expected signal ordering.

## Linting & formatting
- Managed with `ruff` (format + lint). Run `ruff check --fix` and `ruff format` before opening a PR.
- Use `mypy` for static type checking; opt-in gradual types when wrapping third-party libs.
- CI runs `pytest`, `ruff`, and `mypy`. Keep tests hermetic; mark hardware dependencies with `@pytest.mark.hardware`.

## Git workflow
- Create topic branches per feature.
- Keep commits small and descriptive; reference issues when applicable.
- All PRs require review; reviewers focus on safety-critical paths (control loop, sensor drivers).
- Add integration tests or notebooks showcasing behaviour when touching controller math.

## Review checklist
- [ ] Tests cover new behaviour or failure modes.
- [ ] Controller loop remains real-time safe (no blocking I/O or allocations in tight loop).
- [ ] Sensor/actuator adapters handle disconnects gracefully.
- [ ] Docs updated when APIs change.
