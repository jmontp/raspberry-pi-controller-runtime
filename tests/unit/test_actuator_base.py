"""Unit tests for BaseActuator validation and diagnostics."""

from __future__ import annotations

import pytest

from rpc_runtime.actuators.base import (
    ActuatorDiagnostics,
    ActuatorError,
    BaseActuator,
    BaseActuatorConfig,
    TorqueCommand,
)


class _RecordingActuator(BaseActuator):
    """Test double that records applied torque commands."""

    def __init__(self, config: BaseActuatorConfig | None = None) -> None:
        super().__init__(config)
        self.applied: list[TorqueCommand] = []

    def start(self) -> None:  # pragma: no cover - no setup required
        return None

    def stop(self) -> None:  # pragma: no cover - no teardown required
        return None

    def _apply_command(self, command: TorqueCommand) -> None:
        self.applied.append(command)


class _FailingActuator(BaseActuator):
    """Actuator that raises an error when applying commands."""

    def __init__(self) -> None:
        super().__init__(BaseActuatorConfig(joint_names=("knee",)))

    def start(self) -> None:  # pragma: no cover - no-op
        return None

    def stop(self) -> None:  # pragma: no cover - no-op
        return None

    def _apply_command(self, command: TorqueCommand) -> None:
        raise ActuatorError("simulated fault")


@pytest.fixture
def actuator() -> _RecordingActuator:
    """Provide an actuator fixture with a single joint and torque limit."""
    config = BaseActuatorConfig(joint_names=("knee",), torque_limits_nm={"knee": 10.0})
    return _RecordingActuator(config=config)


def test_apply_updates_diagnostics(actuator: _RecordingActuator) -> None:
    """Applying a valid command should update diagnostics counters."""
    command = TorqueCommand(timestamp=1.0, torques_nm={"knee": 5.0})
    actuator.apply(command)
    diag = actuator.diagnostics
    assert isinstance(diag, ActuatorDiagnostics)
    assert diag.command_count == 1
    assert diag.last_timestamp == 1.0
    assert diag.last_command == {"knee": 5.0}
    assert not diag.last_command_clamped


def test_apply_rejects_unknown_joint(actuator: _RecordingActuator) -> None:
    """Commands referencing unknown joints should raise an error."""
    command = TorqueCommand(timestamp=0.5, torques_nm={"ankle": 1.0})
    with pytest.raises(ActuatorError):
        actuator.apply(command)
    diag = actuator.diagnostics
    assert diag.rejected_commands == 1
    assert diag.last_unknown_joints == ("ankle",)


def test_apply_rejects_limits_when_clamp_disabled(actuator: _RecordingActuator) -> None:
    """Exceeding configured limits without clamping should raise errors."""
    command = TorqueCommand(timestamp=0.5, torques_nm={"knee": 12.0})
    with pytest.raises(ActuatorError):
        actuator.apply(command)
    diag = actuator.diagnostics
    assert diag.rejected_commands == 1
    assert "exceeds" in (diag.last_fault or "")


def test_apply_clamps_when_enabled() -> None:
    """Clamping should saturate torque values within configured limits."""
    config = BaseActuatorConfig(
        joint_names=("knee",),
        torque_limits_nm={"knee": 4.0},
        clamp_torque=True,
    )
    actuator = _RecordingActuator(config=config)
    command = TorqueCommand(timestamp=2.0, torques_nm={"knee": 10.0})
    actuator.apply(command)
    diag = actuator.diagnostics
    assert diag.command_count == 1
    assert diag.last_command == {"knee": 4.0}
    assert diag.last_command_clamped


def test_apply_records_faults() -> None:
    """ActuatorError during apply should populate diagnostics."""
    actuator = _FailingActuator()
    command = TorqueCommand(timestamp=3.0, torques_nm={"knee": 1.0})
    with pytest.raises(ActuatorError):
        actuator.apply(command)
    diag = actuator.diagnostics
    assert diag.rejected_commands == 1
    assert diag.last_fault == "simulated fault"
