"""Mock actuator for deterministic controller testing."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from .base import ActuatorError, BaseActuator, BaseActuatorConfig, TorqueCommand


@dataclass(slots=True)
class MockActuator(BaseActuator):
    """Spoof actuator that records applied torques for assertions."""

    commanded: List[TorqueCommand] = field(default_factory=list)
    fail_on_apply: bool = False
    config_override: BaseActuatorConfig | None = None

    def __post_init__(self) -> None:
        """Initialise the base actuator with the supplied configuration."""
        BaseActuator.__init__(self, self.config_override)

    def start(self) -> None:  # pragma: no cover - trivial
        """Prepare the mock actuator (no-op)."""
        return None

    def stop(self) -> None:  # pragma: no cover - trivial
        """Tear down the mock actuator (no-op)."""
        return None

    def _apply_command(self, command: TorqueCommand) -> None:
        """Record the torque command or raise if `fail_on_apply` is set."""
        if self.fail_on_apply:
            raise ActuatorError("MockActuator configured to fail")
        self.commanded.append(command)

    def last_command(self) -> TorqueCommand | None:
        """Return the most recent torque command if one was captured."""
        return self.commanded[-1] if self.commanded else None
