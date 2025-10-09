"""Mock actuator for deterministic controller testing."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from .base import BaseActuator, TorqueCommand


@dataclass(slots=True)
class MockActuator(BaseActuator):
    """Spoof actuator that records applied torques for assertions."""

    commanded: List[TorqueCommand] = field(default_factory=list)
    fail_on_apply: bool = False

    def start(self) -> None:  # pragma: no cover - trivial
        """Prepare the mock actuator (no-op).

        Returns:
            None
        """
        return None

    def stop(self) -> None:  # pragma: no cover - trivial
        """Tear down the mock actuator (no-op).

        Returns:
            None
        """
        return None

    def apply(self, command: TorqueCommand) -> None:
        """Record the torque command or raise if `fail_on_apply` is set.

        Args:
            command: The torque command captured for later assertions.

        Raises:
            RuntimeError: If `fail_on_apply` is enabled to simulate a fault.

        Returns:
            None
        """
        if self.fail_on_apply:
            raise RuntimeError("MockActuator configured to fail")
        self.commanded.append(command)

    def last_command(self) -> TorqueCommand | None:
        """Return the most recent torque command if one was captured."""
        return self.commanded[-1] if self.commanded else None
