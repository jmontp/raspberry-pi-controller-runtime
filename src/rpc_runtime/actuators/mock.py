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
        return None

    def stop(self) -> None:  # pragma: no cover - trivial
        return None

    def apply(self, command: TorqueCommand) -> None:
        if self.fail_on_apply:
            raise RuntimeError("MockActuator configured to fail")
        self.commanded.append(command)

    def last_command(self) -> TorqueCommand | None:
        return self.commanded[-1] if self.commanded else None
