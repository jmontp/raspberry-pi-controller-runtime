"""Mock torque model for controller unit tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable

from .base import TorqueModel


@dataclass(slots=True)
class MockTorqueModel(TorqueModel):
    """Return predefined torque outputs for any feature vector."""

    outputs: Dict[str, float] = field(default_factory=dict)

    def run(self, features: dict[str, float]) -> dict[str, float]:
        """Return a copy of the configured torque outputs."""
        return dict(self.outputs)

    def expected_features(self) -> Iterable[str]:
        """Expose the expected feature names (empty by default)."""
        return ()
