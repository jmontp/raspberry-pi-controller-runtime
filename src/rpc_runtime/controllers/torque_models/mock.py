"""Mock torque model for controller unit tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable

from .base import TorqueModel


@dataclass(slots=True)
class MockTorqueModel(TorqueModel):
    outputs: Dict[str, float] = field(default_factory=dict)

    def run(self, features: dict[str, float]) -> dict[str, float]:
        return dict(self.outputs)

    def expected_features(self) -> Iterable[str]:
        return ()

