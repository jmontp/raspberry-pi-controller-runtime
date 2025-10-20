"""Mock torque model for controller unit tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, Tuple

from .base import TorqueModel


@dataclass(slots=True)
class MockTorqueModel(TorqueModel):
    """Return predefined torque outputs for any feature vector."""

    outputs: Dict[str, float] = field(default_factory=dict)
    inputs: Tuple[str, ...] | Iterable[str] | None = ()

    def __post_init__(self) -> None:
        """Normalise configured inputs to a tuple for downstream consumers."""
        if self.inputs is None:
            self.inputs = ()
        elif not isinstance(self.inputs, tuple):
            self.inputs = tuple(self.inputs)

    def run(self, features: dict[str, float]) -> dict[str, float]:
        """Return a copy of the configured torque outputs."""
        return dict(self.outputs)

    def expected_features(self) -> Iterable[str]:
        """Expose the configured feature names requested by this model."""
        return self.inputs
