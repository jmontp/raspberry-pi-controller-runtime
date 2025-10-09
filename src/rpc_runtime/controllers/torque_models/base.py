"""Base interface for runtime torque models."""

from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import Iterable


@dataclass(slots=True)
class TorqueModelInput:
    feature_names: tuple[str, ...]


class TorqueModel(abc.ABC):
    @abc.abstractmethod
    def run(self, features: dict[str, float]) -> dict[str, float]:
        """Run inference on a single feature vector."""

    def warmup(self) -> None:
        """Optional warmup before entering the real-time loop."""

    @abc.abstractmethod
    def expected_features(self) -> Iterable[str]:
        """Return ordered feature names for validation."""
