"""Abstractions for vertical ground reaction force sensing."""

from __future__ import annotations

import abc
from dataclasses import dataclass


@dataclass(slots=True)
class VerticalGRFSample:
    timestamp: float
    forces_newton: tuple[float, ...]

    @property
    def channels(self) -> int:
        return len(self.forces_newton)


class BaseVerticalGRF(abc.ABC):
    def __enter__(self) -> "BaseVerticalGRF":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    @abc.abstractmethod
    def start(self) -> None:
        """Initialise the sensor and begin streaming data."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Tear down resources."""

    @abc.abstractmethod
    def read(self) -> VerticalGRFSample:
        """Return the latest vertical GRF sample."""

    def zero(self) -> None:
        """Optional re-zero hook."""
