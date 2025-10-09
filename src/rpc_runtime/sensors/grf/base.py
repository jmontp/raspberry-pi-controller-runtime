"""Abstractions for vertical ground reaction force sensing."""

from __future__ import annotations

import abc
from dataclasses import dataclass


@dataclass(slots=True)
class VerticalGRFSample:
    """Vertical ground reaction force measurement.

    Args:
        timestamp: Monotonic capture time in seconds.
        forces_newton: Tuple of vertical forces per sensing channel in newtons.
    """

    timestamp: float
    forces_newton: tuple[float, ...]

    @property
    def channels(self) -> int:
        """Number of force channels contained in the sample."""
        return len(self.forces_newton)


class BaseVerticalGRF(abc.ABC):
    def __enter__(self) -> "BaseVerticalGRF":
        """Enter context manager and start the GRF sensor."""
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Leave context manager and stop the GRF sensor."""
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
        return None
