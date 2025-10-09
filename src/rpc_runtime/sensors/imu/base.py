"""IMU abstractions used by the controller runtime."""

from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import Protocol


@dataclass(slots=True)
class IMUSample:
    """Single IMU reading expressed in controller joint/segment coordinates."""

    timestamp: float
    joint_angles_rad: tuple[float, ...]
    joint_velocities_rad_s: tuple[float, ...]
    segment_angles_rad: tuple[float, ...]
    segment_velocities_rad_s: tuple[float, ...]

    @property
    def joint_count(self) -> int:
        return len(self.joint_angles_rad)


class IMUResettable(Protocol):
    def zero(self) -> None:
        """Reset internal offsets to align with controller coordinates."""


class BaseIMU(abc.ABC):
    """Abstract interface for IMU sources.

    Concrete implementations translate hardware frames into the canonical
    joint and segment representation expected by the controller.
    """

    supports_batch: bool = False

    def __enter__(self) -> "BaseIMU":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    @abc.abstractmethod
    def start(self) -> None:
        """Initialise connections and start streaming data."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Tear down hardware resources."""

    @abc.abstractmethod
    def read(self) -> IMUSample:
        """Return the latest controller-frame IMU sample."""

    def reset(self) -> None:
        """Optional method to zero orientation and velocity estimates."""

    def as_resettable(self) -> IMUResettable | None:
        return self if isinstance(self, IMUResettable) else None
