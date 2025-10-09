"""Actuator abstractions for the controller runtime."""

from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import Mapping


@dataclass(slots=True)
class TorqueCommand:
    timestamp: float
    torques_nm: Mapping[str, float]


class ActuatorError(RuntimeError):
    pass


class BaseActuator(abc.ABC):
    """Common interface for actuators driven by the runtime."""

    def __enter__(self) -> "BaseActuator":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    @abc.abstractmethod
    def start(self) -> None:
        """Initialise communication with the actuator bus."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Shutdown connections and put hardware in a safe state."""

    @abc.abstractmethod
    def apply(self, command: TorqueCommand) -> None:
        """Apply a torque command mapping joint names to Nm."""

    def fault_if_needed(self) -> None:
        """Optional diagnostic hook executed each control tick."""
