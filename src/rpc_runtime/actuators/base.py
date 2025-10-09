"""Actuator abstractions for the controller runtime."""

from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import Mapping


@dataclass(slots=True)
class TorqueCommand:
    """Container describing the torques to apply at the current tick.

    Args:
        timestamp: Monotonic time in seconds when the command was generated.
        torques_nm: Mapping of joint name to commanded torque in newton-metres.
    """

    timestamp: float
    torques_nm: Mapping[str, float]


class ActuatorError(RuntimeError):
    """Raised when the actuator detects a fault or cannot execute a command."""


class BaseActuator(abc.ABC):
    """Common interface for actuators driven by the runtime."""

    def __enter__(self) -> "BaseActuator":
        """Bind resources when entering a context manager.

        Returns:
            BaseActuator: The actuator instance, ready for use.
        """
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Release resources when leaving a context manager.

        Args:
            exc_type: Exception type raised within the context, if any.
            exc: Exception instance raised within the context, if any.
            tb: Traceback associated with the raised exception, if any.

        Returns:
            None
        """
        self.stop()

    @abc.abstractmethod
    def start(self) -> None:
        """Initialise communication with the actuator bus.

        Returns:
            None
        """

    @abc.abstractmethod
    def stop(self) -> None:
        """Shutdown connections and put hardware in a safe state.

        Returns:
            None
        """

    @abc.abstractmethod
    def apply(self, command: TorqueCommand) -> None:
        """Apply a torque command mapping joint names to Nm.

        Args:
            command: The torque command to transmit to the hardware.

        Returns:
            None
        """

    def fault_if_needed(self) -> None:
        """Optional diagnostic hook executed each control tick.

        Returns:
            None
        """
        return None
