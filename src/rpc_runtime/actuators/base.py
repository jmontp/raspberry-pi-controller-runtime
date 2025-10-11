"""Actuator abstractions for the controller runtime."""

from __future__ import annotations

import abc
import logging
from dataclasses import dataclass
from typing import Mapping

LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class TorqueCommand:
    """Container describing the torques to apply at the current tick.

    Args:
        timestamp: Monotonic time in seconds when the command was generated.
        torques_nm: Mapping of joint name to commanded torque in newton-metres.
    """

    timestamp: float
    torques_nm: Mapping[str, float]


@dataclass(slots=True)
class ActuatorDiagnostics:
    """Runtime diagnostics captured by :class:`BaseActuator`.

    Attributes:
        last_timestamp: Timestamp of the most recent successful command.
        last_command: Mapping of joint name to the last commanded torque.
        command_count: Number of successful calls to :meth:`BaseActuator.apply`.
        rejected_commands: Number of commands rejected due to validation errors.
        last_fault: Stringified representation of the last actuator fault.
        last_unknown_joints: Tuple of joints that failed validation, if any.
        last_command_clamped: Flag indicating the last command was torque-limited.
    """

    last_timestamp: float | None = None
    last_command: dict[str, float] | None = None
    command_count: int = 0
    rejected_commands: int = 0
    last_fault: str | None = None
    last_unknown_joints: tuple[str, ...] | None = None
    last_command_clamped: bool = False


@dataclass(slots=True)
class BaseActuatorConfig:
    """Configuration shared across actuator implementations.

    Args:
        joint_names: Tuple containing the joints controlled by the actuator.
        torque_limits_nm: Optional mapping of per-joint absolute torque limits.
        clamp_torque: Clip applied torques to the configured limits instead of
            raising an error when :attr:`torque_limits_nm` is provided.
    """

    joint_names: tuple[str, ...] = ()
    torque_limits_nm: Mapping[str, float] | None = None
    clamp_torque: bool = False


class ActuatorError(RuntimeError):
    """Raised when the actuator detects a fault or cannot execute a command."""


class BaseActuator(abc.ABC):
    """Common interface for actuators driven by the runtime."""

    def __init__(self, config: BaseActuatorConfig | None = None) -> None:
        """Normalise configuration and initialise diagnostics.

        Args:
            config: Optional configuration describing joints and limits. If
                omitted, the actuator does not enforce joint membership or
                torque limits.
        """
        cfg = config or BaseActuatorConfig()
        self._config = self._validate_config(cfg)
        self._diagnostics = ActuatorDiagnostics()

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
        """
        self.stop()

    @property
    def diagnostics(self) -> ActuatorDiagnostics:
        """Return live diagnostics describing actuator command history."""
        return self._diagnostics

    @property
    def config(self) -> BaseActuatorConfig:
        """Return the normalised configuration supplied to the actuator."""
        return self._config

    @abc.abstractmethod
    def start(self) -> None:
        """Initialise communication with the actuator bus."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Shutdown connections and put hardware in a safe state."""

    def apply(self, command: TorqueCommand) -> None:
        """Apply a torque command after enforcing configuration constraints.

        Args:
            command: The torque command to transmit to the hardware.

        Raises:
            ActuatorError: When validation fails or the concrete actuator
                reports a fault.
        """
        validated, clamped = self._prepare_command(command)
        try:
            self._apply_command(validated)
        except ActuatorError as exc:
            self._diagnostics.last_fault = str(exc)
            self._diagnostics.rejected_commands += 1
            raise
        except Exception as exc:  # pragma: no cover - unexpected actuator faults
            self._diagnostics.last_fault = str(exc)
            raise
        else:
            self._diagnostics.last_timestamp = validated.timestamp
            self._diagnostics.last_command = dict(validated.torques_nm)
            self._diagnostics.command_count += 1
            self._diagnostics.last_fault = None
            self._diagnostics.last_unknown_joints = None
            self._diagnostics.last_command_clamped = clamped

    @abc.abstractmethod
    def _apply_command(self, command: TorqueCommand) -> None:
        """Transmit the validated torque command to the underlying hardware."""

    def _prepare_command(self, command: TorqueCommand) -> tuple[TorqueCommand, bool]:
        """Validate joint membership and enforce optional torque limits.

        Args:
            command: Candidate torque command to validate.

        Returns:
            tuple[TorqueCommand, bool]: A tuple containing the validated command
            (potentially clamped) and a flag indicating whether clamping
            occurred.

        Raises:
            ActuatorError: If the command references unknown joints or exceeds
                configured torque limits when clamping is disabled.
        """
        mapping = {joint: float(torque) for joint, torque in command.torques_nm.items()}
        joint_names = set(self._config.joint_names)
        if joint_names:
            unknown = tuple(sorted(set(mapping) - joint_names))
            if unknown:
                message = f"Torque command contains unknown joints: {unknown}"
                self._diagnostics.rejected_commands += 1
                self._diagnostics.last_fault = message
                self._diagnostics.last_unknown_joints = unknown
                raise ActuatorError(message)
        limits = self._config.torque_limits_nm
        clamped = False
        if limits:
            for joint, torque in list(mapping.items()):
                limit = limits.get(joint)
                if limit is None:
                    continue
                limit = float(limit)
                if abs(torque) > limit:
                    if self._config.clamp_torque:
                        clamped = True
                        mapping[joint] = max(-limit, min(limit, torque))
                    else:
                        message = (
                            f"Torque {torque:.3f} Nm exceeds limit {limit:.3f} Nm "
                            f"for joint '{joint}'"
                        )
                        self._diagnostics.rejected_commands += 1
                        self._diagnostics.last_fault = message
                        self._diagnostics.last_unknown_joints = None
                        raise ActuatorError(message)
            if clamped:
                LOGGER.warning("Clamped torque command to configured limits: %s", mapping)
        sanitized = TorqueCommand(timestamp=command.timestamp, torques_nm=mapping)
        return sanitized, clamped

    @classmethod
    def _validate_config(cls, config: BaseActuatorConfig) -> BaseActuatorConfig:
        """Validate and normalise a :class:`BaseActuatorConfig`."""
        joint_names = tuple(config.joint_names)
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("BaseActuatorConfig.joint_names contains duplicates")

        limits = None
        if config.torque_limits_nm is not None:
            limits = {str(name): float(limit) for name, limit in config.torque_limits_nm.items()}
            if joint_names:
                unsupported = set(limits) - set(joint_names)
                if unsupported:
                    raise ValueError(
                        "Torque limits provided for joints outside the configured joint set: "
                        f"{sorted(unsupported)}"
                    )

        config.joint_names = joint_names
        config.torque_limits_nm = limits
        config.clamp_torque = bool(config.clamp_torque)
        return config

    def fault_if_needed(self) -> None:
        """Optional diagnostic hook executed each control tick."""
        return None
