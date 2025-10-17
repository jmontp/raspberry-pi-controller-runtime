"""Adapter for the OpenSourceLeg actuator stack."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Dict, Iterable

from .base import ActuatorError, BaseActuator, BaseActuatorConfig, TorqueCommand

try:  # pragma: no cover - hardware dependency
    from opensourceleg.osl import OpenSourceLeg
except ImportError:  # noqa: F401 pragma: no cover - optional
    OpenSourceLeg = None  # type: ignore[assignment]


@dataclass(slots=True)
class OSLJoint:
    """Description of an OSL joint connection."""

    name: str
    gear_ratio: float
    port: str


@dataclass(slots=True)
class OSLLegConfig:
    """Configuration for an OpenSourceLeg actuator group."""

    controller_hz: int
    joints: tuple[OSLJoint, ...]


class OSLActuator(BaseActuator):
    """Context manager around `opensourceleg.osl.OpenSourceLeg`."""

    def __init__(self, config: OSLLegConfig | Mapping[str, object]):
        """Instantiate the actuator wrapper.

        Args:
            config: Controller frequency and joint definitions for the leg. A mapping
                (deserialised from YAML) is accepted for convenience and converted into
                :class:`OSLLegConfig`.

        Raises:
            RuntimeError: If the underlying `osl` module is unavailable.
        """
        if OpenSourceLeg is None:
            raise RuntimeError("opensourceleg.osl.OpenSourceLeg import failed. Install osl before use.")
        if isinstance(config, Mapping):
            config = self._build_config_from_mapping(config)
        joint_names = tuple(joint.name for joint in config.joints)
        super().__init__(BaseActuatorConfig(joint_names=joint_names))
        self._leg_config = config
        self._leg: Any = OpenSourceLeg(frequency=config.controller_hz)
        self._joint_handles: Dict[str, Any] = {}

    def start(self) -> None:
        """Connect to hardware and register joints with the OSL API."""
        for joint in self._leg_config.joints:
            self._leg.add_joint(name=joint.name, gear_ratio=joint.gear_ratio, port=joint.port)
        self._leg.__enter__()
        for joint in self._leg_config.joints:
            handle: Any = getattr(self._leg, joint.name)
            handle.set_mode(handle.control_modes.current)
            self._joint_handles[joint.name] = handle

    def stop(self) -> None:
        """Return hardware to a safe state and release the OSL context."""
        # Return hardware to idle modes.
        for handle in self._joint_handles.values():
            try:
                handle.set_mode(handle.control_modes.off)
            except Exception:  # pragma: no cover - best effort cleanup
                continue
        self._leg.__exit__(None, None, None)
        self._joint_handles.clear()

    def _apply_command(self, command: TorqueCommand) -> None:
        """Send a torque command to all configured joints."""
        missing = set(command.torques_nm) - set(self._joint_handles)
        if missing:
            raise ActuatorError(f"Torque command contains unknown joints: {missing}")
        for joint_name, torque in command.torques_nm.items():
            handle: Any = self._joint_handles[joint_name]
            handle.set_torque(float(torque))

    def fault_if_needed(self) -> None:
        """Query each joint for fault states and raise when detected.

        Raises:
            ActuatorError: If any joint reports a non-null fault state.

        Returns:
            None
        """
        status = {name: handle.get_fault_state() for name, handle in self._joint_handles.items()}
        faults = {name: state for name, state in status.items() if state is not None}
        if faults:
            raise ActuatorError(f"Detected actuator faults: {faults}")

    @staticmethod
    def _build_config_from_mapping(config: Mapping[str, object]) -> OSLLegConfig:
        controller_value = config.get("controller_hz", 500)
        if isinstance(controller_value, (int, float)):
            controller_hz = int(controller_value)
        elif isinstance(controller_value, str):
            controller_hz = int(controller_value)
        else:
            raise TypeError("controller_hz must be an int, float, or string representation")
        joints_raw = config.get("joints", ())
        joints: list[OSLJoint] = []
        if isinstance(joints_raw, Iterable):
            for entry in joints_raw:
                if not isinstance(entry, Mapping):
                    raise TypeError("Each joint entry must be a mapping")
                name = entry.get("name")
                port = entry.get("port")
                if name is None or port is None:
                    raise ValueError("Each OSL joint mapping must provide 'name' and 'port'")
                joints.append(
                    OSLJoint(
                        name=str(name),
                        gear_ratio=float(entry.get("gear_ratio", 1.0)),
                        port=str(port),
                    )
                )
        if not joints:
            raise ValueError("OSLLegConfig requires at least one joint definition")
        return OSLLegConfig(controller_hz=controller_hz, joints=tuple(joints))
