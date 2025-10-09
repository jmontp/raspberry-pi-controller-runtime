"""Adapter for the OpenSourceLeg actuator stack."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from .base import BaseActuator, TorqueCommand, ActuatorError

try:  # pragma: no cover - hardware dependency
    from opensourceleg.osl import OpenSourceLeg
except ImportError:  # noqa: F401 pragma: no cover - optional
    OpenSourceLeg = None


@dataclass(slots=True)
class OSLJoint:
    name: str
    gear_ratio: float
    port: str


@dataclass(slots=True)
class OSLLegConfig:
    controller_hz: int
    joints: tuple[OSLJoint, ...]


class OSLActuator(BaseActuator):
    """Context manager around `opensourceleg.osl.OpenSourceLeg`."""

    def __init__(self, config: OSLLegConfig):
        if OpenSourceLeg is None:
            raise RuntimeError(
                "opensourceleg.osl.OpenSourceLeg import failed. Install osl before use."
            )
        self._config = config
        self._leg = OpenSourceLeg(frequency=config.controller_hz)
        self._joint_handles: Dict[str, object] = {}

    def start(self) -> None:
        for joint in self._config.joints:
            self._leg.add_joint(name=joint.name, gear_ratio=joint.gear_ratio, port=joint.port)
        self._leg.__enter__()
        for joint in self._config.joints:
            handle = getattr(self._leg, joint.name)
            handle.set_mode(handle.control_modes.current)
            self._joint_handles[joint.name] = handle

    def stop(self) -> None:
        # Return hardware to idle modes.
        for handle in self._joint_handles.values():
            try:
                handle.set_mode(handle.control_modes.off)
            except Exception:  # pragma: no cover - best effort cleanup
                continue
        self._leg.__exit__(None, None, None)
        self._joint_handles.clear()

    def apply(self, command: TorqueCommand) -> None:
        missing = set(command.torques_nm) - set(self._joint_handles)
        if missing:
            raise ActuatorError(f"Torque command contains unknown joints: {missing}")
        for joint_name, torque in command.torques_nm.items():
            handle = self._joint_handles[joint_name]
            handle.set_torque(float(torque))

    def fault_if_needed(self) -> None:
        status = {name: handle.get_fault_state() for name, handle in self._joint_handles.items()}
        faults = {name: state for name, state in status.items() if state is not None}
        if faults:
            raise ActuatorError(f"Detected actuator faults: {faults}")
