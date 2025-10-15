"""Convenience exports for profile loading and runtime construction."""

from __future__ import annotations

from os import PathLike
from typing import Union

from rpc_runtime.runtime.wrangler import HardwareAvailabilityError

from .profile import (
    ActuatorBinding,
    ControllerBundle,
    ControllerManifest,
    RuntimeComponents,
    RuntimeProfile,
    SensorBinding,
    SignalRoute,
    build_runtime_components,
    load_components,
    load_runtime_profile,
)


def load_components_from(
    path: Union[str, bytes, PathLike[str], PathLike[bytes]]
) -> RuntimeComponents:
    """Convenience wrapper mirroring legacy API name."""
    return load_components(path)


__all__ = [
    "load_runtime_profile",
    "build_runtime_components",
    "load_components",
    "load_components_from",
    "RuntimeProfile",
    "RuntimeComponents",
    "ActuatorBinding",
    "ControllerBundle",
    "ControllerManifest",
    "SensorBinding",
    "SignalRoute",
    "HardwareAvailabilityError",
]
