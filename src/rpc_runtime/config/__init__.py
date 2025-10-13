"""Configuration helpers for wiring runtime components from declarative profiles."""

from .loader import load_runtime_profile
from .models import (
    CANONICAL_SIGNAL_REGISTRY,
    ActuatorBinding,
    ControllerBundle,
    ControllerManifest,
    HardwareAvailabilityError,
    InputSchema,
    RuntimeProfile,
    SchemaSignal,
    SensorBinding,
)

__all__ = [
    "load_runtime_profile",
    "CANONICAL_SIGNAL_REGISTRY",
    "ActuatorBinding",
    "ControllerBundle",
    "ControllerManifest",
    "HardwareAvailabilityError",
    "InputSchema",
    "RuntimeProfile",
    "SchemaSignal",
    "SensorBinding",
]
