"""IMU abstractions used by the controller runtime."""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Dict, Protocol, Tuple

DEFAULT_JOINT_NAMES: Tuple[str, ...] = (
    "hip_r",
    "knee_r",
    "ankle_r",
    "hip_l",
    "knee_l",
    "ankle_l",
)
DEFAULT_SEGMENT_NAMES: Tuple[str, ...] = (
    "trunk",
    "thigh_r",
    "shank_r",
    "foot_r",
    "thigh_l",
    "shank_l",
    "foot_l",
)
DEFAULT_JOINT_ANGLE_CONVENTIONS: Dict[str, str] = {
    "hip_r": "Positive flexion (thigh toward pelvis) in the sagittal plane.",
    "knee_r": "Positive flexion (heel toward posterior thigh).",
    "ankle_r": "Positive dorsiflexion (toes toward shin).",
    "hip_l": "Positive flexion (thigh toward pelvis) in the sagittal plane.",
    "knee_l": "Positive flexion (heel toward posterior thigh).",
    "ankle_l": "Positive dorsiflexion (toes toward shin).",
}
DEFAULT_SEGMENT_ANGLE_CONVENTIONS: Dict[str, str] = {
    "trunk": "Positive pitch is counter-clockwise when viewed from the right side (forward lean).",
    "thigh_r": "Positive rotation is counter-clockwise from the right side (anterior tilt).",
    "shank_r": "Positive rotation is counter-clockwise from the right side (dorsiflexion proxy).",
    "foot_r": "Positive rotation is counter-clockwise from the right side (toe-up).",
    "thigh_l": "Positive rotation is counter-clockwise from the left side (anterior tilt).",
    "shank_l": "Positive rotation is counter-clockwise from the left side (dorsiflexion proxy).",
    "foot_l": "Positive rotation is counter-clockwise from the left side (toe-up).",
}
DEFAULT_PORT_MAP: Dict[str, str] = {
    "hip_r": "/dev/ttyIMU0",
    "knee_r": "/dev/ttyIMU0",
    "ankle_r": "/dev/ttyIMU0",
    "hip_l": "/dev/ttyIMU1",
    "knee_l": "/dev/ttyIMU1",
    "ankle_l": "/dev/ttyIMU1",
}


@dataclass(slots=True)
class IMUSample:
    """Single IMU reading expressed in controller joint/segment coordinates.

    Args:
        timestamp: Monotonic time when the sample was captured.
        joint_angles_rad: Joint angles in radians ordered according to
            :pyattr:`BaseIMU.joint_names`.
        joint_velocities_rad_s: Joint angular velocities in radians per second,
            matching the order of ``joint_angles_rad``.
        segment_angles_rad: Segment angles (e.g., limb segments) in radians as
            defined by :pyattr:`BaseIMU.segment_names`.
        segment_velocities_rad_s: Segment angular velocities in radians per
            second aligned with ``segment_angles_rad``.
    """

    timestamp: float
    joint_angles_rad: tuple[float, ...]
    joint_velocities_rad_s: tuple[float, ...]
    segment_angles_rad: tuple[float, ...]
    segment_velocities_rad_s: tuple[float, ...]

    @property
    def joint_count(self) -> int:
        """Number of joints reported in the sample."""
        return len(self.joint_angles_rad)


class IMUResettable(Protocol):
    def zero(self) -> None:
        """Reset internal offsets to align with controller coordinates."""


@dataclass(slots=True)
class BaseIMUConfig:
    """Base configuration shared across IMU implementations."""

    joint_names: Tuple[str, ...] = DEFAULT_JOINT_NAMES
    segment_names: Tuple[str, ...] = DEFAULT_SEGMENT_NAMES
    port_map: Dict[str, str] = field(default_factory=lambda: dict(DEFAULT_PORT_MAP))


class BaseIMU(abc.ABC):
    """Abstract interface for IMU sources.

    Concrete implementations translate hardware frames into the canonical
    joint and segment representation expected by the controller.
    """

    supports_batch: bool = False
    #: Canonical joint names (ordered) that the IMU reports. Sub-classes should
    #: override or expose instance-specific values when available.
    JOINT_NAMES: tuple[str, ...] = DEFAULT_JOINT_NAMES
    #: Canonical segment names (ordered) emitted alongside joint data.
    SEGMENT_NAMES: tuple[str, ...] = DEFAULT_SEGMENT_NAMES
    #: Human-readable description of the joint angle sign convention.
    JOINT_ANGLE_CONVENTIONS: dict[str, str] = DEFAULT_JOINT_ANGLE_CONVENTIONS.copy()
    #: Human-readable description of the segment angle sign convention.
    SEGMENT_ANGLE_CONVENTIONS: dict[str, str] = DEFAULT_SEGMENT_ANGLE_CONVENTIONS.copy()

    def __init__(self, config: BaseIMUConfig | None = None) -> None:
        self._config = self._validate_config(config or BaseIMUConfig())

    def __enter__(self) -> "BaseIMU":
        """Enter context manager and start streaming if required."""
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Leave context manager and stop streaming."""
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
        return None

    def as_resettable(self) -> IMUResettable | None:
        """Expose optional reset behaviour when supported."""
        return self if isinstance(self, IMUResettable) else None

    @property
    def config(self) -> BaseIMUConfig:
        """Active configuration for this IMU instance."""
        return self._config

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Tuple of joint names in the order emitted by :meth:`read`."""
        return self._config.joint_names

    @property
    def segment_names(self) -> tuple[str, ...]:
        """Tuple of segment names in the order emitted by :meth:`read`."""
        return self._config.segment_names

    @property
    def port_map(self) -> dict[str, str]:
        """Mapping of joint name to underlying transport identifier/port."""
        return self._config.port_map

    @property
    def joint_angle_conventions(self) -> dict[str, str]:
        """Mapping of joint name to the signed angle convention description."""
        defaults = {**DEFAULT_JOINT_ANGLE_CONVENTIONS, **self.JOINT_ANGLE_CONVENTIONS}
        fallback = "Positive flexion (thigh toward pelvis) in the sagittal plane."
        return {name: defaults.get(name, fallback) for name in self.joint_names}

    @property
    def segment_angle_conventions(self) -> dict[str, str]:
        """Mapping of segment name to the signed angle convention description."""
        defaults = {**DEFAULT_SEGMENT_ANGLE_CONVENTIONS, **self.SEGMENT_ANGLE_CONVENTIONS}
        fallback = "Positive rotation counter-clockwise in the sagittal plane."
        return {name: defaults.get(name, fallback) for name in self.segment_names}

    @classmethod
    def _validate_config(cls, config: BaseIMUConfig) -> BaseIMUConfig:
        """Validate and normalise a `BaseIMUConfig` instance.

        Args:
            config: Candidate configuration to validate and copy.

        Returns:
            BaseIMUConfig: Normalised configuration with canonical tuples and ports.

        Raises:
            ValueError: If the config omits joints, segments, or required port
                mappings, or if duplicates are present.
        """
        joint_names = tuple(config.joint_names)
        if not joint_names:
            raise ValueError("BaseIMUConfig.joint_names must contain at least one entry")
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("BaseIMUConfig.joint_names contains duplicate entries")
        not_defined = [name for name in joint_names if name not in cls.JOINT_NAMES]
        if not_defined:
            raise ValueError(
                f"Unsupported joint names {not_defined}; allowed subset: {cls.JOINT_NAMES}"
            )

        segment_names = tuple(config.segment_names)
        if not segment_names:
            raise ValueError("BaseIMUConfig.segment_names must contain at least one entry")
        if len(set(segment_names)) != len(segment_names):
            raise ValueError("BaseIMUConfig.segment_names contains duplicate entries")
        unsupported_segments = [name for name in segment_names if name not in cls.SEGMENT_NAMES]
        if unsupported_segments:
            raise ValueError(
                "Unsupported segment names "
                f"{unsupported_segments}; allowed subset: {cls.SEGMENT_NAMES}"
            )

        port_map = dict(config.port_map or {})
        if not port_map:
            raise ValueError("BaseIMUConfig.port_map must map segment names to device ports")
        missing = [name for name in segment_names if name not in port_map]
        if missing:
            raise ValueError(f"Missing port mappings for segments: {missing}")
        normalized_port_map = {name: str(port_map[name]) for name in segment_names}

        config.joint_names = joint_names
        config.segment_names = segment_names
        config.port_map = normalized_port_map
        return config
