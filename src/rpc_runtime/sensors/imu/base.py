"""IMU abstractions used by the controller runtime."""

from __future__ import annotations

import abc
import logging
from dataclasses import dataclass, field
from typing import Dict, Protocol, Tuple, cast

from ..base import BaseSensor, BaseSensorConfig, SensorStaleDataError

LOGGER = logging.getLogger(__name__)

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
    "trunk": "/dev/ttyIMU_trunk",
    "thigh_r": "/dev/ttyIMU_thigh_r",
    "shank_r": "/dev/ttyIMU_shank_r",
    "foot_r": "/dev/ttyIMU_foot_r",
    "thigh_l": "/dev/ttyIMU_thigh_l",
    "shank_l": "/dev/ttyIMU_shank_l",
    "foot_l": "/dev/ttyIMU_foot_l",
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
    """Protocol for adapters that can re-align their orientation estimates."""

    def zero(self) -> None:
        """Reset internal offsets to align with controller coordinates."""


class IMUStaleDataError(RuntimeError):
    """Raised when stale data thresholds are exceeded for an IMU source."""


@dataclass(slots=True)
class BaseIMUConfig(BaseSensorConfig):
    """Base configuration shared across IMU implementations."""

    joint_names: Tuple[str, ...] = DEFAULT_JOINT_NAMES
    """Ordered joint names (subset of ``BaseIMU.JOINT_NAMES``) used for outputs."""

    segment_names: Tuple[str, ...] = DEFAULT_SEGMENT_NAMES
    """Ordered segment names (subset of ``BaseIMU.SEGMENT_NAMES``) expected from hardware."""

    port_map: Dict[str, str] = field(default_factory=lambda: dict(DEFAULT_PORT_MAP))
    """Mapping of segment name to underlying transport identifier/port."""

    max_stale_samples: int = 5
    """Number of consecutive stale reads tolerated before triggering a fault."""

    max_stale_time_s: float = 0.2
    """Maximum wall-clock time (seconds) without fresh data before fault."""

    fault_strategy: str = "raise"
    """One of ``{'raise', 'fallback', 'warn'}`` describing stale handling behaviour."""


class BaseIMU(BaseSensor):
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
        """Validate IMU configuration and wire up shared sensor behaviour."""
        cfg = config or BaseIMUConfig()
        super().__init__(cfg)

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
        return cast(BaseIMUConfig, super().config)

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
        """Mapping of segment name to underlying transport identifier/port."""
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

        Raises:
            ValueError: If the config omits joints, segments, or required port
                mappings, or if duplicates are present.
        """
        config = cast(BaseIMUConfig, super()._validate_config(config))
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

        cls._validate_joint_dependencies(joint_names, segment_names)

        config.joint_names = joint_names
        config.segment_names = segment_names
        config.port_map = normalized_port_map
        return config

    @staticmethod
    def _validate_joint_dependencies(joints: Tuple[str, ...], segments: Tuple[str, ...]) -> None:
        """Ensure joints only reference available segments."""
        required: Dict[str, Tuple[str, ...]] = {
            "knee_r": ("thigh_r", "shank_r"),
            "knee_l": ("thigh_l", "shank_l"),
            "ankle_r": ("shank_r", "foot_r"),
            "ankle_l": ("shank_l", "foot_l"),
            "hip_r": ("trunk", "thigh_r"),
            "hip_l": ("trunk", "thigh_l"),
        }
        missing_dependencies: Dict[str, Tuple[str, ...]] = {}
        for joint in joints:
            deps = required.get(joint)
            if not deps:
                continue
            missing = tuple(dep for dep in deps if dep not in segments)
            if missing:
                missing_dependencies[joint] = missing
        if missing_dependencies:
            details = [
                f"{joint}: missing {deps}"
                for joint, deps in missing_dependencies.items()
            ]
            raise ValueError(
                "Joint dependencies missing segments -> " + "; ".join(details)
            )

    # ------------------------------------------------------------------
    # Staleness management
    # ------------------------------------------------------------------

    def _handle_sample(self, sample: IMUSample | None, *, fresh: bool) -> IMUSample:

        def fallback(timestamp: float) -> IMUSample:
            joints = len(self.joint_names)
            segments = len(self.segment_names)
            return IMUSample(
                timestamp=timestamp,
                joint_angles_rad=tuple(0.0 for _ in range(joints)),
                joint_velocities_rad_s=tuple(0.0 for _ in range(joints)),
                segment_angles_rad=tuple(0.0 for _ in range(segments)),
                segment_velocities_rad_s=tuple(0.0 for _ in range(segments)),
            )

        try:
            return super()._handle_sample(sample, fresh=fresh, fallback_factory=fallback)
        except SensorStaleDataError as exc:
            raise IMUStaleDataError(str(exc)) from exc
