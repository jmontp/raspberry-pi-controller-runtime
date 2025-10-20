"""IMU abstractions used by the controller runtime."""

from __future__ import annotations

import abc
import logging
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, Mapping, Protocol, Tuple, cast, runtime_checkable

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

SEGMENT_NAME_ALIASES: Dict[str, str] = {
    "thigh_ipsi": "thigh_r",
    "thigh_right": "thigh_r",
    "thigh_contra": "thigh_l",
    "thigh_left": "thigh_l",
    "shank_ipsi": "shank_r",
    "shank_right": "shank_r",
    "shank_contra": "shank_l",
    "shank_left": "shank_l",
    "foot_ipsi": "foot_r",
    "foot_right": "foot_r",
    "foot_contra": "foot_l",
    "foot_left": "foot_l",
}

JOINT_NAME_ALIASES: Dict[str, str] = {
    "hip_ipsi": "hip_r",
    "hip_right": "hip_r",
    "hip_contra": "hip_l",
    "hip_left": "hip_l",
    "knee_ipsi": "knee_r",
    "knee_right": "knee_r",
    "knee_contra": "knee_l",
    "knee_left": "knee_l",
    "ankle_ipsi": "ankle_r",
    "ankle_right": "ankle_r",
    "ankle_contra": "ankle_l",
    "ankle_left": "ankle_l",
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

CANONICAL_SEGMENT_ANGLES: Dict[str, str] = {
    "trunk_sagittal_angle_rad": "trunk",
    "thigh_sagittal_angle_ipsi_rad": "thigh_r",
    "shank_sagittal_angle_ipsi_rad": "shank_r",
    "foot_sagittal_angle_ipsi_rad": "foot_r",
    "thigh_sagittal_angle_contra_rad": "thigh_l",
    "shank_sagittal_angle_contra_rad": "shank_l",
    "foot_sagittal_angle_contra_rad": "foot_l",
}

CANONICAL_SEGMENT_VELOCITIES: Dict[str, str] = {
    "trunk_sagittal_velocity_rad_s": "trunk",
    "thigh_sagittal_velocity_ipsi_rad_s": "thigh_r",
    "shank_sagittal_velocity_ipsi_rad_s": "shank_r",
    "foot_sagittal_velocity_ipsi_rad_s": "foot_r",
    "thigh_sagittal_velocity_contra_rad_s": "thigh_l",
    "shank_sagittal_velocity_contra_rad_s": "shank_l",
    "foot_sagittal_velocity_contra_rad_s": "foot_l",
}


def _segment_difference(lhs: str, rhs: str) -> Callable[[Mapping[str, float]], float]:
    def _compute(measured: Mapping[str, float], a: str = lhs, b: str = rhs) -> float:
        return measured[a] - measured[b]

    return _compute


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


@runtime_checkable
class IMUResettable(Protocol):
    """Protocol for adapters that can re-align their orientation estimates."""

    def zero(self) -> None:
        """Reset internal offsets to align with controller coordinates."""


class IMUStaleDataError(RuntimeError):
    """Raised when stale data thresholds are exceeded for an IMU source."""


@dataclass(slots=True)
class BaseIMUConfig(BaseSensorConfig):
    """Base configuration shared across IMU implementations."""

    joint_names: Tuple[str, ...] = field(default_factory=tuple)
    """Ordered joint names (subset of ``BaseIMU.JOINT_NAMES``) used for outputs."""

    segment_names: Tuple[str, ...] = field(default_factory=tuple)
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

    DERIVED_SIGNALS: Dict[str, tuple[tuple[str, ...], Callable[[Mapping[str, float]], float]]] = {
        "hip_flexion_angle_ipsi_rad": (
            ("trunk_sagittal_angle_rad", "thigh_sagittal_angle_ipsi_rad"),
            _segment_difference("trunk_sagittal_angle_rad", "thigh_sagittal_angle_ipsi_rad"),
        ),
        "hip_flexion_angle_contra_rad": (
            ("trunk_sagittal_angle_rad", "thigh_sagittal_angle_contra_rad"),
            _segment_difference("trunk_sagittal_angle_rad", "thigh_sagittal_angle_contra_rad"),
        ),
        "knee_flexion_angle_ipsi_rad": (
            ("thigh_sagittal_angle_ipsi_rad", "shank_sagittal_angle_ipsi_rad"),
            _segment_difference("thigh_sagittal_angle_ipsi_rad", "shank_sagittal_angle_ipsi_rad"),
        ),
        "knee_flexion_angle_contra_rad": (
            ("thigh_sagittal_angle_contra_rad", "shank_sagittal_angle_contra_rad"),
            _segment_difference("thigh_sagittal_angle_contra_rad", "shank_sagittal_angle_contra_rad"),
        ),
        "ankle_dorsiflexion_angle_ipsi_rad": (
            ("shank_sagittal_angle_ipsi_rad", "foot_sagittal_angle_ipsi_rad"),
            _segment_difference("shank_sagittal_angle_ipsi_rad", "foot_sagittal_angle_ipsi_rad"),
        ),
        "ankle_dorsiflexion_angle_contra_rad": (
            ("shank_sagittal_angle_contra_rad", "foot_sagittal_angle_contra_rad"),
            _segment_difference("shank_sagittal_angle_contra_rad", "foot_sagittal_angle_contra_rad"),
        ),
        "hip_flexion_velocity_ipsi_rad_s": (
            ("trunk_sagittal_velocity_rad_s", "thigh_sagittal_velocity_ipsi_rad_s"),
            _segment_difference("trunk_sagittal_velocity_rad_s", "thigh_sagittal_velocity_ipsi_rad_s"),
        ),
        "hip_flexion_velocity_contra_rad_s": (
            ("trunk_sagittal_velocity_rad_s", "thigh_sagittal_velocity_contra_rad_s"),
            _segment_difference("trunk_sagittal_velocity_rad_s", "thigh_sagittal_velocity_contra_rad_s"),
        ),
        "knee_flexion_velocity_ipsi_rad_s": (
            ("thigh_sagittal_velocity_ipsi_rad_s", "shank_sagittal_velocity_ipsi_rad_s"),
            _segment_difference("thigh_sagittal_velocity_ipsi_rad_s", "shank_sagittal_velocity_ipsi_rad_s"),
        ),
        "knee_flexion_velocity_contra_rad_s": (
            ("thigh_sagittal_velocity_contra_rad_s", "shank_sagittal_velocity_contra_rad_s"),
            _segment_difference("thigh_sagittal_velocity_contra_rad_s", "shank_sagittal_velocity_contra_rad_s"),
        ),
        "ankle_dorsiflexion_velocity_ipsi_rad_s": (
            ("shank_sagittal_velocity_ipsi_rad_s", "foot_sagittal_velocity_ipsi_rad_s"),
            _segment_difference("shank_sagittal_velocity_ipsi_rad_s", "foot_sagittal_velocity_ipsi_rad_s"),
        ),
        "ankle_dorsiflexion_velocity_contra_rad_s": (
            ("shank_sagittal_velocity_contra_rad_s", "foot_sagittal_velocity_contra_rad_s"),
            _segment_difference("shank_sagittal_velocity_contra_rad_s", "foot_sagittal_velocity_contra_rad_s"),
        ),
    }

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
        self._imu_config: BaseIMUConfig = cast(BaseIMUConfig, super().config)
        self._latest_measured: Dict[str, float] = {}

    def await_startup_sample(
        self,
        signals: Iterable[str] | None = None,
        *,
        timeout_s: float | None = None,
    ) -> None:
        """Wait until a fresh sample covering requested canonicals is available."""
        requested = set(signals or ())
        handled = set(CANONICAL_SEGMENT_ANGLES) | set(CANONICAL_SEGMENT_VELOCITIES)
        handled |= set(self.DERIVED_SIGNALS)
        interested = [name for name in requested if name in handled]
        if not interested:
            return
        poll = max(self.config.startup_poll_interval_s, 0.0)
        deadline = None
        if timeout_s is None:
            timeout_s = self.config.startup_timeout_s
        if timeout_s is not None and timeout_s > 0:
            deadline = time.monotonic() + timeout_s

        while True:
            sample = self.read()
            measured = self._collect_direct_signals(sample)
            derived = self._collect_derived_signals(measured)
            canonical = {**measured, **derived}
            if self.last_sample_fresh and all(name in canonical for name in interested):
                self._latest_measured = measured
                return
            if deadline is not None and time.monotonic() >= deadline:
                missing = [name for name in interested if name not in canonical]
                raise SensorStaleDataError(
                    "Timed out waiting for fresh IMU data during startup for signals: "
                    + ", ".join(sorted(missing))
                )
            if poll:
                time.sleep(poll)

    @abc.abstractmethod
    def start(self) -> None:
        """Initialise connections and start streaming data."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Tear down hardware resources."""

    @abc.abstractmethod
    def read(self) -> IMUSample:
        """Return the latest controller-frame IMU sample."""

    def read_signals(self, signals: Iterable[str]) -> tuple[IMUSample, dict[str, float]]:
        """Return the latest sample along with selected canonical signals."""
        sample = self.read()
        measured = self._collect_direct_signals(sample)
        derived = self._collect_derived_signals(measured)
        canonical = {**measured, **derived}
        self._latest_measured = measured
        values: dict[str, float] = {}
        for name in signals:
            value = canonical.get(name)
            if value is not None:
                values[name] = value
        return sample, values

    def _collect_direct_signals(self, sample: IMUSample) -> Dict[str, float]:
        """Extract canonical signals available directly from the IMU sample."""
        measured: Dict[str, float] = {}
        for canonical, segment_name in CANONICAL_SEGMENT_ANGLES.items():
            value = self._resolve_named_segment(sample, "segment_angles_rad", segment_name)
            if value is not None:
                measured[canonical] = value

        for canonical, segment_name in CANONICAL_SEGMENT_VELOCITIES.items():
            value = self._resolve_named_segment(sample, "segment_velocities_rad_s", segment_name)
            if value is not None:
                measured[canonical] = value

        return measured

    def _collect_derived_signals(self, measured: Mapping[str, float]) -> Dict[str, float]:
        """Compute derived canonical signals from measured channels."""
        derived: Dict[str, float] = {}
        for name, (dependencies, compute) in self.DERIVED_SIGNALS.items():
            if name in measured:
                continue
            if not all(dep in measured for dep in dependencies):
                continue
            try:
                value = compute(measured)
            except Exception:  # pragma: no cover - defensive guard
                LOGGER.warning("Failed to derive IMU signal '%s'", name, exc_info=True)
                continue
            try:
                derived[name] = float(value)
            except (TypeError, ValueError):
                LOGGER.warning(
                    "Derived IMU signal '%s' produced non-numeric value '%s'", name, value
                )
        return derived

    def reset(self) -> None:
        """Optional method to zero orientation and velocity estimates."""
        return None

    def as_resettable(self) -> IMUResettable | None:
        """Expose optional reset behaviour when supported."""
        return self if isinstance(self, IMUResettable) else None

    @property
    def config(self) -> BaseIMUConfig:
        """Active configuration for this IMU instance."""
        return self._imu_config

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Tuple of joint names in the order emitted by :meth:`read`."""
        return self._imu_config.joint_names

    @property
    def segment_names(self) -> tuple[str, ...]:
        """Tuple of segment names in the order emitted by :meth:`read`."""
        return self._imu_config.segment_names

    @property
    def port_map(self) -> dict[str, str]:
        """Mapping of segment name to underlying transport identifier/port."""
        return self._imu_config.port_map

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

    def _resolve_joint_attribute(
        self,
        sample: IMUSample,
        attribute: str,
        index: int,
    ) -> float | None:
        """Return a joint attribute (angle or velocity) when available."""
        if index < 0:
            return None
        data = getattr(sample, attribute, None)
        if data is None:
            return None
        try:
            return float(data[index])
        except (IndexError, TypeError, ValueError):
            return None

    def _resolve_named_joint(
        self,
        sample: IMUSample,
        attribute: str,
        joint_name: str,
    ) -> float | None:
        """Resolve a joint attribute using the configured joint ordering."""
        try:
            index = self.joint_names.index(joint_name)
        except ValueError:
            return None
        return self._resolve_joint_attribute(sample, attribute, index)

    def _resolve_named_segment(
        self,
        sample: IMUSample,
        attribute: str,
        segment_name: str,
    ) -> float | None:
        """Resolve a segment attribute using the configured segment ordering."""
        try:
            index = self.segment_names.index(segment_name)
        except ValueError:
            return None
        data = getattr(sample, attribute, None)
        if data is None:
            return None
        try:
            return float(data[index])
        except (IndexError, TypeError, ValueError):
            return None

    @classmethod
    def _validate_config(cls, config: BaseSensorConfig) -> BaseSensorConfig:
        """Validate and normalise a `BaseIMUConfig` instance.

        Args:
            config: Candidate configuration to validate and copy.

        Raises:
            ValueError: If the config omits joints, segments, or required port
                mappings, or if duplicates are present.
        """
        if not isinstance(config, BaseIMUConfig):
            raise TypeError("BaseIMU requires a BaseIMUConfig instance")

        config = cast(BaseIMUConfig, super()._validate_config(config))

        raw_joint_names = tuple(config.joint_names or ())
        joint_names = tuple(cls._normalize_joint_name(name) for name in raw_joint_names)
        if joint_names:
            if len(set(joint_names)) != len(joint_names):
                raise ValueError("BaseIMUConfig.joint_names contains duplicate entries")
            not_defined = [name for name in joint_names if name not in cls.JOINT_NAMES]
            if not_defined:
                raise ValueError(
                    f"Unsupported joint names {not_defined}; allowed subset: {cls.JOINT_NAMES}"
                )
        else:
            joint_names = cls.JOINT_NAMES

        raw_port_map = dict(config.port_map or {})
        normalized_port_map: Dict[str, str] = {}
        for key, value in raw_port_map.items():
            canonical = cls._normalize_segment_name(str(key))
            normalized_port_map[canonical] = str(value)
        if not normalized_port_map:
            raise ValueError("BaseIMUConfig.port_map must map segment names to device ports")

        raw_segment_names = tuple(config.segment_names or ())
        requested_segments = tuple(cls._normalize_segment_name(name) for name in raw_segment_names)
        if requested_segments:
            segment_names = requested_segments
        else:
            segment_names = tuple(normalized_port_map.keys()) or cls.SEGMENT_NAMES

        if len(set(segment_names)) != len(segment_names):
            raise ValueError("BaseIMUConfig.segment_names contains duplicate entries")
        unsupported_segments = [name for name in segment_names if name not in cls.SEGMENT_NAMES]
        if unsupported_segments:
            raise ValueError(
                "Unsupported segment names "
                f"{unsupported_segments}; allowed subset: {cls.SEGMENT_NAMES}"
            )

        missing = [name for name in segment_names if name not in normalized_port_map]
        if missing:
            raise ValueError(f"Missing port mappings for segments: {missing}")
        ordered_port_map = {name: normalized_port_map[name] for name in segment_names}

        cls._validate_joint_dependencies(joint_names, segment_names)

        config.joint_names = joint_names
        config.segment_names = segment_names
        config.port_map = ordered_port_map
        return config

    @classmethod
    def _normalize_segment_name(cls, name: str) -> str:
        canonical = SEGMENT_NAME_ALIASES.get(name, name)
        return canonical

    @classmethod
    def _normalize_joint_name(cls, name: str) -> str:
        canonical = JOINT_NAME_ALIASES.get(name, name)
        return canonical

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

    def _handle_sample(
        self,
        sample: IMUSample | None,
        *,
        fresh: bool,
        fallback_factory: Callable[[float], object] | None = None,
    ) -> IMUSample:
        """Normalise stale handling for IMU samples."""

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

        factory: Callable[[float], object] = fallback_factory or fallback
        try:
            result = super()._handle_sample(sample, fresh=fresh, fallback_factory=factory)
        except SensorStaleDataError as exc:
            raise IMUStaleDataError(str(exc)) from exc
        return cast(IMUSample, result)
