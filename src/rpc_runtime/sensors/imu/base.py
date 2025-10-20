"""IMU abstractions used by the controller runtime."""

from __future__ import annotations

import abc
import logging
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, Mapping, Protocol, cast, runtime_checkable

from ..base import BaseSensor, BaseSensorConfig, SensorStaleDataError

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class CanonicalIMUFeature:
    """Description of a canonical IMU signal used by the runtime.

    Attributes:
        name: Canonical identifier used by controllers and diagnostics.
        provided: True when the feature must be read directly from hardware.
        description: Human readable explanation of the feature semantics.
        dependencies: Canonical feature names required to compute this one.
        compute: Callable that derives the feature from other canonical values.
    """

    name: str
    provided: bool
    description: str
    dependencies: tuple[str, ...] = ()
    compute: Callable[[Mapping[str, float]], float] | None = None


@dataclass(frozen=True, slots=True)
class CanonicalFeatureSemantics:
    """Structured interpretation of a canonical IMU feature identifier.

    Attributes:
        raw: Original canonical feature name.
        basis: Primary segment or joint described by the feature.
        axis: Anatomical axis encoded in the identifier when available.
        quantity: Physical quantity (angle, velocity, acceleration).
        motion: Motion qualifier (e.g., flexion, dorsiflexion) when present.
        side: Side hint encoded in the identifier (ipsi/contra) when present.
        units: Unit suffix extracted from the canonical name.
        derivative: Order of the temporal derivative encoded by the units.
        qualifiers: Additional tokens that help adapters refine hardware routing.
    """

    raw: str
    basis: str
    axis: str | None
    quantity: str | None
    motion: str | None
    side: str | None
    units: str | None
    derivative: int
    qualifiers: tuple[str, ...] = ()


def split_canonical_feature_name(name: str) -> CanonicalFeatureSemantics:
    """Decode a canonical IMU feature name into structured semantics.

    Args:
        name: Canonical feature identifier to parse.

    Returns:
        CanonicalFeatureSemantics: Structured metadata describing ``name``.

    Raises:
        ValueError: If ``name`` is empty or only whitespace.
    """
    if not name or not name.strip():
        raise ValueError("Canonical feature name must be non-empty")

    base = name.strip()
    units: str | None = None
    derivative = 0
    if base.endswith("_rad_s"):
        units = "rad/s"
        derivative = 1
        base = base[: -len("_rad_s")]
    elif base.endswith("_rad"):
        units = "rad"
        base = base[: -len("_rad")]

    side: str | None = None
    for suffix, token in (("_ipsi", "ipsi"), ("_contra", "contra")):
        if base.endswith(suffix):
            side = token
            base = base[: -len(suffix)]
            break

    axis: str | None = None
    quantity: str | None = None
    motion: str | None = None
    qualifiers: list[str] = []
    tokens = [token for token in base.split("_") if token]
    if tokens:
        basis = tokens[0]
        for token in tokens[1:]:
            if axis is None and token in {"sagittal", "coronal", "transverse"}:
                axis = token
            elif quantity is None and token in {"angle", "velocity", "acceleration"}:
                quantity = token
            elif motion is None and token in {"flexion", "dorsiflexion", "extension"}:
                motion = token
            else:
                qualifiers.append(token)
    else:
        basis = name.strip()

    return CanonicalFeatureSemantics(
        raw=name.strip(),
        basis=basis,
        axis=axis,
        quantity=quantity,
        motion=motion,
        side=side,
        units=units,
        derivative=derivative,
        qualifiers=tuple(qualifiers),
    )


def _difference(lhs: str, rhs: str) -> Callable[[Mapping[str, float]], float]:
    """Return a helper that computes ``values[lhs] - values[rhs]``."""

    def _compute(values: Mapping[str, float], a: str = lhs, b: str = rhs) -> float:
        return float(values[a]) - float(values[b])

    return _compute


_MEASURED_FEATURES: tuple[tuple[str, str], ...] = (
    ("trunk_sagittal_angle_rad", "Sagittal pitch of the trunk segment in radians."),
    (
        "thigh_sagittal_angle_ipsi_rad",
        "Sagittal pitch of the ipsilateral thigh segment in radians.",
    ),
    (
        "shank_sagittal_angle_ipsi_rad",
        "Sagittal pitch of the ipsilateral shank segment in radians.",
    ),
    (
        "foot_sagittal_angle_ipsi_rad",
        "Sagittal pitch of the ipsilateral foot segment in radians.",
    ),
    (
        "thigh_sagittal_angle_contra_rad",
        "Sagittal pitch of the contralateral thigh segment in radians.",
    ),
    (
        "shank_sagittal_angle_contra_rad",
        "Sagittal pitch of the contralateral shank segment in radians.",
    ),
    (
        "foot_sagittal_angle_contra_rad",
        "Sagittal pitch of the contralateral foot segment in radians.",
    ),
    ("trunk_sagittal_velocity_rad_s", "Sagittal trunk angular velocity in rad/s."),
    (
        "thigh_sagittal_velocity_ipsi_rad_s",
        "Sagittal angular velocity of the ipsilateral thigh in rad/s.",
    ),
    (
        "shank_sagittal_velocity_ipsi_rad_s",
        "Sagittal angular velocity of the ipsilateral shank in rad/s.",
    ),
    (
        "foot_sagittal_velocity_ipsi_rad_s",
        "Sagittal angular velocity of the ipsilateral foot in rad/s.",
    ),
    (
        "thigh_sagittal_velocity_contra_rad_s",
        "Sagittal angular velocity of the contralateral thigh in rad/s.",
    ),
    (
        "shank_sagittal_velocity_contra_rad_s",
        "Sagittal angular velocity of the contralateral shank in rad/s.",
    ),
    (
        "foot_sagittal_velocity_contra_rad_s",
        "Sagittal angular velocity of the contralateral foot in rad/s.",
    ),
)


_DERIVED_FEATURES: tuple[
    tuple[str, str, tuple[str, ...], Callable[[Mapping[str, float]], float]], ...
] = (
    (
        "hip_flexion_angle_ipsi_rad",
        "Ipsilateral hip flexion angle computed as trunk minus thigh (rad).",
        ("trunk_sagittal_angle_rad", "thigh_sagittal_angle_ipsi_rad"),
        _difference("trunk_sagittal_angle_rad", "thigh_sagittal_angle_ipsi_rad"),
    ),
    (
        "hip_flexion_angle_contra_rad",
        "Contralateral hip flexion angle computed as trunk minus thigh (rad).",
        ("trunk_sagittal_angle_rad", "thigh_sagittal_angle_contra_rad"),
        _difference("trunk_sagittal_angle_rad", "thigh_sagittal_angle_contra_rad"),
    ),
    (
        "knee_flexion_angle_ipsi_rad",
        "Ipsilateral knee flexion angle computed as thigh minus shank (rad).",
        ("thigh_sagittal_angle_ipsi_rad", "shank_sagittal_angle_ipsi_rad"),
        _difference("thigh_sagittal_angle_ipsi_rad", "shank_sagittal_angle_ipsi_rad"),
    ),
    (
        "knee_flexion_angle_contra_rad",
        "Contralateral knee flexion angle computed as thigh minus shank (rad).",
        ("thigh_sagittal_angle_contra_rad", "shank_sagittal_angle_contra_rad"),
        _difference("thigh_sagittal_angle_contra_rad", "shank_sagittal_angle_contra_rad"),
    ),
    (
        "ankle_dorsiflexion_angle_ipsi_rad",
        "Ipsilateral ankle dorsiflexion angle computed as shank minus foot (rad).",
        ("shank_sagittal_angle_ipsi_rad", "foot_sagittal_angle_ipsi_rad"),
        _difference("shank_sagittal_angle_ipsi_rad", "foot_sagittal_angle_ipsi_rad"),
    ),
    (
        "ankle_dorsiflexion_angle_contra_rad",
        "Contralateral ankle dorsiflexion angle computed as shank minus foot (rad).",
        ("shank_sagittal_angle_contra_rad", "foot_sagittal_angle_contra_rad"),
        _difference("shank_sagittal_angle_contra_rad", "foot_sagittal_angle_contra_rad"),
    ),
    (
        "hip_flexion_velocity_ipsi_rad_s",
        "Ipsilateral hip flexion velocity computed as trunk minus thigh (rad/s).",
        (
            "trunk_sagittal_velocity_rad_s",
            "thigh_sagittal_velocity_ipsi_rad_s",
        ),
        _difference("trunk_sagittal_velocity_rad_s", "thigh_sagittal_velocity_ipsi_rad_s"),
    ),
    (
        "hip_flexion_velocity_contra_rad_s",
        "Contralateral hip flexion velocity computed as trunk minus thigh (rad/s).",
        (
            "trunk_sagittal_velocity_rad_s",
            "thigh_sagittal_velocity_contra_rad_s",
        ),
        _difference("trunk_sagittal_velocity_rad_s", "thigh_sagittal_velocity_contra_rad_s"),
    ),
    (
        "knee_flexion_velocity_ipsi_rad_s",
        "Ipsilateral knee flexion velocity computed as thigh minus shank (rad/s).",
        (
            "thigh_sagittal_velocity_ipsi_rad_s",
            "shank_sagittal_velocity_ipsi_rad_s",
        ),
        _difference("thigh_sagittal_velocity_ipsi_rad_s", "shank_sagittal_velocity_ipsi_rad_s"),
    ),
    (
        "knee_flexion_velocity_contra_rad_s",
        "Contralateral knee flexion velocity computed as thigh minus shank (rad/s).",
        (
            "thigh_sagittal_velocity_contra_rad_s",
            "shank_sagittal_velocity_contra_rad_s",
        ),
        _difference(
            "thigh_sagittal_velocity_contra_rad_s",
            "shank_sagittal_velocity_contra_rad_s",
        ),
    ),
    (
        "ankle_dorsiflexion_velocity_ipsi_rad_s",
        "Ipsilateral ankle dorsiflexion velocity computed as shank minus foot (rad/s).",
        (
            "shank_sagittal_velocity_ipsi_rad_s",
            "foot_sagittal_velocity_ipsi_rad_s",
        ),
        _difference(
            "shank_sagittal_velocity_ipsi_rad_s",
            "foot_sagittal_velocity_ipsi_rad_s",
        ),
    ),
    (
        "ankle_dorsiflexion_velocity_contra_rad_s",
        "Contralateral ankle dorsiflexion velocity computed as shank minus foot (rad/s).",
        (
            "shank_sagittal_velocity_contra_rad_s",
            "foot_sagittal_velocity_contra_rad_s",
        ),
        _difference(
            "shank_sagittal_velocity_contra_rad_s",
            "foot_sagittal_velocity_contra_rad_s",
        ),
    ),
)


CANONICAL_FEATURE_REGISTRY: Dict[str, CanonicalIMUFeature] = {
    name: CanonicalIMUFeature(name=name, provided=True, description=description)
    for name, description in _MEASURED_FEATURES
}
for name, description, deps, func in _DERIVED_FEATURES:
    CANONICAL_FEATURE_REGISTRY[name] = CanonicalIMUFeature(
        name=name,
        provided=False,
        description=description,
        dependencies=deps,
        compute=func,
    )

HARDWARE_CANONICAL_FEATURES: tuple[str, ...] = tuple(
    name for name, feature in CANONICAL_FEATURE_REGISTRY.items() if feature.provided
)


@dataclass(slots=True)
class IMUSample:
    """Single IMU reading expressed purely in canonical coordinates.

    Args:
        timestamp: Monotonic time when the sample was captured.
        values: Mapping of canonical feature names to measured values.
    """

    timestamp: float
    values: Mapping[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Normalise mapping keys and values for quick lookups."""
        normalised: Dict[str, float] = {}
        for key, value in dict(self.values).items():
            try:
                normalised[str(key)] = float(value)
            except (TypeError, ValueError):
                LOGGER.debug("Ignoring non-numeric IMU value for '%s': %s", key, value)
        object.__setattr__(self, "values", normalised)

    def as_dict(self) -> Mapping[str, float]:
        """Expose canonical IMU values without copying."""
        return self.values

    def get(self, name: str, default: float | None = None) -> float | None:
        """Return the canonical value for ``name`` when present."""
        return self.values.get(name, default)

    def available_features(self) -> tuple[str, ...]:
        """List canonical features present in this sample."""
        return tuple(self.values.keys())


@runtime_checkable
class IMUResettable(Protocol):
    """Protocol for adapters that can re-align their orientation estimates."""

    def zero(self) -> None:
        """Reset internal offsets to align with controller coordinates."""


class IMUStaleDataError(RuntimeError):
    """Raised when stale data thresholds are exceeded for an IMU source."""


@dataclass(slots=True)
class BaseIMUConfig(BaseSensorConfig):
    """Base configuration shared across IMU implementations.

    Attributes:
        port_map: Mapping from canonical feature names to hardware transport identifiers.
    """

    port_map: Dict[str, str] = field(default_factory=dict)


class BaseIMU(BaseSensor):
    """Abstract interface for IMU sources in canonical controller coordinates."""

    CANONICAL_FEATURES: Mapping[str, CanonicalIMUFeature] = CANONICAL_FEATURE_REGISTRY
    HARDWARE_FEATURES: tuple[str, ...] = HARDWARE_CANONICAL_FEATURES

    def __init__(self, config: BaseIMUConfig | None = None) -> None:
        """Validate IMU configuration and initialise shared behaviour."""
        cfg = config or BaseIMUConfig()
        super().__init__(cfg)
        self._imu_config: BaseIMUConfig = cast(BaseIMUConfig, super().config)
        self._hardware_feature_order = tuple(self._imu_config.port_map.keys())
        self._latest_measured: Dict[str, float] = {}

    def await_startup_sample(
        self,
        signals: Iterable[str] | None = None,
        *,
        timeout_s: float | None = None,
    ) -> None:
        """Wait until a fresh sample covering requested canonicals is available."""
        requested = tuple(signals or self._hardware_feature_order)
        interested = [name for name in requested if name in self.CANONICAL_FEATURES]
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
            canonical = self._canonicalize_sample(sample)
            if self.last_sample_fresh and all(name in canonical for name in interested):
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
        canonical = self._canonicalize_sample(sample)
        values: dict[str, float] = {}
        for name in signals:
            value = canonical.get(name)
            if value is not None:
                values[name] = value
        return sample, values

    def _canonicalize_sample(self, sample: IMUSample) -> Dict[str, float]:
        """Normalise an IMU sample into the canonical feature dictionary."""
        measured = self._extract_measured_signals(sample)
        derived = self._collect_derived_signals(measured)
        canonical = {**measured, **derived}
        return canonical

    def _extract_measured_signals(self, sample: IMUSample) -> Dict[str, float]:
        """Extract canonical signals available directly from the IMU sample."""
        measured: Dict[str, float] = {}
        for name, value in sample.values.items():
            if name not in self.CANONICAL_FEATURES:
                continue
            feature = self.CANONICAL_FEATURES[name]
            if feature.compute is not None:
                measured[name] = float(value)
                continue
            measured[name] = float(value)
        self._latest_measured = measured
        return measured

    def _collect_derived_signals(self, measured: Mapping[str, float]) -> Dict[str, float]:
        """Compute derived canonical signals from measured channels."""
        derived: Dict[str, float] = {}
        available: Dict[str, float] = dict(measured)
        for feature in self.CANONICAL_FEATURES.values():
            if feature.compute is None:
                continue
            if feature.name in available:
                continue
            if not all(dep in available for dep in feature.dependencies):
                continue
            try:
                value = feature.compute(available)
            except Exception:  # pragma: no cover - defensive guard
                LOGGER.warning("Failed to derive IMU signal '%s'", feature.name, exc_info=True)
                continue
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                LOGGER.warning(
                    "Derived IMU signal '%s' produced non-numeric value '%s'",
                    feature.name,
                    value,
                )
                continue
            derived[feature.name] = numeric
            available[feature.name] = numeric
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
    def hardware_features(self) -> tuple[str, ...]:
        """Canonical features provided directly by the hardware mapping."""
        return self._hardware_feature_order

    @property
    def port_map(self) -> dict[str, str]:
        """Mapping of canonical feature name to underlying transport identifier."""
        return self._imu_config.port_map

    def feature_semantics(self, name: str) -> CanonicalFeatureSemantics:
        """Return parsed semantics for a canonical feature name."""
        return split_canonical_feature_name(name)

    @classmethod
    def _validate_config(cls, config: BaseSensorConfig) -> BaseSensorConfig:
        """Validate and normalise a ``BaseIMUConfig`` instance."""
        if not isinstance(config, BaseIMUConfig):
            raise TypeError("BaseIMU requires a BaseIMUConfig instance")

        config = cast(BaseIMUConfig, super()._validate_config(config))

        raw_port_map = dict(config.port_map or {})
        if not raw_port_map:
            raise ValueError("BaseIMUConfig.port_map must map canonical features to device ports")

        ordered: Dict[str, str] = {}
        for name, port in raw_port_map.items():
            canonical = str(name)
            feature = cls.CANONICAL_FEATURES.get(canonical)
            if feature is None:
                raise ValueError(
                    f"Unknown canonical IMU feature '{canonical}' in port_map. "
                    "Refer to CANONICAL_FEATURES for supported identifiers."
                )
            if feature.compute is not None:
                raise ValueError(
                    f"Feature '{canonical}' is derived and cannot be mapped to hardware."
                )
            ordered[canonical] = str(port)

        config.port_map = ordered
        return config

    def _handle_sample(
        self,
        sample: IMUSample | None,
        *,
        fresh: bool,
        fallback_factory: Callable[[float], object] | None = None,
    ) -> IMUSample:
        """Normalise stale handling for IMU samples."""

        def fallback(timestamp: float) -> IMUSample:
            baseline = dict.fromkeys(self._hardware_feature_order, 0.0)
            return IMUSample(timestamp=timestamp, values=baseline)

        factory: Callable[[float], object] = fallback_factory or fallback
        try:
            result = super()._handle_sample(sample, fresh=fresh, fallback_factory=factory)
        except SensorStaleDataError as exc:
            raise IMUStaleDataError(str(exc)) from exc
        return cast(IMUSample, result)
