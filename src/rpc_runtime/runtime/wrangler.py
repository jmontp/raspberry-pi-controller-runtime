"""Data wrangling utilities that normalise sensor inputs into controller features."""

from __future__ import annotations

import time
import types
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, Iterable, Mapping, Sequence

from ..sensors.base import BaseSensor, SensorStaleDataError
from ..sensors.combinators import ControlInputs

if TYPE_CHECKING:  # pragma: no cover - typing aid only
    from ..config.models import SignalRoute


@dataclass(slots=True)
class FeatureMetadata:
    """Per-tick metadata accompanying a feature view."""

    timestamp: float
    stale: bool = False
    optional_presence: Mapping[str, bool] = field(default_factory=dict)
    missing_optional: tuple[str, ...] = ()


@dataclass(slots=True)
class FeatureView:
    """Read-only mapping-like view over a feature dictionary."""

    _data: Mapping[str, float]

    def __getitem__(self, key: str) -> float:
        """Return the value for a canonical feature name."""
        return self._data[key]

    def get(self, key: str, default: float | None = None) -> float | None:
        """Fetch a feature value or return ``default`` when absent."""
        return self._data.get(key, default)

    def items(self):  # pragma: no cover - convenience
        """Iterate over ``(name, value)`` pairs in the view."""
        return self._data.items()

    def as_dict(self) -> Mapping[str, float]:  # pragma: no cover - convenience
        """Expose the underlying mapping without copying."""
        return self._data


class HardwareAvailabilityError(RuntimeError):
    """Raised when required sensors or signals are missing for a profile."""


@dataclass(slots=True)
class SchemaSignal:
    """Describe a canonical signal expected by a controller model."""

    name: str
    required: bool = True


@dataclass(slots=True)
class InputSchema:
    """Ordered collection of input channels backing a controller manifest."""

    name: str
    signals: tuple[SchemaSignal, ...]
    description: str | None = None

    def required_signals(self) -> set[str]:
        """Return the set of required signal names."""
        return {s.name for s in self.signals if s.required}

    def all_signals(self) -> set[str]:
        """Return the set of all signal names."""
        return {s.name for s in self.signals}

    def validate_signals(
        self,
        available_signals: Mapping[str, bool] | Iterable[str],
    ) -> None:
        """Ensure all required signals are available from configured sensors."""
        available = set(available_signals)
        missing = self.required_signals() - available
        if missing:
            raise HardwareAvailabilityError(
                "Configured hardware does not expose required signals: "
                + ", ".join(sorted(missing))
            )


class DataWrangler:
    """Plan and serve controller features from sensor inputs."""

    def __init__(
        self,
        required_inputs: InputSchema,
        signal_routes: Sequence["SignalRoute"],
        sensors: Mapping[str, BaseSensor],
        *,
        diagnostics_sink=None,
    ) -> None:
        """Create a new wrangler and validate required signals against hardware."""
        if len(signal_routes) != len(required_inputs.signals):
            raise ValueError(
                "Input schema signals do not match the number of declared signal routes"
            )
        self._schema = required_inputs
        self._routes = tuple(signal_routes)
        self._sensors: Dict[str, BaseSensor] = dict(sensors)
        self._diagnostics = diagnostics_sink

        self._ordered_names = [signal.name for signal in self._schema.signals]
        self._required_names = self._schema.required_signals()
        self._optional_names = {
            signal.name for signal in self._schema.signals if not signal.required
        }

        provider_signals: Dict[str, list[str]] = {}
        for route in self._routes:
            provider = route.provider
            if provider is None:
                continue
            if provider not in self._sensors:
                if route.name in self._required_names:
                    raise HardwareAvailabilityError(
                        f"Signal '{route.name}' references unknown sensor '{provider}'"
                    )
                continue
            provider_signals.setdefault(provider, []).append(route.name)
        self._provider_signals = {
            alias: tuple(signals) for alias, signals in provider_signals.items()
        }
        available_for_validation: set[str] = set()
        for signals in self._provider_signals.values():
            available_for_validation.update(signals)
        available_for_validation.update(
            route.name
            for route in self._routes
            if route.provider is None or route.provider not in self._sensors
        )
        self._schema.validate_signals(available_for_validation)


    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def __enter__(self) -> "DataWrangler":  # pragma: no cover - simple delegation
        """Start hardware streams when entering a context."""
        self.probe()
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # pragma: no cover - simple delegation
        """Stop hardware streams when leaving a context."""
        self.stop()

    def probe(self) -> None:
        """Perform lightweight checks that hardware can start."""
        for sensor in self._sensors.values():
            probe = getattr(sensor, "probe", None)
            if probe is not None:
                probe()

    def start(self) -> None:
        """Start all configured sensors."""
        for alias, sensor in self._sensors.items():
            sensor.start()
            await_fn = getattr(sensor, "await_startup_sample", None)
            if not callable(await_fn):
                continue
            signals = self._provider_signals.get(alias, ())
            timeout = getattr(getattr(sensor, "config", None), "startup_timeout_s", None)
            await_fn(signals, timeout_s=timeout)

    def stop(self) -> None:
        """Stop all configured sensors."""
        for sensor in self._sensors.values():
            sensor.stop()

    # ------------------------------------------------------------------
    # Data path
    # ------------------------------------------------------------------

    def get_sensor_data(self) -> tuple[FeatureView, FeatureMetadata, ControlInputs]:
        """Fetch the latest canonical features and metadata."""
        feature_values: Dict[str, float] = {}
        samples: Dict[str, object] = {}
        missing_required: set[str] = set()
        optional_presence: Dict[str, bool] = dict.fromkeys(self._optional_names, False)
        primary_timestamp: float | None = None

        for alias, signals in self._provider_signals.items():
            sensor = self._sensors[alias]
            sample, values = sensor.read_signals(signals)
            samples[alias] = sample
            if primary_timestamp is None and hasattr(sample, "timestamp"):
                ts = sample.timestamp  # type: ignore[attr-defined]
                if ts is not None:
                    try:
                        primary_timestamp = float(ts)
                    except (TypeError, ValueError):
                        primary_timestamp = None
            for signal in signals:
                value = values.get(signal)
                if value is None:
                    if signal in self._required_names:
                        missing_required.add(signal)
                    continue
                feature_values[signal] = float(value)
                if signal in optional_presence:
                    optional_presence[signal] = True

        for name in self._required_names:
            if name not in feature_values:
                missing_required.add(name)

        if missing_required:
            raise SensorStaleDataError(
                "Missing required signals while building features: "
                + ", ".join(sorted(set(missing_required)))
            )

        frozen = FeatureView(types.MappingProxyType(dict(feature_values)))
        timestamp = primary_timestamp if primary_timestamp is not None else time.monotonic()
        meta = FeatureMetadata(
            timestamp=timestamp,
            stale=False,
            optional_presence=dict(optional_presence),
            missing_optional=tuple(
                sorted(name for name, present in optional_presence.items() if not present)
            ),
        )
        control_inputs = ControlInputs(samples=samples)
        return frozen, meta, control_inputs
