"""Common sensor abstractions shared by IMU and GRF adapters."""

from __future__ import annotations

import abc
import logging
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Iterable

LOGGER = logging.getLogger(__name__)


def _compute_histogram(samples: Iterable[float], bins: int) -> tuple[tuple[float, ...], tuple[int, ...]]:
    values = [float(s) for s in samples if not math.isnan(float(s))]
    if not values:
        return (), ()
    minimum = min(values)
    maximum = max(values)
    if math.isclose(minimum, maximum):
        return (minimum, maximum), (len(values),)
    bin_count = max(1, min(bins, len(values)))
    span = maximum - minimum
    width = span / bin_count if span else 1.0
    edges = [minimum + i * width for i in range(bin_count)]
    edges.append(maximum)
    counts = [0 for _ in range(bin_count)]
    for value in values:
        if value >= maximum:
            index = bin_count - 1
        else:
            index = int((value - minimum) / width)
        counts[index] += 1
    return tuple(edges), tuple(counts)


@dataclass(slots=True)
class SensorDiagnostics:
    """Diagnostics bundle capturing sampling statistics."""

    last_timestamp: float | None = None
    last_dt: float | None = None
    hz_estimate: float | None = None
    hz_min: float | None = None
    hz_max: float | None = None
    hz_mean: float | None = None
    stale_samples: int = 0
    total_samples: int = 0
    max_stale_samples: int = 0
    max_stale_time_s: float = 0.0
    max_consecutive_stale: int = 0
    recent_sample_periods: tuple[float, ...] = ()
    recent_sample_rates: tuple[float, ...] = ()
    recent_event_timestamps: tuple[float, ...] = ()
    recent_event_fresh: tuple[bool, ...] = ()
    fresh_ratio_window: float | None = None


@dataclass(slots=True)
class BaseSensorConfig:
    """Configuration shared across sensor modalities."""

    max_stale_samples: int = 5
    max_stale_time_s: float = 0.2
    fault_strategy: str = "raise"  # raise, fallback, warn
    diagnostics_window: int = 128


class SensorStaleDataError(RuntimeError):
    """Raised when stale data thresholds are exceeded for a sensor source."""


class BaseSensor(abc.ABC):
    """Common functionality for sensor adapters (staleness, diagnostics)."""

    def __init__(self, config: BaseSensorConfig | None = None) -> None:
        """Normalise configuration and initialise staleness tracking."""
        cfg = config or BaseSensorConfig()
        self._config = self._validate_config(cfg)
        self._stale_samples = 0
        self._last_timestamp: float | None = None
        self._fault_strategy = self._config.fault_strategy
        self._diagnostics = SensorDiagnostics(
            max_stale_samples=self._config.max_stale_samples,
            max_stale_time_s=self._config.max_stale_time_s,
        )
        self._history: Deque[float] = deque(maxlen=self._config.diagnostics_window)
        self._event_history: Deque[tuple[float, bool]] = deque(
            maxlen=self._config.diagnostics_window
        )
        self._event_origin: float | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def __enter__(self) -> "BaseSensor":
        """Start the sensor when entering a context manager."""
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Ensure resources are released when leaving a context."""
        self.stop()

    def probe(self) -> None:  # pragma: no cover - default no-op
        """Perform a lightweight readiness check before starting streams."""
        return None

    @abc.abstractmethod
    def start(self) -> None:
        """Initialise the sensor and begin streaming data."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Tear down resources."""

    def read_signals(self, signals: Iterable[str]) -> tuple[object, dict[str, float]]:
        """Return the latest sample along with selected canonical signals."""
        raise NotImplementedError(
            f"{self.__class__.__name__} does not implement 'read_signals'"
        )

    # ------------------------------------------------------------------
    # Diagnostics & validation
    # ------------------------------------------------------------------

    @property
    def diagnostics(self) -> SensorDiagnostics:
        """Return the latest sampling diagnostics for the sensor."""
        if self._history:
            dt_min = min(self._history)
            dt_max = max(self._history)
            dt_mean = sum(self._history) / len(self._history)
            self._diagnostics.hz_min = 1.0 / dt_max if dt_max > 0 else None
            self._diagnostics.hz_max = 1.0 / dt_min if dt_min > 0 else None
            self._diagnostics.hz_mean = 1.0 / dt_mean if dt_mean > 0 else None
        else:
            self._diagnostics.hz_min = None
            self._diagnostics.hz_max = None
            self._diagnostics.hz_mean = None
        self._diagnostics.recent_sample_periods = tuple(self._history)
        self._diagnostics.recent_sample_rates = tuple(
            (1.0 / dt) if dt > 0 else math.inf for dt in self._history
        )
        if self._event_history:
            timestamps, fresh_flags = zip(*self._event_history, strict=False)
            fresh_ratio = sum(1.0 for flag in fresh_flags if flag) / len(fresh_flags)
            self._diagnostics.recent_event_timestamps = tuple(timestamps)
            self._diagnostics.recent_event_fresh = tuple(fresh_flags)
            self._diagnostics.fresh_ratio_window = fresh_ratio
        else:
            self._diagnostics.recent_event_timestamps = ()
            self._diagnostics.recent_event_fresh = ()
            self._diagnostics.fresh_ratio_window = None
        return self._diagnostics

    def sample_period_histogram(self, bins: int = 10) -> tuple[tuple[float, ...], tuple[int, ...]]:
        """Return histogram bins/counts for recent sample periods."""
        return _compute_histogram(self._history, bins)

    def sample_rate_histogram(self, bins: int = 10) -> tuple[tuple[float, ...], tuple[int, ...]]:
        """Return histogram bins/counts for recent sample rates (Hz)."""
        rates = (1.0 / dt for dt in self._history if dt > 0)
        return _compute_histogram(rates, bins)

    @property
    def config(self) -> BaseSensorConfig:
        """Active configuration for the sensor instance."""
        return self._config

    @classmethod
    def _validate_config(cls, config: BaseSensorConfig) -> BaseSensorConfig:
        """Validate base configuration constraints shared across sensors."""
        if config.max_stale_samples < 0:
            raise ValueError("max_stale_samples must be non-negative")
        if config.max_stale_time_s < 0:
            raise ValueError("max_stale_time_s must be non-negative")
        if config.diagnostics_window <= 0:
            raise ValueError("diagnostics_window must be positive")
        strategy = config.fault_strategy.lower()
        if strategy not in {"raise", "fallback", "warn"}:
            raise ValueError("fault_strategy must be 'raise', 'fallback', or 'warn'")
        config.fault_strategy = strategy
        return config

    # ------------------------------------------------------------------
    # Sample management
    # ------------------------------------------------------------------

    def _handle_sample(
        self,
        sample,
        *,
        fresh: bool,
        fallback_factory: Callable[[float], object],
    ):
        """Track staleness and apply the configured strategy for missing data."""
        event_time = time.monotonic()
        timestamp = getattr(sample, "timestamp", None) if sample is not None else None
        now = event_time if timestamp is None else float(timestamp)
        reason = self._check_stale(now, fresh)
        if reason is not None:
            result = self._apply_fault_strategy(sample, now, reason, fallback_factory)
            self._record_sample_event(event_time, False)
            return result
        if sample is not None:
            self._record_sample_event(event_time, True)
            return sample
        fallback_sample = fallback_factory(now)
        self._record_sample_event(event_time, False)
        return fallback_sample

    def _check_stale(self, now: float, fresh: bool) -> str | None:
        """Update counters and report a fault reason when thresholds are exceeded."""
        if fresh:
            if self._last_timestamp is not None:
                dt = max(now - self._last_timestamp, 1e-9)
                self._history.append(dt)
                self._diagnostics.last_dt = dt
                self._diagnostics.hz_estimate = 1.0 / dt
            self._stale_samples = 0
            self._last_timestamp = now
            self._diagnostics.last_timestamp = now
            self._diagnostics.stale_samples = 0
            self._diagnostics.total_samples += 1
            return None

        self._stale_samples += 1
        self._diagnostics.stale_samples = self._stale_samples
        self._diagnostics.max_consecutive_stale = max(
            self._diagnostics.max_consecutive_stale, self._stale_samples
        )
        stale_time = 0.0
        if self._last_timestamp is not None:
            stale_time = now - self._last_timestamp

        exceeded_samples = (
            self._config.max_stale_samples
            and self._stale_samples >= self._config.max_stale_samples
        )
        exceeded_time = (
            self._config.max_stale_time_s
            and stale_time >= self._config.max_stale_time_s
        )
        if exceeded_samples or exceeded_time:
            return (
                f"stale_samples={self._stale_samples} (max {self._config.max_stale_samples}), "
                f"stale_time={stale_time:.3f}s (max {self._config.max_stale_time_s:.3f}s)"
            )
        return None

    def _apply_fault_strategy(
        self,
        sample,
        timestamp: float,
        reason: str,
        fallback_factory: Callable[[float], object],
    ):
        """Execute the configured stale-data policy and return a safe sample."""
        fallback_sample = fallback_factory(timestamp)
        self._stale_samples = 0
        self._last_timestamp = timestamp
        self._diagnostics.last_timestamp = timestamp
        self._diagnostics.stale_samples = 0

        strategy = self._fault_strategy
        if strategy == "raise":
            raise SensorStaleDataError(reason)

        LOGGER.warning("Sensor stale data detected (%s); using fallback sample", reason)
        if strategy == "warn" and sample is not None:
            return sample
        return fallback_sample

    def _record_sample_event(self, timestamp: float, fresh: bool) -> None:
        """Record a sampling event for diagnostics reporting."""
        if self._event_origin is None:
            self._event_origin = timestamp
        relative = max(timestamp - self._event_origin, 0.0)
        self._event_history.append((relative, fresh))
