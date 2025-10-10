"""Common sensor abstractions shared by IMU and GRF adapters."""

from __future__ import annotations

import abc
import logging
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque

LOGGER = logging.getLogger(__name__)


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
    recent_sample_periods: tuple[float, ...] = ()


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

    @abc.abstractmethod
    def start(self) -> None:
        """Initialise the sensor and begin streaming data."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Tear down resources."""

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
        return self._diagnostics

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
        timestamp = getattr(sample, "timestamp", None) if sample is not None else None
        now = time.monotonic() if timestamp is None else timestamp
        reason = self._check_stale(now, fresh)
        if reason is not None:
            return self._apply_fault_strategy(sample, now, reason, fallback_factory)
        if sample is not None:
            return sample
        return fallback_factory(now)

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
