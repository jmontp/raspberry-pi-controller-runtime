"""Abstractions for vertical ground reaction force sensing."""

from __future__ import annotations

import abc
import logging
import time
from dataclasses import dataclass
from typing import Dict, Tuple

LOGGER = logging.getLogger(__name__)

DEFAULT_CHANNEL_NAMES: Tuple[str, ...] = ("channel_0",)
DEFAULT_FORCE_CONVENTIONS: Dict[str, str] = {
    "channel_0": "Positive indicates upward (support) force in newtons."
}


@dataclass(slots=True)
class VerticalGRFSample:
    """Vertical ground reaction force measurement.

    Args:
        timestamp: Monotonic capture time in seconds.
        forces_newton: Tuple of vertical forces per sensing channel in newtons.
            Forces are positive when acting upward against the ground.
    """

    timestamp: float
    forces_newton: tuple[float, ...]

    @property
    def channels(self) -> int:
        """Number of force channels contained in the sample."""
        return len(self.forces_newton)


@dataclass(slots=True)
class BaseVerticalGRFConfig:
    """Configuration shared across GRF implementations."""

    channel_names: Tuple[str, ...] = DEFAULT_CHANNEL_NAMES
    """Ordered channel names (subset of ``BaseVerticalGRF.CHANNEL_NAMES``)."""

    max_stale_samples: int = 5
    """Number of consecutive stale reads tolerated before triggering a fault."""

    max_stale_time_s: float = 0.2
    """Maximum wall-clock time (seconds) without fresh data before fault."""

    fault_strategy: str = "raise"
    """One of ``{'raise', 'fallback', 'warn'}`` describing stale handling behaviour."""


class GRFStaleDataError(RuntimeError):
    """Raised when stale data thresholds are exceeded for a GRF source."""


class BaseVerticalGRF(abc.ABC):
    """Abstract interface for vertical GRF sources."""

    CHANNEL_NAMES: Tuple[str, ...] = DEFAULT_CHANNEL_NAMES
    FORCE_CONVENTIONS: Dict[str, str] = DEFAULT_FORCE_CONVENTIONS.copy()

    def __init__(self, config: BaseVerticalGRFConfig | None = None) -> None:
        cfg = config or BaseVerticalGRFConfig()
        self._config = self._validate_config(cfg)
        self._stale_samples = 0
        self._last_sample_timestamp: float | None = None
        self._max_stale_samples = max(0, self._config.max_stale_samples)
        self._max_stale_time = max(0.0, self._config.max_stale_time_s)
        self._fault_strategy = self._config.fault_strategy

    def __enter__(self) -> "BaseVerticalGRF":
        """Enter context manager and start the GRF sensor."""
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Leave context manager and stop the GRF sensor."""
        self.stop()

    @property
    def config(self) -> BaseVerticalGRFConfig:
        """Active configuration for this GRF instance."""
        return self._config

    @property
    def channel_names(self) -> tuple[str, ...]:
        """Channel names in the order emitted by :meth:`read`."""
        return self._config.channel_names

    @property
    def channel_force_conventions(self) -> Dict[str, str]:
        """Mapping of channel name to force direction convention."""
        defaults = {**DEFAULT_FORCE_CONVENTIONS, **self.FORCE_CONVENTIONS}
        fallback = "Positive indicates upward (support) force in newtons."
        return {name: defaults.get(name, fallback) for name in self.channel_names}

    @abc.abstractmethod
    def start(self) -> None:
        """Initialise the sensor and begin streaming data."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Tear down resources."""

    @abc.abstractmethod
    def read(self) -> VerticalGRFSample:
        """Return the latest vertical GRF sample."""

    def zero(self) -> None:
        """Optional re-zero hook."""
        return None

    def _handle_sample(
        self, sample: VerticalGRFSample | None, *, fresh: bool
    ) -> VerticalGRFSample:
        """Apply configured staleness policy and return a sample."""

        timestamp = sample.timestamp if sample is not None else time.monotonic()
        if fresh and sample is not None:
            self._stale_samples = 0
            self._last_sample_timestamp = timestamp
            return sample

        self._stale_samples += 1
        if self._last_sample_timestamp is None:
            self._last_sample_timestamp = timestamp
        stale_time = timestamp - self._last_sample_timestamp

        exceeded_samples = (
            self._max_stale_samples
            and self._stale_samples >= self._max_stale_samples
        )
        exceeded_time = (
            self._max_stale_time
            and stale_time >= self._max_stale_time
        )

        if exceeded_samples or exceeded_time:
            reason = (
                f"stale_samples={self._stale_samples} (max {self._max_stale_samples}), "
                f"stale_time={stale_time:.3f}s (max {self._max_stale_time:.3f}s)"
            )
            return self._apply_fault_strategy(sample, timestamp, reason)

        if sample is not None:
            return sample
        return self._fallback_sample(timestamp)

    def _apply_fault_strategy(
        self,
        sample: VerticalGRFSample | None,
        timestamp: float,
        reason: str,
    ) -> VerticalGRFSample:
        strategy = self._fault_strategy
        if strategy == "raise":
            raise GRFStaleDataError(f"GRF stale data threshold exceeded ({reason})")

        fallback_sample = sample if sample is not None else self._fallback_sample(timestamp)
        self._stale_samples = 0
        self._last_sample_timestamp = timestamp

        if strategy == "fallback":
            LOGGER.warning("GRF stale data detected (%s); using fallback sample", reason)
            return fallback_sample

        if strategy == "warn":
            LOGGER.warning("GRF stale data detected (%s); passing through sample", reason)
            return fallback_sample

        raise GRFStaleDataError(
            f"GRF stale data threshold exceeded ({reason}); unknown strategy '{strategy}'"
        )

    def _fallback_sample(self, timestamp: float) -> VerticalGRFSample:
        return VerticalGRFSample(
            timestamp=timestamp,
            forces_newton=tuple(0.0 for _ in self.channel_names),
        )

    @classmethod
    def _validate_config(cls, config: BaseVerticalGRFConfig) -> BaseVerticalGRFConfig:
        channels = tuple(config.channel_names)
        if not channels:
            raise ValueError("BaseVerticalGRFConfig.channel_names must contain at least one entry")
        if len(set(channels)) != len(channels):
            raise ValueError("BaseVerticalGRFConfig.channel_names contains duplicate entries")
        unsupported = [name for name in channels if name not in cls.CHANNEL_NAMES]
        if unsupported:
            raise ValueError(
                f"Unsupported channel names {unsupported}; allowed subset: {cls.CHANNEL_NAMES}"
            )

        if config.max_stale_samples < 0:
            raise ValueError("BaseVerticalGRFConfig.max_stale_samples must be non-negative")
        if config.max_stale_time_s < 0:
            raise ValueError("BaseVerticalGRFConfig.max_stale_time_s must be non-negative")
        strategy = config.fault_strategy.lower()
        if strategy not in {"raise", "fallback", "warn"}:
            raise ValueError(
                "BaseVerticalGRFConfig.fault_strategy must be 'raise', 'fallback', or 'warn'"
            )
        config.channel_names = channels
        config.fault_strategy = strategy
        return config
