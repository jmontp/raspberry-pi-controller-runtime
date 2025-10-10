"""Abstractions for vertical ground reaction force sensing."""

from __future__ import annotations

import abc
import logging
from dataclasses import dataclass
from typing import Dict, Tuple, cast

from ..base import BaseSensor, BaseSensorConfig, SensorStaleDataError

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
class BaseVerticalGRFConfig(BaseSensorConfig):
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


class BaseVerticalGRF(BaseSensor):
    """Abstract interface for vertical GRF sources."""

    CHANNEL_NAMES: Tuple[str, ...] = DEFAULT_CHANNEL_NAMES
    FORCE_CONVENTIONS: Dict[str, str] = DEFAULT_FORCE_CONVENTIONS.copy()

    def __init__(self, config: BaseVerticalGRFConfig | None = None) -> None:
        """Initialise the base with validated GRF configuration."""
        cfg = config or BaseVerticalGRFConfig()
        super().__init__(cfg)

    @property
    def config(self) -> BaseVerticalGRFConfig:
        """Active configuration for this GRF instance."""
        return cast(BaseVerticalGRFConfig, super().config)

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
        self,
        sample: VerticalGRFSample | None,
        *,
        fresh: bool,
    ) -> VerticalGRFSample:

        def fallback(timestamp: float) -> VerticalGRFSample:
            return VerticalGRFSample(
                timestamp=timestamp,
                forces_newton=tuple(0.0 for _ in self.channel_names),
            )

        try:
            return super()._handle_sample(sample, fresh=fresh, fallback_factory=fallback)
        except SensorStaleDataError as exc:
            raise GRFStaleDataError(str(exc)) from exc

    @classmethod
    def _validate_config(cls, config: BaseVerticalGRFConfig) -> BaseVerticalGRFConfig:
        config = cast(BaseVerticalGRFConfig, super()._validate_config(config))
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

        config.channel_names = channels
        return config
