"""Abstractions for vertical ground reaction force sensing."""

from __future__ import annotations

import abc
import logging
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, Tuple, cast

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
        self._grf_config: BaseVerticalGRFConfig = cast(BaseVerticalGRFConfig, super().config)

    @property
    def config(self) -> BaseVerticalGRFConfig:
        """Active configuration for this GRF instance."""
        return self._grf_config

    @property
    def channel_names(self) -> tuple[str, ...]:
        """Channel names in the order emitted by :meth:`read`."""
        return self._grf_config.channel_names

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

    def read_signals(self, signals: Iterable[str]) -> tuple[VerticalGRFSample, dict[str, float]]:
        """Return the latest sample with canonical GRF signals."""
        sample = self.read()
        values: dict[str, float] = {}
        for name in signals:
            value = self._map_signal(sample, name)
            if value is not None:
                values[name] = value
        return sample, values

    def zero(self) -> None:
        """Optional re-zero hook."""
        return None

    def _handle_sample(
        self,
        sample: VerticalGRFSample | None,
        *,
        fresh: bool,
        fallback_factory: Callable[[float], object] | None = None,
    ) -> VerticalGRFSample:

        def fallback(timestamp: float) -> VerticalGRFSample:
            return VerticalGRFSample(
                timestamp=timestamp,
                forces_newton=tuple(0.0 for _ in self.channel_names),
            )

        factory: Callable[[float], object] = fallback_factory or fallback
        try:
            result = super()._handle_sample(sample, fresh=fresh, fallback_factory=factory)
        except SensorStaleDataError as exc:
            raise GRFStaleDataError(str(exc)) from exc
        return cast(VerticalGRFSample, result)

    @classmethod
    def _validate_config(cls, config: BaseSensorConfig) -> BaseSensorConfig:
        if not isinstance(config, BaseVerticalGRFConfig):
            raise TypeError("BaseVerticalGRF requires a BaseVerticalGRFConfig instance")
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

    def _map_signal(self, sample: VerticalGRFSample, name: str) -> float | None:
        """Return canonical GRF signals from a sample when available."""
        if name == "vertical_grf_ipsi_N":
            return self._resolve_channel(sample, 0)
        return None

    def _resolve_channel(self, sample: VerticalGRFSample, index: int) -> float | None:
        """Return a channel value by index when present."""
        try:
            return float(sample.forces_newton[index])
        except (IndexError, TypeError, ValueError):
            return None
