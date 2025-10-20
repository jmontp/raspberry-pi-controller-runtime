"""Mock IMU emitting a predefined sequence of samples."""

from __future__ import annotations

import random
from dataclasses import dataclass, field, replace
from typing import Iterable, Sequence

from .base import BaseIMU, BaseIMUConfig, IMUSample


def _default_port_map() -> dict[str, str]:
    """Generate a mock port map covering all hardware canonical features."""
    return {name: f"mock://{name}" for name in BaseIMU.HARDWARE_FEATURES}


def _default_samples(
    config: BaseIMUConfig,
    *,
    amplitude: float = 0.2,
) -> Sequence[IMUSample]:
    """Create a small sequence of random IMU samples for fall-back usage."""
    rng = random.Random(0)
    features = tuple(config.port_map.keys()) or BaseIMU.HARDWARE_FEATURES
    samples: list[IMUSample] = []
    for idx in range(10):
        timestamp = idx * 0.01
        values = {name: rng.uniform(-amplitude, amplitude) for name in features}
        samples.append(IMUSample(timestamp=timestamp, values=values))
    return tuple(samples)


@dataclass(slots=True)
class MockIMU(BaseIMU):
    """Finite sample IMU for deterministic testing."""

    samples: Iterable[IMUSample] | None = None
    loop: bool = False
    config_override: BaseIMUConfig | None = None
    _samples: Sequence[IMUSample] = field(init=False, repr=False)
    _index: int = field(init=False, repr=False)

    def __post_init__(self) -> None:
        """Initialise the base class and materialise the sample sequence."""
        cfg = self.config_override
        if cfg is None:
            cfg = BaseIMUConfig(port_map=_default_port_map())
        elif not cfg.port_map:
            cfg = replace(cfg, port_map=_default_port_map())
        BaseIMU.__init__(self, cfg)
        if self.samples is None:
            materialized = tuple(_default_samples(self.config))
        else:
            materialized = tuple(self.samples)
            if not materialized:
                raise ValueError("MockIMU requires at least one sample when samples is provided")
        self._samples = materialized
        self._index = 0

    def start(self) -> None:
        """No-op start hook for compatibility with context manager usage."""
        return None

    def stop(self) -> None:
        """No-op stop hook for compatibility with context manager usage."""
        return None

    def await_startup_sample(
        self,
        signals: Iterable[str] | None = None,
        *,
        timeout_s: float | None = None,
    ) -> None:
        """Mock sensors are ready immediately; no blocking required."""
        return None

    def read(self) -> IMUSample:
        """Return the next mock sample, optionally cycling when ``loop`` is set."""
        if not self._samples:
            raise RuntimeError("MockIMU sample sequence is empty")
        if self._index >= len(self._samples):
            if not self.loop:
                raise RuntimeError("MockIMU sample sequence exhausted")
            self._index = 0
        sample = self._samples[self._index]
        self._index += 1
        return self._handle_sample(sample, fresh=True)
