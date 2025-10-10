"""Deterministic mock of vertical GRF sensor."""

from __future__ import annotations

import random
import time
from dataclasses import dataclass, field
from typing import Iterable, Sequence

from .base import BaseVerticalGRF, BaseVerticalGRFConfig, VerticalGRFSample


def _default_waveform(config: BaseVerticalGRFConfig) -> Sequence[VerticalGRFSample]:
    rng = random.Random(0)
    t0 = time.monotonic()
    channels = len(config.channel_names)
    samples: list[VerticalGRFSample] = []
    for _ in range(100):
        t = time.monotonic() - t0
        forces = tuple(400.0 + rng.uniform(-40.0, 40.0) for _ in range(channels))
        samples.append(VerticalGRFSample(timestamp=t, forces_newton=forces))
    return tuple(samples)


@dataclass(slots=True)
class MockVerticalGRF(BaseVerticalGRF):
    """Yield vertical GRF samples from a supplied generator."""

    generator: Iterable[VerticalGRFSample] | None = None
    loop: bool = True
    config_override: BaseVerticalGRFConfig | None = None
    _samples: Sequence[VerticalGRFSample] = field(init=False, repr=False)
    _index: int = field(init=False, repr=False)

    def __post_init__(self) -> None:
        BaseVerticalGRF.__init__(self, self.config_override)
        if self.generator is None:
            self._samples = _default_waveform(self.config)
        else:
            materialized = tuple(self.generator)
            if not materialized:
                raise ValueError("MockVerticalGRF requires at least one sample")
            self._samples = materialized
        self._index = 0

    def start(self) -> None:
        """No-op start hook matching the base interface."""
        return None

    def stop(self) -> None:
        """No-op stop hook matching the base interface."""
        return None

    def read(self) -> VerticalGRFSample:
        """Return the next sample, applying staleness policy if needed."""
        if not self._samples:
            raise RuntimeError("MockVerticalGRF sample sequence empty")
        if self._index >= len(self._samples):
            if not self.loop:
                raise RuntimeError("MockVerticalGRF sample sequence exhausted")
            self._index = 0
        sample = self._samples[self._index]
        self._index += 1
        return self._handle_sample(sample, fresh=True)
