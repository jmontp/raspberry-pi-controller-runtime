"""Deterministic mock of vertical GRF sensor."""

from __future__ import annotations

import itertools
import time
from dataclasses import dataclass
from typing import Iterable

from .base import BaseVerticalGRF, VerticalGRFSample


def _default_waveform() -> Iterable[VerticalGRFSample]:
    t0 = time.monotonic()
    for i in itertools.count():
        t = time.monotonic() - t0
        load = 400 + 40 * ((-1) ** (i // 50))
        yield VerticalGRFSample(timestamp=t, forces_newton=(float(load),))


@dataclass(slots=True)
class MockVerticalGRF(BaseVerticalGRF):
    generator: Iterable[VerticalGRFSample] | None = None

    def __post_init__(self) -> None:
        self._iterator = iter(self.generator or _default_waveform())

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def read(self) -> VerticalGRFSample:
        try:
            return next(self._iterator)
        except StopIteration as exc:  # pragma: no cover - test harness
            raise RuntimeError("Mock generator exhausted") from exc
