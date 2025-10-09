"""Mock IMU emitting a predefined sequence of samples."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Iterable

from .base import BaseIMU, IMUSample


@dataclass(slots=True)
class MockIMU(BaseIMU):
    samples: Iterable[IMUSample]
    loop: bool = False
    _queue: Deque[IMUSample] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._queue = deque(self.samples)
        if not self._queue:
            raise ValueError("MockIMU requires at least one sample")

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def read(self) -> IMUSample:
        if not self._queue:
            raise RuntimeError("MockIMU sample queue exhausted")
        sample = self._queue.popleft()
        if self.loop:
            self._queue.append(sample)
        return sample
