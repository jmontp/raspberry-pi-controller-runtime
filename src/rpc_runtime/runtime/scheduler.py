"""Schedulers for controlling the runtime loop cadence."""

from __future__ import annotations

import abc
import contextlib
import time
from dataclasses import dataclass
from typing import Callable, Iterator


class BaseScheduler(abc.ABC):
    """Abstract loop scheduler that yields on each control tick."""

    @abc.abstractmethod
    def ticks(self) -> Iterator[float]:
        """Yield the time since the loop started for each tick."""

    def close(self) -> None:
        """Optional cleanup hook executed when leaving the scheduler context."""
        return None

    def __enter__(self) -> "BaseScheduler":
        """Enter the scheduler context."""
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Exit the scheduler context, performing any cleanup."""
        self.close()


@dataclass(slots=True)
class SimpleScheduler(BaseScheduler):
    """Sleep-based scheduler useful for tests and non-realtime execution."""

    frequency_hz: float
    duration_s: float | None = None

    def ticks(self) -> Iterator[float]:
        """Yield elapsed time while sleeping to maintain the requested frequency."""
        dt = 1.0 / self.frequency_hz
        start = time.monotonic()
        count = 0
        while True:
            target = start + count * dt
            now = time.monotonic()
            sleep_time = target - now
            if sleep_time > 0:
                time.sleep(sleep_time)
            yield time.monotonic() - start
            count += 1
            if self.duration_s is not None and (time.monotonic() - start) >= self.duration_s:
                break


@dataclass(slots=True)
class SoftRealtimeScheduler(BaseScheduler):
    """Wrap a soft-realtime loop implementation behind the scheduler interface."""

    frequency_hz: float
    loop_class: Callable[..., object]

    def __post_init__(self) -> None:
        """Initialise the loop placeholder prior to lazy construction."""
        self._loop = None

    def ticks(self) -> Iterator[float]:
        """Yield elapsed times from the underlying soft-realtime implementation."""
        loop = self.loop_class(dt=1.0 / self.frequency_hz)
        self._loop = loop
        with contextlib.ExitStack() as stack:
            stack.enter_context(loop)
            for t in loop:
                yield t

    def close(self) -> None:
        """Call `stop()` on the underlying loop when available."""
        loop = self._loop
        if loop is None:
            return
        if hasattr(loop, "stop"):
            loop.stop()
        self._loop = None

