"""Schedulers for controlling the runtime loop cadence."""

from __future__ import annotations

import abc
import contextlib
import logging
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Iterator, Mapping, Protocol

LOGGER = logging.getLogger(__name__)


class SchedulerFault(RuntimeError):  # noqa: N818
    """Raised when the scheduler detects sustained timing overruns."""


class BaseScheduler(abc.ABC):
    """Abstract loop scheduler that yields on each control tick."""

    @abc.abstractmethod
    def ticks(self) -> Iterator[float]:
        """Yield the time since the loop started for each tick."""

    def close(self) -> None:
        """Optional cleanup hook executed when leaving the scheduler context."""
        return None

    def metrics(self) -> Mapping[str, float]:  # pragma: no cover - default no-op
        """Return timing metrics collected by the scheduler."""
        return {}

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
    jitter_budget_s: float = 0.0
    max_overruns_before_fault: int = 3
    fault_on_overrun: bool = False
    overrun_warning_cooldown_s: float = 1.0

    def __post_init__(self) -> None:
        """Normalise configuration and initialise jitter tracking."""
        if self.frequency_hz <= 0:
            raise ValueError("frequency_hz must be positive")
        if self.jitter_budget_s < 0:
            raise ValueError("jitter_budget_s must be non-negative")
        self._dt_target = 1.0 / float(self.frequency_hz)
        self._overrun_threshold = max(0, int(self.max_overruns_before_fault))
        self._overrun_streak = 0
        self._tick_count = 0
        self._last_dt_actual: float | None = None
        self._last_jitter: float | None = None
        self._jitter_min: float | None = None
        self._jitter_max: float | None = None
        self._jitter_sum = 0.0
        self._jitter_count = 0
        self._last_overrun_warning: float | None = None

    def ticks(self) -> Iterator[float]:
        """Yield elapsed time while sleeping to maintain the requested frequency."""
        start = time.monotonic()
        count = 0
        previous = start
        while True:
            target = start + count * self._dt_target
            now = time.monotonic()
            sleep_time = target - now
            if sleep_time > 0:
                time.sleep(sleep_time)
                now = time.monotonic()

            actual_dt = now - previous
            previous = now
            self._record_tick(actual_dt)

            elapsed = now - start
            yield elapsed

            count += 1
            if self.duration_s is not None and elapsed >= self.duration_s:
                break

    def metrics(self) -> Mapping[str, float]:
        """Return the latest scheduler timing metrics."""
        jitter_min = self._jitter_min if self._jitter_min is not None else 0.0
        jitter_max = self._jitter_max if self._jitter_max is not None else 0.0
        jitter_mean = self._jitter_sum / self._jitter_count if self._jitter_count else 0.0
        dt_actual = self._last_dt_actual if self._last_dt_actual is not None else self._dt_target
        jitter_latest = self._last_jitter if self._last_jitter is not None else 0.0
        return {
            "dt_target_s": float(self._dt_target),
            "dt_actual_s": float(dt_actual),
            "jitter_latest_s": float(jitter_latest),
            "jitter_min_s": float(jitter_min),
            "jitter_max_s": float(jitter_max),
            "jitter_mean_s": float(jitter_mean),
            "overrun_streak": float(self._overrun_streak),
            "ticks": float(self._tick_count),
        }

    def _record_tick(self, actual_dt: float) -> None:
        """Update jitter statistics and enforce overrun policies."""
        self._tick_count += 1
        self._last_dt_actual = actual_dt
        jitter = actual_dt - self._dt_target
        self._last_jitter = jitter
        self._update_jitter_extrema(jitter)

        if self._overrun_threshold and actual_dt > (self._dt_target + self.jitter_budget_s):
            self._overrun_streak += 1
            if self._overrun_streak >= self._overrun_threshold:
                if self.fault_on_overrun:
                    raise SchedulerFault(
                        f"Scheduler detected {self._overrun_streak} consecutive overruns"
                    )
                self._emit_overrun_warning(actual_dt)
                self._overrun_streak = 0
        else:
            self._overrun_streak = 0

    def _update_jitter_extrema(self, jitter: float) -> None:
        """Track jitter distribution statistics."""
        self._jitter_count += 1
        self._jitter_sum += jitter
        if self._jitter_min is None or jitter < self._jitter_min:
            self._jitter_min = jitter
        if self._jitter_max is None or jitter > self._jitter_max:
            self._jitter_max = jitter

    def _emit_overrun_warning(self, actual_dt: float) -> None:
        """Log a warning when the overrun threshold is exceeded."""
        if self._overrun_threshold <= 0:
            return
        cooldown = max(0.0, float(self.overrun_warning_cooldown_s))
        now = time.monotonic()
        if self._last_overrun_warning is not None and (now - self._last_overrun_warning) < cooldown:
            return
        self._last_overrun_warning = now
        LOGGER.warning(
            "Scheduler experienced %d consecutive overruns "
            "(dt=%.6f s, target=%.6f s, jitter_budget=%.6f s); continuing execution",
            self._overrun_threshold,
            actual_dt,
            self._dt_target,
            self.jitter_budget_s,
        )


@dataclass(slots=True)
class SoftRealtimeScheduler(BaseScheduler):
    """Wrap a soft-realtime loop implementation behind the scheduler interface."""

    frequency_hz: float
    loop_class: Callable[..., "SoftRealtimeLoop"]

    def __post_init__(self) -> None:
        """Initialise the loop placeholder prior to lazy construction."""
        self._loop: SoftRealtimeLoop | None = None

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


class SoftRealtimeLoop(Protocol, Iterable[float]):
    """Protocol describing the soft-realtime loop contract consumed by the scheduler."""

    def __enter__(self) -> "SoftRealtimeLoop":
        """Enter the loop context."""
        ...

    def __exit__(self, exc_type, exc, tb) -> None:
        """Leave the loop context, performing any cleanup."""
        ...

    def __iter__(self) -> Iterator[float]:
        """Iterate over elapsed tick times emitted by the loop."""
        ...

    def stop(self) -> None:  # pragma: no cover - optional for some implementations
        """Request the loop to stop producing further ticks."""
        ...
