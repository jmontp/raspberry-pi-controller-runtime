"""Utilities for persisting runtime diagnostics artifacts."""

from __future__ import annotations

import logging
import math
import shutil
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable, Mapping, Sequence

from .diagnostics import CSVDiagnosticsSink, DiagnosticsSink
from ..sensors.base import BaseSensor

LOGGER = logging.getLogger(__name__)


def _safe_profile_name(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in name)


def _compute_histogram(samples: Sequence[float], bins: int) -> tuple[tuple[float, ...], tuple[int, ...]]:
    if not samples:
        return (), ()
    minimum = min(samples)
    maximum = max(samples)
    if math.isclose(minimum, maximum):
        return (minimum, maximum), (len(samples),)
    bin_count = max(1, min(bins, len(samples)))
    span = maximum - minimum
    width = span / bin_count if span else 1.0
    edges = [minimum + i * width for i in range(bin_count)]
    edges.append(maximum)
    counts = [0 for _ in range(bin_count)]
    for value in samples:
        if value >= maximum:
            index = bin_count - 1
        else:
            index = int((value - minimum) / width)
        counts[index] += 1
    return tuple(edges), tuple(counts)


def _write_histogram_csv(path: Path, samples: Sequence[float], bins: int) -> None:
    edges, counts = _compute_histogram(samples, bins)
    _write_histogram_counts(path, edges, counts)


def _write_histogram_counts(path: Path, edges: Sequence[float], counts: Sequence[int]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        handle.write("bin_start,bin_end,count\n")
        if not edges:
            return
        if len(counts) == 1:
            handle.write(f"{edges[0]},{edges[-1]},{counts[0]}\n")
            return
        for idx, count in enumerate(counts):
            handle.write(f"{edges[idx]},{edges[idx + 1]},{count}\n")


def _write_series(path: Path, samples: Iterable[float], header: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        handle.write(f"{header}\n")
        for value in samples:
            handle.write(f"{value}\n")


def _render_histogram_png(path: Path, samples: Sequence[float], bins: int) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - optional dependency
        LOGGER.warning("Matplotlib unavailable (%s); histogram PNG skipped", exc)
        return

    if not samples:
        return

    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.hist(list(samples), bins=bins, edgecolor="#1a4a7a", color="#3182ce")
    ax.set_title("Runtime Loop Rate Histogram")
    ax.set_xlabel("Loop rate (Hz)")
    ax.set_ylabel("Count")
    ax.grid(True, axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    try:
        fig.savefig(path, dpi=150, format="png")
    except Exception as exc:  # pragma: no cover - runtime guard
        LOGGER.warning("Failed to persist loop rate histogram PNG: %s", exc)
    finally:
        plt.close(fig)


@dataclass(slots=True)
class DiagnosticsArtifacts:
    """Manage filesystem layout and exports for runtime diagnostics."""

    enabled: bool
    run_dir: Path
    sensors_dir: Path
    running_stats_dir: Path
    sink: DiagnosticsSink | None
    _loop_periods: list[float] = field(default_factory=list, init=False)
    _last_tick_time: float | None = field(default=None, init=False)

    @classmethod
    def create(
        cls,
        *,
        root: Path | None,
        profile_path: Path,
        profile_name: str,
        enable_csv: bool = True,
    ) -> "DiagnosticsArtifacts":
        """Initialise a diagnostics run directory and CSV sink."""
        if root is None:
            return cls(
                enabled=False,
                run_dir=Path(),
                sensors_dir=Path(),
                running_stats_dir=Path(),
                sink=None,
            )

        timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
        run_dir = root / f"{_safe_profile_name(profile_name)}_{timestamp}"
        sensors_dir = run_dir / "sensors"
        running_stats_dir = run_dir / "running_stats"

        running_stats_dir.mkdir(parents=True, exist_ok=True)
        sensors_dir.mkdir(parents=True, exist_ok=True)

        sink: DiagnosticsSink | None = None
        if enable_csv:
            sink = CSVDiagnosticsSink(run_dir / "loop.csv")

        try:
            shutil.copy2(profile_path, run_dir / profile_path.name)
        except Exception as exc:  # pragma: no cover - best effort
            LOGGER.warning("Failed to copy profile '%s' into diagnostics dir: %s", profile_path, exc)

        return cls(
            enabled=True,
            run_dir=run_dir,
            sensors_dir=sensors_dir,
            running_stats_dir=running_stats_dir,
            sink=sink,
        )

    def diagnostics_enabled(self) -> bool:
        """Return whether filesystem artifacts are being recorded."""
        return self.enabled

    def record_scheduler_tick(self) -> None:
        """Track scheduler period using a high-resolution timestamp."""
        if not self.enabled:
            return
        now = time.perf_counter()
        if self._last_tick_time is not None:
            self._loop_periods.append(now - self._last_tick_time)
        self._last_tick_time = now

    def finalise(self, sensors: Mapping[str, BaseSensor]) -> None:
        """Persist aggregated scheduler and sensor diagnostics."""
        if not self.enabled:
            return
        self._write_scheduler_metrics()
        self._write_sensor_metrics(sensors)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _write_scheduler_metrics(self) -> None:
        periods = tuple(self._loop_periods)
        rates = [1.0 / p for p in periods if p > 0]
        if not rates:
            _write_histogram_csv(self.running_stats_dir / "loop_rate_histogram.csv", (), bins=10)
            return
        _write_histogram_csv(self.running_stats_dir / "loop_rate_histogram.csv", rates, bins=10)
        _write_series(self.running_stats_dir / "loop_rates_hz.csv", rates, header="hz")
        _render_histogram_png(self.running_stats_dir / "loop_rate_histogram.png", rates, bins=10)

    def _write_sensor_metrics(self, sensors: Mapping[str, BaseSensor]) -> None:
        for alias, sensor in sensors.items():
            diagnostics = getattr(sensor, "diagnostics", None)
            if diagnostics is None:
                continue
            target = self.sensors_dir / alias
            target.mkdir(parents=True, exist_ok=True)
            periods = getattr(diagnostics, "recent_sample_periods", ()) or ()
            rates = getattr(diagnostics, "recent_sample_rates", ()) or ()
            _write_series(target / "sample_periods.csv", periods, header="seconds")
            period_edges, period_counts = sensor.sample_period_histogram(bins=10)
            _write_histogram_counts(target / "sample_period_histogram.csv", period_edges, period_counts)
            _write_series(target / "sample_rates_hz.csv", rates, header="hz")
            rate_edges, rate_counts = sensor.sample_rate_histogram(bins=10)
            _write_histogram_counts(target / "sample_rate_histogram.csv", rate_edges, rate_counts)
            summary_lines = [
                f"total_samples={getattr(diagnostics, 'total_samples', 0)}",
                f"stale_samples={getattr(diagnostics, 'stale_samples', 0)}",
                f"hz_estimate={getattr(diagnostics, 'hz_estimate', None)}",
                f"hz_min={getattr(diagnostics, 'hz_min', None)}",
                f"hz_max={getattr(diagnostics, 'hz_max', None)}",
                f"hz_mean={getattr(diagnostics, 'hz_mean', None)}",
            ]
            try:
                (target / "summary.txt").write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
            except Exception as exc:  # pragma: no cover - best effort
                LOGGER.warning("Failed to write sensor summary for '%s': %s", alias, exc)
