"""Utilities for persisting runtime diagnostics artifacts."""

from __future__ import annotations

import logging
import math
import shutil
import statistics
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
    additional_dir: Path
    sensors_dir: Path
    running_stats_dir: Path
    sink: DiagnosticsSink | None
    profile_name: str | None = None
    timestamp_str: str | None = None
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
                additional_dir=Path(),
                sensors_dir=Path(),
                running_stats_dir=Path(),
                sink=None,
            )

        timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
        run_dir = root / f"{_safe_profile_name(profile_name)}_{timestamp}"
        additional_dir = run_dir / "additional_info"
        sensors_dir = additional_dir / "sensors"
        running_stats_dir = additional_dir / "running_stats"

        running_stats_dir.mkdir(parents=True, exist_ok=True)
        sensors_dir.mkdir(parents=True, exist_ok=True)

        sink: DiagnosticsSink | None = None
        if enable_csv:
            sink = CSVDiagnosticsSink(run_dir / "data.csv")

        try:
            shutil.copy2(profile_path, additional_dir / profile_path.name)
        except Exception as exc:  # pragma: no cover - best effort
            LOGGER.warning("Failed to copy profile '%s' into diagnostics dir: %s", profile_path, exc)

        return cls(
            enabled=True,
            run_dir=run_dir,
            additional_dir=additional_dir,
            sensors_dir=sensors_dir,
            running_stats_dir=running_stats_dir,
            sink=sink,
            profile_name=profile_name,
            timestamp_str=timestamp,
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
        loop_stats = self._write_scheduler_metrics()
        sensor_summaries = self._write_sensor_metrics(sensors)
        self._write_report(loop_stats, sensor_summaries)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _write_scheduler_metrics(self) -> dict[str, float | int | None]:
        periods = tuple(self._loop_periods)
        rates = [1.0 / p for p in periods if p > 0]
        if not rates:
            _write_histogram_csv(self.running_stats_dir / "loop_rate_histogram.csv", (), bins=10)
            return {
                "ticks": 0,
                "duration_s": 0.0,
                "min_rate_hz": None,
                "mean_rate_hz": None,
                "max_rate_hz": None,
                "min_period_s": None,
                "mean_period_s": None,
                "max_period_s": None,
            }
        _write_histogram_csv(self.running_stats_dir / "loop_rate_histogram.csv", rates, bins=10)
        _write_series(self.running_stats_dir / "loop_rates_hz.csv", rates, header="hz")
        _render_histogram_png(self.running_stats_dir / "loop_rate_histogram.png", rates, bins=10)
        duration = sum(periods)
        min_period = min(periods) if periods else None
        max_period = max(periods) if periods else None
        mean_period = statistics.mean(periods) if periods else None
        min_rate = min(rates) if rates else None
        max_rate = max(rates) if rates else None
        mean_rate = statistics.mean(rates) if rates else None
        return {
            "ticks": len(periods),
            "duration_s": duration,
            "min_rate_hz": min_rate,
            "mean_rate_hz": mean_rate,
            "max_rate_hz": max_rate,
            "min_period_s": min_period,
            "mean_period_s": mean_period,
            "max_period_s": max_period,
        }

    def _write_sensor_metrics(self, sensors: Mapping[str, BaseSensor]) -> list[dict[str, object]]:
        summaries: list[dict[str, object]] = []
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
            rate_plot_path = None
            try:
                rate_plot_path = self._render_rate_plot(alias, rates, periods, target)
            except Exception as exc:  # pragma: no cover - best effort
                LOGGER.warning("Failed to render rate plot for '%s': %s", alias, exc)
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
            summaries.append(
                {
                    "alias": alias,
                    "total_samples": getattr(diagnostics, "total_samples", 0),
                    "stale_samples": getattr(diagnostics, "stale_samples", 0),
                    "hz_estimate": getattr(diagnostics, "hz_estimate", None),
                    "hz_min": getattr(diagnostics, "hz_min", None),
                    "hz_max": getattr(diagnostics, "hz_max", None),
                    "hz_mean": getattr(diagnostics, "hz_mean", None),
                    "max_period_s": max(periods) if periods else None,
                    "period_hist": (period_edges, period_counts),
                    "rate_hist": (rate_edges, rate_counts),
                    "periods_csv": target / "sample_periods.csv",
                    "rates_csv": target / "sample_rates_hz.csv",
                    "period_hist_csv": target / "sample_period_histogram.csv",
                    "rate_hist_csv": target / "sample_rate_histogram.csv",
                    "summary_path": target / "summary.txt",
                    "config": getattr(sensor, "config", None),
                    "rate_plot": rate_plot_path,
                    "drop_intervals": self._compute_drop_intervals(periods, getattr(sensor, "config", None)),
                }
            )
        return summaries

    def _write_report(
        self,
        loop_stats: dict[str, float | int | None],
        sensor_summaries: list[dict[str, object]],
    ) -> None:
        report_path = self.run_dir / "report.md"
        lines: list[str] = []
        profile_display = self.profile_name or "runtime"
        timestamp = self.timestamp_str or ""
        lines.append(f"# Run Report – {profile_display} ({timestamp})")
        lines.append("")
        lines.append("| Metric | Value |")
        lines.append("|--------|-------|")
        duration = loop_stats.get("duration_s")
        ticks = loop_stats.get("ticks")
        lines.append(
            f"| Duration (s) | {duration:.2f} |" if isinstance(duration, (int, float)) else "| Duration (s) | N/A |"
        )
        lines.append(f"| Ticks | {ticks} |")
        lines.append(f"| Diagnostics Path | `{self.run_dir}` |")
        lines.append("| Data CSV | [data.csv](data.csv) |")
        lines.append("")

        lines.append("## How to Read This Report")
        lines.append("")
        lines.append(
            "- **Loop Performance** summarises overall scheduler cadence (smaller period ⇒ higher rate). "
            "Min/Max rows refer to fastest/slowest control cycles."
        )
        lines.append(
            "- **Sensors** tables show sample counts and observed bandwidth. "
            "Histograms indicate how often each sensor reported within a given period bin."
        )
        lines.append("- ⚠️ Alerts call out potential problems (e.g., dropouts exceeding configured tolerances).")
        lines.append("")

        lines.append("## Loop Performance")
        lines.append("")
        lines.append("| Condition | Period (ms) | Rate (Hz) |")
        lines.append("|-----------|-------------|-----------|")
        min_period = loop_stats.get("min_period_s")
        mean_period = loop_stats.get("mean_period_s")
        max_period = loop_stats.get("max_period_s")
        min_rate = loop_stats.get("min_rate_hz")
        mean_rate = loop_stats.get("mean_rate_hz")
        max_rate = loop_stats.get("max_rate_hz")

        def _fmt(value: float | None, scale: float = 1.0, decimals: int = 2) -> str:
            if value is None:
                return "N/A"
            return f"{value * scale:.{decimals}f}"

        lines.append(f"| Fastest (min period) | {_fmt(min_period, 1000.0)} | {_fmt(max_rate)} |")
        lines.append(f"| Typical (mean period) | {_fmt(mean_period, 1000.0)} | {_fmt(mean_rate)} |")
        lines.append(f"| Slowest (max period) | {_fmt(max_period, 1000.0)} | {_fmt(min_rate)} |")
        lines.append("")

        histogram_path = self.running_stats_dir / "loop_rate_histogram.png"
        if histogram_path.exists():
            rel_histogram = histogram_path.relative_to(self.run_dir)
            lines.append(f"![Loop Rate Histogram]({rel_histogram.as_posix()})")
            lines.append("")

        lines.append("## Sensors")
        lines.append("")

        alerts: list[str] = []
        for summary in sorted(sensor_summaries, key=lambda s: s["alias"]):
            alias = summary["alias"]
            lines.append(f"### {alias}")
            lines.append("")
            lines.append("| Metric | Value |")
            lines.append("|--------|-------|")
            lines.append(f"| Total Samples | {summary['total_samples']} |")
            stale = summary["stale_samples"]
            lines.append(f"| Stale Samples | {stale} |")
            hz_mean = summary["hz_mean"]
            hz_min = summary["hz_min"]
            hz_max = summary["hz_max"]
            lines.append(f"| Mean Rate (Hz) | {_fmt(hz_mean)} |")
            lines.append(f"| Min Rate (Hz) | {_fmt(hz_min)} |")
            lines.append(f"| Max Rate (Hz) | {_fmt(hz_max)} |")
            max_period = summary["max_period_s"]
            lines.append(f"| Longest Gap (s) | {_fmt(max_period)} |")
            lines.append("")

            period_edges, period_counts = summary["period_hist"]
            if period_edges:
                lines.append("| Period Bin (s) | Count |")
                lines.append("|----------------|-------|")
                for idx, count in enumerate(period_counts):
                    start = period_edges[idx]
                    end = period_edges[idx + 1] if idx + 1 < len(period_edges) else period_edges[idx]
                    lines.append(f"| {start:.3f}–{end:.3f} | {count} |")
            lines.append("")

            lines.append(
                "Links: "
                f"[periods]({summary['periods_csv'].relative_to(self.run_dir).as_posix()}), "
                f"[rates]({summary['rates_csv'].relative_to(self.run_dir).as_posix()}), "
                f"[period_hist]({summary['period_hist_csv'].relative_to(self.run_dir).as_posix()}), "
                f"[rate_hist]({summary['rate_hist_csv'].relative_to(self.run_dir).as_posix()})"
            )
            lines.append("")

            if summary.get("rate_plot") is not None and summary["rate_plot"].exists():
                rel_plot = summary["rate_plot"].relative_to(self.run_dir)
                lines.append(f"![Sample Rate Timeline]({rel_plot.as_posix()})")
                lines.append("")

            drop_intervals = summary.get("drop_intervals") or []
            if drop_intervals:
                threshold = getattr(summary.get("config"), "max_stale_time_s", None)
                if threshold is not None:
                    lines.append(f"Dropout intervals (period > {threshold:.2f}s):")
                else:
                    lines.append("Dropout intervals:")
                for start, end, duration in drop_intervals:
                    lines.append(f"- {start:.2f}s → {end:.2f}s (duration {duration:.2f}s)")
            lines.append("")

            config = summary.get("config")
            if config is not None:
                max_stale_time = getattr(config, "max_stale_time_s", None)
                if max_stale_time is not None and isinstance(max_period, (int, float)) and max_period > max_stale_time:
                    alerts.append(
                        f"⚠️ `{alias}` observed gap {max_period:.2f}s exceeding configured "
                        f"max_stale_time_s={max_stale_time:.2f}s."
                    )
            if isinstance(stale, int) and stale > 0:
                alerts.append(f"⚠️ `{alias}` reported {stale} stale samples.")

        lines.append("## Alerts")
        lines.append("")
        if alerts:
            for alert in alerts:
                lines.append(f"- {alert}")
        else:
            lines.append("- ✅ No notable diagnostics alerts recorded.")
        lines.append("")

        report_path.write_text("\n".join(lines), encoding="utf-8")

    def _render_rate_plot(
        self,
        alias: str,
        rates: Sequence[float],
        periods: Sequence[float],
        target_dir: Path,
    ) -> Path | None:
        if not rates or not periods:
            return None
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception:
            return None

        times: list[float] = []
        cumulative = 0.0
        for period in periods:
            cumulative += period
            times.append(cumulative)

        path = target_dir / "sample_rate_timeseries.png"
        fig, ax = plt.subplots(figsize=(6, 2.5))
        ax.plot(times[: len(rates)], rates, color="#2a4365", linewidth=1.2)
        ax.set_title(f"{alias} Sample Rate")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Rate (Hz)")
        ax.grid(True, linestyle="--", alpha=0.4)
        fig.tight_layout()
        try:
            fig.savefig(path, dpi=150, format="png")
        finally:
            plt.close(fig)
        return path

    def _compute_drop_intervals(
        self,
        periods: Sequence[float],
        config: object,
    ) -> list[tuple[float, float, float]]:
        if not periods:
            return []
        threshold = getattr(config, "max_stale_time_s", None)
        if threshold is None:
            return []
        intervals: list[tuple[float, float, float]] = []
        cumulative = 0.0
        start = None
        for period in periods:
            cumulative += period
            if period > threshold:
                if start is None:
                    start = cumulative - period
            else:
                if start is not None:
                    end = cumulative
                    intervals.append((start, end, end - start))
                    start = None
        if start is not None:
            end = cumulative
            intervals.append((start, end, end - start))
        return intervals
