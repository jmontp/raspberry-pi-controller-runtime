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
from typing import TYPE_CHECKING, Iterable, Mapping, Sequence

import numpy as np

from ..actuators.base import TorqueCommand
from ..sensors.base import BaseSensor
from .diagnostics import CSVDiagnosticsSink, DiagnosticsSink

LOGGER = logging.getLogger(__name__)

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..config.models import SignalRoute

GLOSSARY_ENTRIES: tuple[tuple[str, str], ...] = (
    (
        "Sensor",
        "Hardware adapter providing the data for this signal.",
    ),
    (
        "Fresh Sample Coverage",
        "Share of scheduler ticks where the sensor produced a fresh reading; parentheses show the raw fresh-count.",
    ),
    (
        "Max Stale Run (ticks)",
        "Longest streak of consecutive stale ticks observed during the run.",
    ),
    (
        "Missed Samples",
        "Scheduler ticks without a fresh reading (estimated as total ticks minus fresh samples).",
    ),
    (
        "Delivered Fresh Rate (Hz)",
        "Effective fresh-data bandwidth after accounting for missed ticks (coverage × loop rate, or samples ÷ run time).",
    ),
    (
        "Longest Gap (s)",
        "Longest observed interval between two fresh samples within the diagnostics window.",
    ),
    (
        "Dropout intervals",
        "Time spans where the inter-sample period exceeded the configured `max_stale_time_s` for the sensor.",
    ),
    (
        "Data Availability Timeline",
        "Timeline highlighting stale/missing events as red bars; blank regions correspond to fresh readings.",
    ),
)

INPUT_COLOR = "#1f77b4"
OUTPUT_COLOR = "#d62728"

MEASUREMENT_BASE_COLORS: dict[str, str] = {
    "hip_flexion": "#1f77b4",
    "thigh_sagittal": "#ff7f0e",
    "knee_flexion": "#2ca02c",
    "shank_sagittal": "#d62728",
    "ankle_dorsiflexion": "#9467bd",
    "foot_sagittal": "#8c564b",
    "trunk_sagittal": "#17becf",
    "vertical_grf": "#bcbd22",
}

SIDE_LIGHTEN: dict[str, float] = {
    "ipsi": 0.0,
    "contra": 0.2,
}
VELOCITY_LIGHTEN = 0.35
FORCE_LIGHTEN = 0.15

ANGLE_SIGNALS = [
    "hip_flexion_angle_ipsi_rad",
    "knee_flexion_angle_ipsi_rad",
    "ankle_dorsiflexion_angle_ipsi_rad",
    "hip_flexion_angle_contra_rad",
    "knee_flexion_angle_contra_rad",
    "ankle_dorsiflexion_angle_contra_rad",
]

VELOCITY_SIGNALS = [
    "hip_flexion_velocity_ipsi_rad_s",
    "knee_flexion_velocity_ipsi_rad_s",
    "ankle_dorsiflexion_velocity_ipsi_rad_s",
    "hip_flexion_velocity_contra_rad_s",
    "knee_flexion_velocity_contra_rad_s",
    "ankle_dorsiflexion_velocity_contra_rad_s",
]

SEGMENT_ANGLE_SIGNALS = [
    "trunk_sagittal_angle_rad",
    "thigh_sagittal_angle_ipsi_rad",
    "shank_sagittal_angle_ipsi_rad",
    "foot_sagittal_angle_ipsi_rad",
    "thigh_sagittal_angle_contra_rad",
    "shank_sagittal_angle_contra_rad",
    "foot_sagittal_angle_contra_rad",
]

SEGMENT_VELOCITY_SIGNALS = [
    "trunk_sagittal_velocity_rad_s",
    "thigh_sagittal_velocity_ipsi_rad_s",
    "shank_sagittal_velocity_ipsi_rad_s",
    "foot_sagittal_velocity_ipsi_rad_s",
    "thigh_sagittal_velocity_contra_rad_s",
    "shank_sagittal_velocity_contra_rad_s",
    "foot_sagittal_velocity_contra_rad_s",
]

GRF_SIGNALS = [
    "vertical_grf_ipsi_N",
    "vertical_grf_contra_N",
]

SIGNAL_UNITS: dict[str, str] = {}
SIGNAL_UNITS.update(dict.fromkeys(ANGLE_SIGNALS, "rad"))
SIGNAL_UNITS.update(dict.fromkeys(VELOCITY_SIGNALS, "rad/s"))
SIGNAL_UNITS.update(dict.fromkeys(SEGMENT_ANGLE_SIGNALS, "rad"))
SIGNAL_UNITS.update(dict.fromkeys(SEGMENT_VELOCITY_SIGNALS, "rad/s"))
SIGNAL_UNITS.update(dict.fromkeys(GRF_SIGNALS, "N"))

SIGNAL_ORDER: list[str] = (
    ANGLE_SIGNALS
    + VELOCITY_SIGNALS
    + SEGMENT_ANGLE_SIGNALS
    + SEGMENT_VELOCITY_SIGNALS
    + GRF_SIGNALS
)


def _lighten_color(color: str, amount: float) -> str:
    color_value = color.lstrip("#")
    if len(color_value) != 6:
        return f"#{color_value}" if not color.startswith("#") else color
    try:
        r, g, b = (int(color_value[i : i + 2], 16) for i in (0, 2, 4))
    except ValueError:
        return f"#{color_value}" if not color.startswith("#") else color
    amount_clamped = max(0.0, min(amount, 1.0))
    r = int(r + (255 - r) * amount_clamped)
    g = int(g + (255 - g) * amount_clamped)
    b = int(b + (255 - b) * amount_clamped)
    return f"#{r:02x}{g:02x}{b:02x}"


def _strip_signal_units(name: str) -> str:
    stripped = name
    for suffix in ("_rad_s", "_rad", "_Nm_kg", "_Nm", "_N"):
        if stripped.endswith(suffix):
            stripped = stripped[: -len(suffix)]
            break
    return stripped


def _signal_color_metadata(name: str) -> tuple[str, str | None, str]:
    working = _strip_signal_units(name)
    side: str | None = None
    for candidate in ("ipsi", "contra"):
        marker = f"_{candidate}"
        if marker in working:
            working = working.split(marker)[0]
            side = candidate
            break
    quantity = "other"
    if "_velocity" in working:
        quantity = "velocity"
    elif "_angle" in working:
        quantity = "angle"
    elif "_grf" in working:
        quantity = "force"
    measurement = working
    for suffix in ("_angle", "_velocity"):
        if measurement.endswith(suffix):
            measurement = measurement[: -len(suffix)]
            break
    return measurement, side, quantity


def _resolve_signal_unit(name: str) -> str:
    if name in SIGNAL_UNITS:
        return SIGNAL_UNITS[name]
    if name.startswith("torque_safe_") or name.startswith("torque_raw_"):
        if name.endswith("_Nm_kg"):
            return "Nm/kg"
        if name.endswith("_Nm"):
            return "Nm"
        return "Nm"
    return ""


def _resolve_signal_color(name: str) -> str:
    if name.startswith("torque_safe_") or name.startswith("torque_raw_"):
        return OUTPUT_COLOR
    measurement, side, quantity = _signal_color_metadata(name)
    base_color = MEASUREMENT_BASE_COLORS.get(measurement, INPUT_COLOR)
    lighten_amount = 0.0
    if quantity == "velocity":
        lighten_amount += VELOCITY_LIGHTEN
    elif quantity == "force":
        lighten_amount += FORCE_LIGHTEN
    if side is not None:
        lighten_amount += SIDE_LIGHTEN.get(side, 0.0)
    return _lighten_color(base_color, lighten_amount)


def _resolve_display_name(name: str) -> str:
    if name.startswith("torque_safe_"):
        return name.removeprefix("torque_safe_")
    if name.startswith("torque_raw_"):
        return name.removeprefix("torque_raw_")
    return name


class _RTPlotPublisher:
    """Thin wrapper around better-rtplot for streaming runtime features."""

    def __init__(self, host: str, ordered_entries: Sequence[tuple[str, str, str, str]]) -> None:
        try:
            from rtplot import client as rt_client  # type: ignore[import]
        except Exception as exc:
            raise RuntimeError(f"rtplot client unavailable: {exc}") from exc

        host_clean = (host or "").strip()
        host_key = host_clean.lower()
        if not host_clean or host_key == "local":
            rt_client.local_plot()
        elif host_key == "tv":
            rt_client.plot_to_neurobionics_tv()
        else:
            rt_client.configure_ip(host_clean)

        grouped: dict[str, list[tuple[str, str, str]]] = {}
        for column, label, unit, color in ordered_entries:
            unit_key = unit if unit else "unitless"
            grouped.setdefault(unit_key, []).append((column, label, color))

        layout = []
        ordered_columns: list[str] = []
        for unit_key, items in grouped.items():
            ylabel = "" if unit_key == "unitless" else unit_key
            title = "Unitless" if unit_key == "unitless" else unit_key
            widths = [1] * len(items)
            plot_desc = {
                "names": [label for _, label, _ in items],
                "colors": [color for _, _, color in items],
                "line_width": widths,
                "ylabel": ylabel,
                "title": title,
            }
            if unit_key == "rad" and any("angle" in column for column, _, _ in items):
                plot_desc["yrange"] = [-math.pi / 2, math.pi / 2]
            layout.append(plot_desc)
            ordered_columns.extend(column for column, _, _ in items)

        if not ordered_columns:
            raise RuntimeError("No streamable signals provided for rtplot")

        rt_client.initialize_plots(layout)
        self._client = rt_client
        self._ordered_columns = ordered_columns

    def send(self, features: Mapping[str, float]) -> None:
        import numpy as _np

        values = [_np.float32(features.get(name, 0.0)) for name in self._ordered_columns]
        payload = _np.asarray(values, dtype=_np.float32).reshape(len(values), 1)
        self._client.send_array(payload)

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
    target_frequency_hz: float | None = None
    sensor_signal_map: dict[str, tuple[str, ...]] = field(default_factory=dict)
    rtplot_host: str | None = None
    _loop_periods: list[float] = field(default_factory=list, init=False)
    _last_tick_time: float | None = field(default=None, init=False)
    _rtplot_publisher: _RTPlotPublisher | None = field(default=None, init=False)

    @classmethod
    def create(
        cls,
        *,
        root: Path | None,
        profile_path: Path,
        profile_name: str,
        enable_csv: bool = True,
        target_frequency_hz: float | None = None,
        signal_routes: Sequence["SignalRoute"] | None = None,
        rtplot_host: str | None = None,
    ) -> "DiagnosticsArtifacts":
        """Initialise a diagnostics run directory and CSV sink."""
        signal_map: dict[str, tuple[str, ...]] = {}
        if signal_routes is not None:
            grouped: dict[str, set[str]] = {}
            for route in signal_routes:
                provider = getattr(route, "provider", None)
                name = getattr(route, "name", None)
                if provider is None or name is None:
                    continue
                grouped.setdefault(provider, set()).add(name)
            signal_map = {alias: tuple(sorted(names)) for alias, names in grouped.items()}

        if isinstance(rtplot_host, str):
            stripped = rtplot_host.strip()
            host_value = stripped if stripped else None
        else:
            host_value = None

        if root is None:
            return cls(
                enabled=False,
                run_dir=Path(),
                additional_dir=Path(),
                sensors_dir=Path(),
                running_stats_dir=Path(),
                sink=None,
                target_frequency_hz=target_frequency_hz,
                sensor_signal_map=signal_map,
                rtplot_host=host_value,
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
            target_frequency_hz=target_frequency_hz,
            sensor_signal_map=signal_map,
            rtplot_host=host_value,
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

    def publish_realtime(self, features: Mapping[str, float], torque: TorqueCommand | None = None) -> None:
        """Stream live diagnostics metrics via rtplot when enabled."""
        if self.rtplot_host is None:
            return

        payload: dict[str, float] = dict(features)
        if torque is not None:
            for joint, value in torque.torques_nm.items():
                payload[f"torque_safe_{joint}"] = float(value)

        if not payload:
            return

        if self._rtplot_publisher is None:
            entries = self._build_rtplot_entries(payload)
            if not entries:
                LOGGER.warning("No eligible signals to stream via rtplot; disabling publisher")
                self.rtplot_host = None
                return
            try:
                self._rtplot_publisher = _RTPlotPublisher(self.rtplot_host, entries)
            except Exception as exc:
                LOGGER.warning("Failed to initialise rtplot streaming: %s", exc)
                self.rtplot_host = None
                return

        try:
            self._rtplot_publisher.send(payload)
        except Exception as exc:
            LOGGER.warning("rtplot streaming error (%s); disabling publisher", exc)
            self._rtplot_publisher = None
            self.rtplot_host = None

    def _build_rtplot_entries(self, values: Mapping[str, float]) -> list[tuple[str, str, str, str]]:
        seen: set[str] = set()
        entries: list[tuple[str, str, str, str]] = []

        for name in SIGNAL_ORDER:
            if name in values:
                unit = _resolve_signal_unit(name)
                label = _resolve_display_name(name)
                color = _resolve_signal_color(name)
                entries.append((name, label, unit, color))
                seen.add(name)

        torque_keys = [key for key in values if key.startswith("torque_safe_") or key.startswith("torque_raw_")]
        for name in sorted(torque_keys):
            if name in seen:
                continue
            unit = _resolve_signal_unit(name)
            label = _resolve_display_name(name)
            color = _resolve_signal_color(name)
            entries.append((name, label, unit, color))
            seen.add(name)

        for name in values:
            if (
                name in seen
                or name == "timestamp"
                or name.startswith("scheduler_")
            ):
                continue
            unit = _resolve_signal_unit(name)
            label = _resolve_display_name(name)
            color = _resolve_signal_color(name)
            entries.append((name, label, unit, color))
            seen.add(name)

        return entries

    def finalise(self, sensors: Mapping[str, BaseSensor]) -> None:
        """Persist aggregated scheduler and sensor diagnostics."""
        if not self.enabled:
            return
        loop_stats = self._write_scheduler_metrics()
        sensor_summaries = self._write_sensor_metrics(sensors, loop_stats)
        self._write_report(loop_stats, sensor_summaries)
        self._render_data_plot()

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

    def _write_sensor_metrics(
        self,
        sensors: Mapping[str, BaseSensor],
        loop_stats: Mapping[str, float | int | None],
    ) -> list[dict[str, object]]:
        summaries: list[dict[str, object]] = []
        expected_ticks_raw = loop_stats.get("ticks")
        expected_ticks = expected_ticks_raw if isinstance(expected_ticks_raw, int) and expected_ticks_raw > 0 else None
        duration_raw = loop_stats.get("duration_s")
        duration_s = duration_raw if isinstance(duration_raw, (int, float)) and duration_raw > 0 else None
        reference_rate = loop_stats.get("mean_rate_hz")
        if not isinstance(reference_rate, (int, float)) or not math.isfinite(reference_rate):
            reference_rate = None
        if reference_rate is None:
            ref_target = self.target_frequency_hz
            if isinstance(ref_target, (int, float)) and math.isfinite(ref_target) and ref_target > 0:
                reference_rate = ref_target
        if reference_rate is not None and reference_rate <= 0:
            reference_rate = None
        expected_period_s = (1.0 / reference_rate) if reference_rate else None

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

            sensor_config = getattr(sensor, "config", None)
            drop_intervals = self._compute_drop_intervals(periods, sensor_config)
            events = list(
                zip(
                    getattr(diagnostics, "recent_event_timestamps", ()) or (),
                    getattr(diagnostics, "recent_event_fresh", ()) or (), strict=False,
                )
            )
            availability_plot_path = None
            try:
                availability_plot_path = self._render_availability_plot(
                    alias,
                    events,
                    target,
                    run_duration=duration_s,
                    expected_period_s=expected_period_s,
                )
            except Exception as exc:  # pragma: no cover - best effort
                LOGGER.warning("Failed to render availability plot for '%s': %s", alias, exc)
            total_samples = getattr(diagnostics, "total_samples", 0)
            coverage_pct: float | None = None
            coverage_ratio: float | None = None
            missed_samples: int | None = None
            if expected_ticks is not None and expected_ticks > 0:
                coverage_ratio = min(max(total_samples / expected_ticks, 0.0), 1.0)
                coverage_pct = coverage_ratio * 100.0
                missed_samples = max(expected_ticks - total_samples, 0)
            effective_rate: float | None = None
            if coverage_ratio is not None and reference_rate is not None:
                effective_rate = coverage_ratio * reference_rate
            elif duration_s is not None and duration_s > 0:
                effective_rate = total_samples / duration_s if total_samples > 0 else 0.0
            if effective_rate is not None and reference_rate is not None:
                effective_rate = min(effective_rate, reference_rate)

            hz_estimate = getattr(diagnostics, "hz_estimate", None)
            hz_min = getattr(diagnostics, "hz_min", None)
            hz_max = getattr(diagnostics, "hz_max", None)
            hz_mean = getattr(diagnostics, "hz_mean", None)
            fresh_ratio_window = getattr(diagnostics, "fresh_ratio_window", None)
            if isinstance(fresh_ratio_window, (int, float)) and math.isfinite(fresh_ratio_window):
                fresh_ratio_pct = fresh_ratio_window * 100.0
            else:
                fresh_ratio_pct = None
            max_consecutive_stale = getattr(diagnostics, "max_consecutive_stale", None)
            current_stale = getattr(diagnostics, "stale_samples", 0)

            summary_lines = [
                f"total_samples={total_samples}",
                f"missed_samples={missed_samples}",
                f"stale_samples_current={current_stale}",
                f"max_consecutive_stale={max_consecutive_stale}",
                f"hz_estimate={hz_estimate}",
                f"hz_min={hz_min}",
                f"hz_max={hz_max}",
                f"hz_mean={hz_mean}",
                f"coverage_pct={coverage_pct}",
                f"effective_rate_hz={effective_rate}",
                f"fresh_ratio_window={fresh_ratio_window}",
            ]
            try:
                (target / "summary.txt").write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
            except Exception as exc:  # pragma: no cover - best effort
                LOGGER.warning("Failed to write sensor summary for '%s': %s", alias, exc)
            sensor_summary = {
                "sensor_alias": alias,
                "total_samples": total_samples,
                "missed_samples": missed_samples,
                "stale_samples": current_stale,
                "max_consecutive_stale": max_consecutive_stale,
                "hz_estimate": hz_estimate,
                "hz_min": hz_min,
                "hz_max": hz_max,
                "hz_mean": hz_mean,
                "coverage_pct": coverage_pct,
                "coverage_ratio": coverage_ratio,
                "effective_rate_hz": effective_rate,
                "fresh_ratio_window": fresh_ratio_window,
                "fresh_ratio_pct": fresh_ratio_pct,
                "events_window": len(events),
                "max_period_s": max(periods) if periods else None,
                "period_hist": (period_edges, period_counts),
                "rate_hist": (rate_edges, rate_counts),
                "periods_csv": target / "sample_periods.csv",
                "rates_csv": target / "sample_rates_hz.csv",
                "period_hist_csv": target / "sample_period_histogram.csv",
                "rate_hist_csv": target / "sample_rate_histogram.csv",
                "summary_path": target / "summary.txt",
                "config": sensor_config,
                "availability_plot": availability_plot_path,
                "drop_intervals": drop_intervals,
            }
            signals = self.sensor_signal_map.get(alias, ())
            if signals:
                for signal_name in signals:
                    entry = dict(sensor_summary)
                    entry["signal_name"] = signal_name
                    summaries.append(entry)
            else:
                entry = dict(sensor_summary)
                entry["signal_name"] = alias
                summaries.append(entry)
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
        if isinstance(ticks, int):
            lines.append(f"| Ticks | {ticks} |")
        else:
            lines.append("| Ticks | N/A |")
        target_rate = self.target_frequency_hz
        if isinstance(target_rate, (int, float)) and math.isfinite(target_rate):
            lines.append(f"| Target Loop Rate (Hz) | {target_rate:.2f} |")
        mean_rate_overall = loop_stats.get("mean_rate_hz")
        if isinstance(mean_rate_overall, (int, float)) and math.isfinite(mean_rate_overall):
            lines.append(f"| Mean Observed Rate (Hz) | {mean_rate_overall:.2f} |")
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
            "- **Sensors** tables show fresh coverage, stale streaks, and data availability. "
            "CSV links capture detailed period and rate histograms."
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

        def _signal_key(summary: Mapping[str, object]) -> str:
            name = summary.get("signal_name")
            return str(name)

        for summary in sorted(sensor_summaries, key=_signal_key):
            signal_name_obj = summary.get("signal_name")
            signal_name = str(signal_name_obj) if signal_name_obj is not None else "unknown"
            sensor_alias_obj = summary.get("sensor_alias")
            sensor_alias = str(sensor_alias_obj) if sensor_alias_obj else None
            sensor_label = sensor_alias if sensor_alias else "unassigned sensor"
            lines.append(f"### {signal_name}")
            lines.append("")
            lines.append("| Metric | Value |")
            lines.append("|--------|-------|")
            if sensor_alias:
                lines.append(f"| Sensor | `{sensor_alias}` |")
            coverage_pct_obj = summary.get("coverage_pct")
            total_samples = summary.get("total_samples")
            if isinstance(coverage_pct_obj, (int, float)):
                coverage_pct = float(coverage_pct_obj)
                coverage_display = f"{coverage_pct:.1f}%"
                if isinstance(total_samples, int):
                    coverage_display += f" ({total_samples})"
            else:
                coverage_display = str(total_samples)
            lines.append(f"| Fresh Sample Coverage | {coverage_display} |")
            missed_samples = summary.get("missed_samples")
            if isinstance(missed_samples, int):
                lines.append(f"| Missed Samples | {missed_samples} |")
            max_stale = summary.get("max_consecutive_stale")
            if isinstance(max_stale, int):
                lines.append(f"| Max Stale Run (ticks) | {max_stale} |")
            max_period_obj = summary.get("max_period_s")
            max_period = float(max_period_obj) if isinstance(max_period_obj, (int, float)) else None
            lines.append(f"| Longest Gap (s) | {_fmt(max_period)} |")
            effective_rate_obj = summary.get("effective_rate_hz")
            effective_rate = float(effective_rate_obj) if isinstance(effective_rate_obj, (int, float)) else None
            lines.append(f"| Delivered Fresh Rate (Hz) | {_fmt(effective_rate)} |")
            lines.append("")

            availability_plot_obj = summary.get("availability_plot")
            availability_plot = (
                availability_plot_obj if isinstance(availability_plot_obj, Path) else None
            )
            if availability_plot is not None and availability_plot.exists():
                rel_plot = availability_plot.relative_to(self.run_dir)
                lines.append(f"![Data Availability Timeline]({rel_plot.as_posix()})")
                lines.append("")

            drop_intervals_obj = summary.get("drop_intervals")
            drop_intervals: list[Sequence[object]] = []
            if isinstance(drop_intervals_obj, Iterable) and not isinstance(
                drop_intervals_obj, (str, bytes)
            ):
                for item in drop_intervals_obj:
                    if isinstance(item, Sequence):
                        drop_intervals.append(item)
            if drop_intervals:
                threshold = getattr(summary.get("config"), "max_stale_time_s", None)
                if threshold is not None:
                    lines.append(f"Dropout intervals (period > {threshold:.2f}s):")
                else:
                    lines.append("Dropout intervals:")
                for interval in drop_intervals:
                    if not isinstance(interval, Sequence) or len(interval) != 3:
                        continue
                    start_obj, end_obj, duration_obj = interval
                    if not isinstance(start_obj, (int, float)):
                        continue
                    if not isinstance(end_obj, (int, float)):
                        continue
                    if not isinstance(duration_obj, (int, float)):
                        continue
                    start = float(start_obj)
                    end = float(end_obj)
                    duration = float(duration_obj)
                    lines.append(f"- {start:.2f}s → {end:.2f}s (duration {duration:.2f}s)")
            lines.append("")

            config = summary.get("config")
            if config is not None:
                max_stale_time = getattr(config, "max_stale_time_s", None)
                if max_stale_time is not None and isinstance(max_period, (int, float)) and max_period > max_stale_time:
                    alerts.append(
                        f"⚠️ `{signal_name}` ({sensor_label}) observed gap {max_period:.2f}s exceeding configured "
                        f"max_stale_time_s={max_stale_time:.2f}s."
                    )
            stale_current = summary.get("stale_samples")
            if isinstance(stale_current, int) and stale_current > 0:
                alerts.append(f"⚠️ `{signal_name}` ({sensor_label}) reported {stale_current} stale samples.")

        lines.append("## Alerts")
        lines.append("")
        if alerts:
            for alert in alerts:
                lines.append(f"- {alert}")
        else:
            lines.append("- ✅ No notable diagnostics alerts recorded.")
        lines.append("")

        if GLOSSARY_ENTRIES:
            lines.append("## Glossary")
            lines.append("")
            lines.append("| Metric | Meaning |")
            lines.append("|--------|---------|")
            for term, description in GLOSSARY_ENTRIES:
                lines.append(f"| {term} | {description} |")
            lines.append("")

        report_path.write_text("\n".join(lines), encoding="utf-8")

    def _render_data_plot(self) -> None:
        """Render a multi-panel plot of recorded inputs and outputs."""
        if not self.enabled:
            return
        data_path = self.run_dir / "data.csv"
        if not data_path.exists():
            return
        try:
            import pandas as pd
        except Exception as exc:  # pragma: no cover - optional dependency
            LOGGER.warning("Pandas unavailable (%s); skipping data plot", exc)
            return
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:  # pragma: no cover - optional dependency
            LOGGER.warning("Matplotlib unavailable (%s); skipping data plot", exc)
            return

        try:
            df = pd.read_csv(data_path)
        except Exception as exc:  # pragma: no cover - best effort
            LOGGER.warning("Failed to load diagnostics CSV for plotting: %s", exc)
            return
        if df.empty:
            return

        if "timestamp" in df.columns:
            time_series = pd.to_numeric(df["timestamp"], errors="coerce")
            time_label = "Time (s)"
        else:
            time_series = pd.Series(range(len(df)), dtype=float)
            time_label = "Sample"
        time_series = time_series.ffill().bfill()
        time_values = time_series.to_numpy(dtype=float, copy=True)

        entries: list[tuple[str, str, str, str, pd.Series]] = []

        def _append_entry(column: str) -> None:
            if column not in df.columns:
                return
            series = pd.to_numeric(df[column], errors="coerce")
            if series.dropna().empty:
                return
            label = _resolve_display_name(column)
            unit = _resolve_signal_unit(column)
            color = _resolve_signal_color(column)
            entries.append((column, label, unit, color, series))

        for column in SIGNAL_ORDER:
            _append_entry(column)

        for column in sorted(c for c in df.columns if c.startswith("torque_safe_")):
            _append_entry(column)

        feature_columns = {col for col, _, _, _, _ in entries if not col.startswith("torque_safe_")}
        for column in sorted(c for c in df.columns if c not in feature_columns and not c.startswith("torque_") and not c.startswith("scheduler_") and c not in {"timestamp"}):
            _append_entry(column)

        if not entries:
            return

        unit_ranges: dict[str, tuple[float, float]] = {}
        for _, _, unit, _, series in entries:
            numeric = series.to_numpy(dtype=float, copy=True)
            finite_mask = np.isfinite(numeric)
            if not finite_mask.any():
                continue
            finite_values = numeric[finite_mask]
            try:
                vmin = float(np.nanmin(finite_values))
                vmax = float(np.nanmax(finite_values))
            except (TypeError, ValueError):
                continue
            if not math.isfinite(vmin) or not math.isfinite(vmax):
                continue
            current = unit_ranges.get(unit)
            if current:
                vmin = min(vmin, current[0])
                vmax = max(vmax, current[1])
            unit_ranges[unit] = (vmin, vmax)

        unit_limits: dict[str, tuple[float, float]] = {}
        for unit, (vmin, vmax) in unit_ranges.items():
            if math.isclose(vmin, vmax):
                span = abs(vmin) if abs(vmin) > 1e-6 else 1.0
                pad = span * 0.1 if span else 0.1
                vmin -= pad
                vmax += pad
            unit_limits[unit] = (vmin, vmax)

        fig_height = max(4.0, 1.05 * len(entries) + 3.0)
        fig, axes = plt.subplots(len(entries), 1, sharex=True, figsize=(14, fig_height))
        title = "Runtime Signals"
        if self.profile_name:
            title = f"{self.profile_name} Signals"
        fig.suptitle(title, fontsize=14, y=0.995)
        if len(entries) == 1:
            axes = [axes]  # type: ignore[list-item]
        for ax, (_, name, unit, color, series) in zip(axes, entries, strict=False):
            values = series.to_numpy(dtype=float, copy=True)
            finite_mask = np.isfinite(values)
            aligned_time = time_values
            if len(values) != len(time_values):
                limit = min(len(values), len(time_values))
                values = values[:limit]
                finite_mask = finite_mask[:limit]
                aligned_time = time_values[:limit]
            plotted_values = np.where(finite_mask, values, np.nan)
            ax.plot(aligned_time, plotted_values, color=color, linewidth=1.2)
            if not np.all(finite_mask):
                nan_times = aligned_time[~finite_mask]
                unique_nan_times = np.unique(nan_times)
                for x in unique_nan_times:
                    ax.axvline(x, color="#c53030", linewidth=0.9, alpha=0.85, linestyle="--")
            ax.set_title(name, loc="left", fontsize=10, pad=8)
            axis_label = unit if unit else ""
            ax.set_ylabel(axis_label, fontsize=9)
            limits = unit_limits.get(unit)
            if limits:
                ax.set_ylim(*limits)
            ax.margins(x=0.0)
            ax.grid(True, linestyle="--", alpha=0.25)
            ax.tick_params(axis="y", labelsize=8, pad=3)
            ax.spines["top"].set_visible(False)
            ax.spines["right"].set_visible(False)
            ax.spines["left"].set_linewidth(0.8)
            ax.spines["left"].set_color("#2d3748")
        axes[-1].set_xlabel(time_label)
        axes[-1].tick_params(axis="x", labelsize=9, pad=2)
        fig.align_ylabels(axes)
        fig.subplots_adjust(left=0.06, right=0.975, top=0.982, bottom=0.07, hspace=0.42)
        output_path = data_path.with_suffix(".png")
        try:
            fig.savefig(output_path, dpi=150, format="png")
        except Exception as exc:  # pragma: no cover - runtime guard
            LOGGER.warning("Failed to render diagnostics data plot: %s", exc)
        finally:
            plt.close(fig)

    def _render_availability_plot(
        self,
        alias: str,
        events: Sequence[tuple[float, bool]],
        target_dir: Path,
        *,
        run_duration: float | None,
        expected_period_s: float | None,
    ) -> Path | None:
        if not events:
            return None
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception:
            return None

        timestamps = [float(ts) for ts, _ in events]
        freshness = [bool(flag) for _, flag in events]
        if not timestamps:
            return None
        start_time = timestamps[0]
        rel_times = [max(ts - start_time, 0.0) for ts in timestamps]

        path = target_dir / "data_availability.png"
        fig, ax = plt.subplots(figsize=(6, 2.0))
        span_end = run_duration if isinstance(run_duration, (int, float)) and run_duration > 0 else None
        if span_end is None:
            tail_extension = expected_period_s if isinstance(expected_period_s, (int, float)) and expected_period_s > 0 else 0.1
            span_end = rel_times[-1] + tail_extension
        span_end = max(span_end, rel_times[-1] if rel_times else 0.1, 0.1)
        for time_value, is_fresh in zip(rel_times, freshness, strict=False):
            if not is_fresh:
                ax.axvline(time_value, color="#e53e3e", linewidth=1.4, alpha=0.9)

        ax.set_title(f"{alias} Data Availability")
        ax.set_xlabel("Time since first event (s)")
        ax.set_xlim(0.0, span_end)
        ax.set_ylim(0.0, 1.0)
        ax.set_yticks([])
        ax.set_ylabel("")
        for spine in ("left", "right", "top"):
            ax.spines[spine].set_visible(False)
        ax.spines["bottom"].set_color("#4a5568")
        ax.spines["bottom"].set_linewidth(0.8)
        ax.grid(False)
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
