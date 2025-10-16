"""Convenience CLI for running the runtime loop with a mock profile."""

from __future__ import annotations

import argparse
import logging
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Mapping, Sequence

# Ensure the repository `src` directory is on sys.path so local changes are importable without installing.
REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

VENV_SITE = REPO_ROOT / ".venv" / f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
if VENV_SITE.exists() and str(VENV_SITE) not in sys.path:
    sys.path.insert(0, str(VENV_SITE))

from rpc_runtime.config import build_runtime_components, load_runtime_profile
from rpc_runtime.runtime.diagnostics import CSVDiagnosticsSink, DiagnosticsSink
from rpc_runtime.runtime.loop import RuntimeLoop, RuntimeLoopConfig

LOGGER = logging.getLogger("rpc_runtime.scripts.run")

DEFAULT_PROFILE_PATH = Path(__file__).with_name("mock_hardware_config.yaml")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="Run the controller runtime loop with the supplied profile.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_PROFILE_PATH,
        help="Path to hardware profile YAML",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=100.0,
        help="Target loop frequency in Hz",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional run duration in seconds",
    )
    parser.add_argument(
        "--diagnostics",
        dest="diagnostics_root",
        type=str,
        default=str(DEFAULT_PROFILE_PATH.with_name("diagnostics")),
        help="Directory for diagnostics artifacts; pass 'none' to disable",
    )
    parser.add_argument(
        "--profile-name",
        type=str,
        default=None,
        help="Override profile name recorded in the runtime config",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable debug logging",
    )
    return parser.parse_args(argv)


def _configure_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger("matplotlib").setLevel(logging.WARNING)


def _prepare_run_artifacts(
    root: Path | None,
    *,
    profile_path: Path,
    profile_name: str,
) -> tuple[DiagnosticsSink | None, Path | None, Path | None]:
    """Create a timestamped diagnostics directory and return a CSV sink."""
    if root is None:
        return None, None, None

    root.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
    safe_profile = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in profile_name)
    run_dir = root / f"{safe_profile}_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    sensors_dir = run_dir / "sensors"
    sensors_dir.mkdir(parents=True, exist_ok=True)

    config_dest = run_dir / profile_path.name
    try:
        shutil.copy2(profile_path, config_dest)
    except Exception as exc:
        LOGGER.warning("Failed to copy hardware config to diagnostics dir: %s", exc)

    sink_path = run_dir / "loop.csv"
    sink = CSVDiagnosticsSink(str(sink_path))
    sensors_pointer = run_dir / "sensors_directory.txt"
    try:
        sensors_pointer.write_text(f"{sensors_dir.name}\n", encoding="utf-8")
    except Exception as exc:
        LOGGER.warning("Failed to record sensors directory pointer: %s", exc)
    return sink, run_dir, sensors_dir


def _compute_histogram(samples: Sequence[float], *, bins: int = 10) -> tuple[tuple[float, ...], tuple[int, ...]]:
    """Compute histogram edges and counts for the provided samples."""
    if not samples:
        return (), ()
    min_val = min(samples)
    max_val = max(samples)
    if min_val == max_val:
        return (min_val, max_val), (len(samples),)
    bin_count = max(1, min(bins, len(samples)))
    span = max_val - min_val
    width = span / bin_count if span > 0 else 1.0
    edges = [min_val + i * width for i in range(bin_count)]
    edges.append(max_val)
    counts = [0 for _ in range(bin_count)]
    for value in samples:
        if value >= max_val:
            index = bin_count - 1
        else:
            index = int((value - min_val) / width)
        counts[index] += 1
    return tuple(edges), tuple(counts)


def _write_histogram_csv(path: Path, samples: Sequence[float], *, bins: int = 10) -> None:
    """Persist a simple histogram CSV from a sequence of samples."""
    path.parent.mkdir(parents=True, exist_ok=True)
    edges, counts = _compute_histogram(samples, bins=bins)
    with path.open("w", encoding="utf-8") as handle:
        handle.write("bin_start,bin_end,count\n")
        if not edges:
            return
        if len(counts) == 1:
            handle.write(f"{edges[0]},{edges[-1]},{counts[0]}\n")
            return
        for idx, count in enumerate(counts):
            handle.write(f"{edges[idx]},{edges[idx + 1]},{count}\n")


def _write_series_csv(path: Path, samples: Sequence[float], header: str) -> None:
    """Write a simple one-column CSV of sample values."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        handle.write(f"{header}\n")
        for value in samples:
            handle.write(f"{value}\n")


def _render_histogram_image(values: Sequence[float], path: Path, *, bins: int = 10) -> None:
    """Render a histogram PNG using matplotlib when possible, otherwise fall back."""
    if not values:
        LOGGER.info("No loop rate samples captured; histogram image skipped")
        return

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - optional dependency
        LOGGER.warning("Matplotlib unavailable (%s); falling back to Pillow renderer", exc)
        _render_histogram_image_fallback(values, path, bins=bins)
        return

    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.hist(list(values), bins=bins, edgecolor="#1a4a7a", color="#3182ce")
    ax.set_title("Runtime Loop Rate Histogram")
    ax.set_xlabel("Loop rate (Hz)")
    ax.set_ylabel("Count")
    ax.grid(True, axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    try:
        fig.savefig(path, dpi=150, format="png")
    except Exception as exc:  # pragma: no cover - runtime issues (e.g., OMP errors)
        LOGGER.warning("Matplotlib render failed (%s); falling back to Pillow", exc)
        _render_histogram_image_fallback(values, path, bins=bins)
    finally:
        plt.close(fig)


def _render_histogram_image_fallback(values: Sequence[float], path: Path, *, bins: int = 10) -> None:
    """Render a histogram PNG using Pillow with basic axis annotations."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except Exception as exc:  # pragma: no cover - optional dependency
        LOGGER.warning("Pillow unavailable (%s); skipping histogram image at %s", exc, path)
        return

    edges, counts = _compute_histogram(values, bins=bins)
    path.parent.mkdir(parents=True, exist_ok=True)

    width = 640
    height = 360
    margin = 60
    plot_width = width - 2 * margin
    plot_height = height - 2 * margin

    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)
    font = ImageFont.load_default()

    def measure(text: str) -> tuple[int, int]:
        bbox = draw.textbbox((0, 0), text, font=font)
        return int(bbox[2] - bbox[0]), int(bbox[3] - bbox[1])

    # Axes
    draw.line((margin, height - margin, width - margin, height - margin), fill="#333333", width=2)
    draw.line((margin, margin, margin, height - margin), fill="#333333", width=2)

    if edges:
        max_count = max(counts) if counts else 1
        min_edge, max_edge = edges[0], edges[-1]
        span = max(max_edge - min_edge, 1e-9)
        for idx, count in enumerate(counts):
            bin_start = edges[idx]
            bin_end = edges[idx + 1] if idx + 1 < len(edges) else edges[idx]
            bar_height = (count / max_count) * plot_height if max_count else 0
            x = margin + ((bin_start - min_edge) / span) * plot_width
            width_frac = max((bin_end - bin_start) / span, 1 / max(len(counts), 50))
            bar_width = width_frac * plot_width
            y = height - margin - bar_height
            draw.rectangle(
                (int(x), int(y), int(x + bar_width), int(height - margin)),
                fill="#3182ce",
                outline="#1a4a7a",
            )

        # X-axis ticks (min, mid, max)
        tick_values = [min_edge, min_edge + span / 2, max_edge]
        for value in tick_values:
            label = f"{value:.2f}"
            label_width, label_height = measure(label)
            pos = margin + ((value - min_edge) / span) * plot_width
            draw.line((pos, height - margin, pos, height - margin + 6), fill="#333333", width=2)
            draw.text((pos - label_width / 2, height - margin + 8), label, fill="#111111", font=font)

        # Y-axis ticks (0 and max_count)
        for value in (0, max_count):
            label = str(value)
            label_width, label_height = measure(label)
            pos = height - margin - (value / max_count if max_count else 0) * plot_height
            draw.line((margin - 6, pos, margin, pos), fill="#333333", width=2)
            draw.text((margin - label_width - 8, pos - label_height / 2), label, fill="#111111", font=font)

    title = "Runtime Loop Rate Histogram"
    title_width, title_height = measure(title)
    draw.text(((width - title_width) / 2, margin / 2 - title_height / 2), title, fill="#111111", font=font)

    x_label = "Loop rate (Hz)"
    x_width, x_height = measure(x_label)
    draw.text(((width - x_width) / 2, height - margin + 24), x_label, fill="#111111", font=font)

    y_label = "Count"
    y_width, y_height = measure(y_label)
    draw.text((margin / 2 - y_width / 2, margin - y_height - 16), y_label, fill="#111111", font=font)

    img.save(path, format="PNG")


def _record_scheduler_metrics(run_dir: Path, periods: Sequence[float]) -> None:
    """Write scheduler timing artifacts such as loop rate histogram and image."""
    stats_dir = run_dir / "running_stats"
    stats_dir.mkdir(parents=True, exist_ok=True)
    rates = [1.0 / p for p in periods if p > 0]

    histogram_csv = stats_dir / "loop_rate_histogram.csv"
    _write_histogram_csv(histogram_csv, rates, bins=10)

    histogram_png = stats_dir / "loop_rate_histogram.png"
    _render_histogram_image(rates, histogram_png, bins=10)


def _record_sensor_metrics(
    sensors: Mapping[str, object],
    sensors_dir: Path,
) -> None:
    """Persist timing diagnostics for each sensor when available."""
    for alias, sensor in sensors.items():
        diagnostics = getattr(sensor, "diagnostics", None)
        if diagnostics is None:
            continue
        periods = tuple(getattr(diagnostics, "recent_sample_periods", ()) or ())
        target_dir = sensors_dir / alias
        target_dir.mkdir(parents=True, exist_ok=True)
        _write_series_csv(target_dir / "sample_periods.csv", periods, "seconds")
        _write_histogram_csv(target_dir / "sample_period_histogram.csv", periods)
        summary_path = target_dir / "summary.txt"
        summary = [
            f"total_samples={getattr(diagnostics, 'total_samples', 0)}",
            f"stale_samples={getattr(diagnostics, 'stale_samples', 0)}",
            f"hz_estimate={getattr(diagnostics, 'hz_estimate', None)}",
            f"hz_min={getattr(diagnostics, 'hz_min', None)}",
            f"hz_max={getattr(diagnostics, 'hz_max', None)}",
            f"hz_mean={getattr(diagnostics, 'hz_mean', None)}",
        ]
        summary_path.write_text("\n".join(summary) + "\n", encoding="utf-8")


def run_loop(
    *,
    profile_path: Path,
    frequency_hz: float,
    duration_s: float | None,
    diagnostics_root: Path | None,
    profile_name_override: str | None = None,
) -> Path | None:
    """Instantiate components from the profile and drive the runtime loop."""
    profile = load_runtime_profile(profile_path)
    effective_profile_name = profile_name_override or profile.name

    diagnostics_sink, run_dir, sensors_dir = _prepare_run_artifacts(
        diagnostics_root,
        profile_path=profile_path,
        profile_name=effective_profile_name,
    )
    components = build_runtime_components(profile)
    loop = RuntimeLoop(
        RuntimeLoopConfig(
            frequency_hz=frequency_hz,
            profile_name=effective_profile_name,
        ),
        sensors=components.sensors,
        actuator=components.actuator,
        controller=components.controller,
        signal_routes=profile.signal_routes,
        diagnostics=diagnostics_sink,
    )
    LOGGER.info("Starting runtime loop at %.1f Hz (profile=%s)", frequency_hz, profile.name)
    if run_dir is not None:
        LOGGER.info("Diagnostics directory: %s", run_dir)
    if duration_s is not None:
        LOGGER.info("Configured duration: %.2f seconds", duration_s)

    loop_periods: list[float] = []
    last_tick_time: float | None = None

    try:
        with loop:
            for _ in loop.run(duration_s=duration_s):
                now = time.perf_counter()
                if last_tick_time is not None:
                    loop_periods.append(now - last_tick_time)
                last_tick_time = now
    except KeyboardInterrupt:
        LOGGER.info("Received keyboard interrupt, stopping runtime loop")
    finally:
        LOGGER.info("Runtime loop finished")
        if run_dir is not None:
            _record_scheduler_metrics(run_dir, loop_periods)
            if sensors_dir is not None:
                _record_sensor_metrics(components.sensors, sensors_dir)
    return run_dir


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    _configure_logging(args.verbose)
    profile_path = args.config.expanduser().resolve()
    diagnostics_root: Path | None
    if args.diagnostics_root is None:
        diagnostics_root = None
    else:
        diag_arg = str(args.diagnostics_root).strip()
        if diag_arg.lower() == "none":
            diagnostics_root = None
        else:
            diagnostics_root = Path(diag_arg).expanduser().resolve()
    try:
        run_dir = run_loop(
            profile_path=profile_path,
            frequency_hz=args.frequency,
            duration_s=args.duration,
            diagnostics_root=diagnostics_root,
            profile_name_override=args.profile_name,
        )
        if run_dir is not None:
            LOGGER.info("Artifacts captured in %s", run_dir)
    except FileNotFoundError as exc:
        LOGGER.error("Profile file not found: %s", exc)
        return 1
    except Exception:
        LOGGER.exception("Runtime loop terminated with an error")
        return 2
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    sys.exit(main())
