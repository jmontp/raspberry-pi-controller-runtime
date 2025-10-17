"""Convenience CLI for running the runtime loop with optional diagnostics."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Sequence

# Ensure local sources are importable without installation
REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rpc_runtime.config import build_runtime_components, load_runtime_profile  # noqa: E402
from rpc_runtime.runtime import DiagnosticsArtifacts, RuntimeLoop, RuntimeLoopConfig  # noqa: E402

LOGGER = logging.getLogger("rpc_runtime.scripts.run")

DEFAULT_PROFILE_PATH = Path(__file__).with_name("mock_hardware_config.yaml")
DEFAULT_DIAGNOSTICS_ROOT = DEFAULT_PROFILE_PATH.with_name("diagnostics")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse CLI arguments for the runtime loop helper."""
    parser = argparse.ArgumentParser(description="Run the controller runtime loop")
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
        type=str,
        default=str(DEFAULT_DIAGNOSTICS_ROOT),
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
    parser.add_argument(
        "--rtplot",
        action="store_true",
        help="Stream live feature traces to better-rtplot; requires the plotting server to be running.",
    )
    parser.add_argument(
        "--rtplot-host",
        type=str,
        default="local",
        help="Target plot host for rtplot (e.g. 'local', 'tv', '192.168.1.42:5555').",
    )
    return parser.parse_args(argv)


def _configure_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger("matplotlib").setLevel(logging.WARNING)


def _normalise_diagnostics_root(raw: str | None) -> Path | None:
    if raw is None:
        return None
    value = raw.strip()
    if not value or value.lower() == "none":
        return None
    return Path(value).expanduser().resolve()


def run_loop(
    *,
    profile_path: Path,
    frequency_hz: float,
    duration_s: float | None,
    diagnostics_root: Path | None,
    profile_name_override: str | None,
    rtplot: bool,
    rtplot_host: str,
) -> Path | None:
    """Build runtime components and execute the control loop."""
    profile = load_runtime_profile(profile_path)
    components = build_runtime_components(profile)
    effective_profile_name = profile_name_override or profile.name

    diagnostics_root = diagnostics_root
    if diagnostics_root is not None:
        diagnostics_root.mkdir(parents=True, exist_ok=True)

    rtplot_host_value = rtplot_host if rtplot else None

    artifacts = DiagnosticsArtifacts.create(
        root=diagnostics_root,
        profile_path=profile_path,
        profile_name=effective_profile_name,
        enable_csv=diagnostics_root is not None,
        target_frequency_hz=frequency_hz,
        signal_routes=profile.signal_routes,
        rtplot_host=rtplot_host_value,
    )

    loop = RuntimeLoop(
        RuntimeLoopConfig(frequency_hz=frequency_hz, profile_name=effective_profile_name),
        sensors=components.sensors,
        actuator=components.actuator,
        controller=components.controller,
        signal_routes=profile.signal_routes,
        diagnostics=artifacts.sink,
        artifacts=artifacts if artifacts.diagnostics_enabled() else None,
    )

    LOGGER.info("Starting runtime loop at %.1f Hz (profile=%s)", frequency_hz, profile.name)
    if artifacts.diagnostics_enabled():
        LOGGER.info("Diagnostics directory: %s", artifacts.run_dir)
    if duration_s is not None:
        LOGGER.info("Configured duration: %.2f seconds", duration_s)

    try:
        with loop:
            for _ in loop.run(duration_s=duration_s):
                pass
    except KeyboardInterrupt:
        LOGGER.info("Received keyboard interrupt, stopping runtime loop")
    finally:
        LOGGER.info("Runtime loop finished")

    return artifacts.run_dir if artifacts.diagnostics_enabled() else None


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point for the runtime loop CLI."""
    args = parse_args(argv)
    _configure_logging(args.verbose)

    profile_path = args.config.expanduser().resolve()
    diagnostics_root = _normalise_diagnostics_root(args.diagnostics)

    try:
        run_dir = run_loop(
            profile_path=profile_path,
            frequency_hz=args.frequency,
            duration_s=args.duration,
            diagnostics_root=diagnostics_root,
            profile_name_override=args.profile_name,
            rtplot=args.rtplot,
            rtplot_host=args.rtplot_host,
        )
    except FileNotFoundError as exc:
        LOGGER.error("Profile file not found: %s", exc)
        return 1
    except Exception:
        LOGGER.exception("Runtime loop terminated with an error")
        return 2

    if run_dir is not None:
        LOGGER.info("Artifacts captured in %s", run_dir)
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    sys.exit(main())
