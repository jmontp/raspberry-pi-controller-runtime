"""Runtime pipeline orchestrating sensors, controller, and actuators."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Iterable, Mapping, Sequence

from ..actuators.base import BaseActuator
from ..controllers.pi_controller import PIController
from ..sensors.base import BaseSensor
from ..sensors.combinators import ControlInputs
from .diagnostics import DiagnosticsSink, NoOpDiagnosticsSink
from .diagnostics_artifacts import DiagnosticsArtifacts
from .safety import SafetyConfig, SafetyManager
from .scheduler import BaseScheduler, SimpleScheduler
from .wrangler import DataWrangler, InputSchema

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..config.models import SignalRoute


@dataclass(slots=True)
class RuntimeLoopConfig:
    """Configuration parameters for `RuntimeLoop`."""

    frequency_hz: float
    profile_name: str = "default"


class RuntimeLoop:
    """Orchestrate sensor polling, controller execution, and actuation."""

    LOGGER = logging.getLogger(__name__)

    def __init__(
        self,
        config: RuntimeLoopConfig,
        *,
        sensors: Mapping[str, BaseSensor],
        actuator: BaseActuator,
        controller: PIController,
        signal_routes: Sequence["SignalRoute"],
        input_schema: InputSchema | None = None,
        scheduler_factory: Callable[[float | None], BaseScheduler] | None = None,
        diagnostics: DiagnosticsSink | None = None,
        artifacts: DiagnosticsArtifacts | None = None,
    ) -> None:
        """Initialise the runtime loop with hardware interfaces and scheduling."""
        self._config = config
        self._actuator = actuator
        self._controller = controller
        self._scheduler_factory = scheduler_factory or (
            lambda duration: SimpleScheduler(config.frequency_hz, duration)
        )
        self._diagnostics: DiagnosticsSink = diagnostics or NoOpDiagnosticsSink()
        self._artifacts = artifacts
        self._sensors: dict[str, BaseSensor] = dict(sensors)

        schema = input_schema or getattr(controller, "_input_schema", None)
        if schema is None:
            raise ValueError("Controller must declare an input schema for DataWrangler")

        self._wrangler = DataWrangler(
            required_inputs=schema,
            signal_routes=signal_routes,
            sensors=self._sensors,
            diagnostics_sink=self._diagnostics,
        )
        limits = getattr(getattr(self._actuator, "config", None), "torque_limits_nm", None)
        self._safety = SafetyManager(SafetyConfig(torque_limits_nm=limits or None))

    def __enter__(self) -> "RuntimeLoop":
        """Enter the loop context by initialising sensors and actuators."""
        self._wrangler.__enter__()
        self._actuator.__enter__()
        self._controller.reset()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Tear down resources when leaving the context."""
        self._actuator.__exit__(exc_type, exc, tb)
        self._wrangler.__exit__(exc_type, exc, tb)
        self._diagnostics.flush()
        if self._artifacts is not None:
            self._artifacts.finalise(self._sensors)

    def run(
        self,
        duration_s: float | None = None,
        *,
        tick_hook: Callable[[Mapping[str, float]], None] | None = None,
    ) -> Iterable[ControlInputs]:
        """Drive the read→compute→actuate loop."""
        start = time.monotonic()
        scheduler = self._scheduler_factory(duration_s)
        with scheduler:
            try:
                for tick_index, _ in enumerate(scheduler.ticks()):
                    if self._artifacts is not None:
                        self._artifacts.record_scheduler_tick()
                    features_view, meta, control_inputs = self._wrangler.get_sensor_data()
                    feature_snapshot = dict(features_view.as_dict())
                    raw_command = self._controller.compute_torque(
                        feature_snapshot,
                        timestamp=meta.timestamp,
                    )
                    safe_command = self._safety.enforce(raw_command)
                    scheduler_snapshot: Mapping[str, float] = {
                        "loop_time_s": time.monotonic() - start,
                        "tick_index": float(tick_index),
                        "target_frequency_hz": self._config.frequency_hz,
                    }

                    timestamp = meta.timestamp
                    self._diagnostics.log_tick(
                        timestamp=float(timestamp),
                        features=feature_snapshot,
                        feature_packet=control_inputs,
                        torque_command_raw=raw_command,
                        torque_command_safe=safe_command,
                        scheduler=scheduler_snapshot,
                    )
                    if self._artifacts is not None:
                        try:
                            self._artifacts.publish_realtime(feature_snapshot, safe_command)
                        except Exception:  # pragma: no cover - defensive
                            self.LOGGER.exception("Diagnostics realtime streaming failed")
                    if tick_hook is not None:
                        try:
                            tick_hook(feature_snapshot)
                        except Exception:  # pragma: no cover - defensive guard
                            self.LOGGER.exception("Tick hook raised an exception; disabling hook")
                            tick_hook = None
                    self._actuator.apply(safe_command)
                    self._actuator.fault_if_needed()
                    yield control_inputs
                    if duration_s is not None and (time.monotonic() - start) >= duration_s:
                        break
            except Exception:
                raise
