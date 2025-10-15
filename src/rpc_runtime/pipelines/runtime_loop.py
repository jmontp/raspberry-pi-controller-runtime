"""Runtime pipeline orchestrating sensors, controller, and actuators."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Iterable, Mapping

from ..actuators.base import BaseActuator
from ..controllers.pi_controller import PIController
from ..sensors.combinators import ControlInputs
from ..sensors.grf.base import BaseVerticalGRF
from ..sensors.imu.base import BaseIMU
from .diagnostics import DiagnosticsSink, NoOpDiagnosticsSink
from .data_wrangler import DataWrangler
from .scheduler import BaseScheduler, SimpleScheduler


@dataclass(slots=True)
class RuntimeLoopConfig:
    """Configuration parameters for `RuntimeLoop`."""

    frequency_hz: float
    profile_name: str = "default"


class RuntimeLoop:
    """Orchestrate sensor polling, controller execution, and actuation."""

    def __init__(
        self,
        config: RuntimeLoopConfig,
        imu: BaseIMU,
        actuator: BaseActuator,
        controller: PIController,
        vertical_grf: BaseVerticalGRF | None = None,
        scheduler_factory: Callable[[float | None], BaseScheduler] | None = None,
        diagnostics: DiagnosticsSink | None = None,
    ) -> None:
        """Initialise the runtime loop with hardware interfaces and scheduling."""
        self._config = config
        self._imu = imu
        self._actuator = actuator
        self._controller = controller
        self._vertical_grf = vertical_grf
        self._scheduler_factory = scheduler_factory or (
            lambda duration: SimpleScheduler(config.frequency_hz, duration)
        )
        self._diagnostics: DiagnosticsSink = diagnostics or NoOpDiagnosticsSink()
        # Opportunistically create a DataWrangler when the controller declares
        # an input schema. This keeps backward compatibility while aligning
        # with the new architecture.
        input_schema = getattr(controller, "_input_schema", None)
        self._wrangler: DataWrangler | None = None
        if input_schema is not None:
            try:
                self._wrangler = DataWrangler(
                    required_inputs=input_schema,
                    imu=self._imu,
                    vertical_grf=self._vertical_grf,
                    diagnostics_sink=self._diagnostics,
                )
            except Exception:
                # If wrangler construction fails for any reason, fall back to
                # legacy direct sensor polling.
                self._wrangler = None

    def __enter__(self) -> "RuntimeLoop":
        """Enter the loop context by initialising sensors and actuators."""
        if self._wrangler is not None:
            self._wrangler.__enter__()
        else:
            self._imu.__enter__()
            if self._vertical_grf is not None:
                self._vertical_grf.__enter__()
        self._actuator.__enter__()
        self._controller.reset()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Tear down resources when leaving the context."""
        self._actuator.__exit__(exc_type, exc, tb)
        if self._wrangler is not None:
            self._wrangler.__exit__(exc_type, exc, tb)
        else:
            if self._vertical_grf is not None:
                self._vertical_grf.__exit__(exc_type, exc, tb)
            self._imu.__exit__(exc_type, exc, tb)
        self._diagnostics.flush()

    def run(self, duration_s: float | None = None) -> Iterable[ControlInputs]:
        """Drive the read→compute→actuate loop."""
        start = time.monotonic()
        scheduler = self._scheduler_factory(duration_s)
        with scheduler:
            try:
                for tick_index, _ in enumerate(scheduler.ticks()):
                    if self._wrangler is not None:
                        _, meta, control_inputs = self._wrangler.get_sensor_data()
                        imu_sample = control_inputs.imu
                    else:
                        imu_sample = self._imu.read()
                        grf_sample = (
                            self._vertical_grf.read()
                            if self._vertical_grf is not None
                            else None
                        )
                        control_inputs = ControlInputs(imu=imu_sample, vertical_grf=grf_sample)
                    raw_command = self._controller.tick(control_inputs)
                    safe_command = raw_command
                    scheduler_snapshot: Mapping[str, float] = {
                        "loop_time_s": time.monotonic() - start,
                        "tick_index": float(tick_index),
                        "target_frequency_hz": self._config.frequency_hz,
                    }
                    self._diagnostics.log_tick(
                        timestamp=imu_sample.timestamp,
                        feature_packet=control_inputs,
                        torque_command_raw=raw_command,
                        torque_command_safe=safe_command,
                        scheduler=scheduler_snapshot,
                    )
                    self._actuator.apply(safe_command)
                    self._actuator.fault_if_needed()
                    yield control_inputs
                    if duration_s is not None and (time.monotonic() - start) >= duration_s:
                        break
            except Exception:
                raise
