"""Runtime pipeline orchestrating sensors, controller, and actuators."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Iterable

from ..actuators.base import BaseActuator
from ..controllers.pi_controller import PIController
from ..sensors.combinators import ControlInputs
from ..sensors.grf.base import BaseVerticalGRF
from ..sensors.imu.base import BaseIMU
from .scheduler import BaseScheduler, SimpleScheduler


@dataclass(slots=True)
class RuntimeLoopConfig:
    frequency_hz: float
    profile_name: str = "default"


class RuntimeLoop:
    def __init__(
        self,
        config: RuntimeLoopConfig,
        imu: BaseIMU,
        actuator: BaseActuator,
        controller: PIController,
        vertical_grf: BaseVerticalGRF | None = None,
        scheduler_factory: Callable[[float | None], BaseScheduler] | None = None,
    ) -> None:
        self._config = config
        self._imu = imu
        self._actuator = actuator
        self._controller = controller
        self._vertical_grf = vertical_grf
        self._scheduler_factory = scheduler_factory or (
            lambda duration: SimpleScheduler(config.frequency_hz, duration)
        )

    def __enter__(self) -> "RuntimeLoop":
        self._imu.__enter__()
        if self._vertical_grf is not None:
            self._vertical_grf.__enter__()
        self._actuator.__enter__()
        self._controller.reset()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self._actuator.__exit__(exc_type, exc, tb)
        if self._vertical_grf is not None:
            self._vertical_grf.__exit__(exc_type, exc, tb)
        self._imu.__exit__(exc_type, exc, tb)

    def run(self, duration_s: float | None = None) -> Iterable[ControlInputs]:
        start = time.monotonic()
        scheduler = self._scheduler_factory(duration_s)
        with scheduler:
            for _ in scheduler.ticks():
                imu_sample = self._imu.read()
                grf_sample = (
                    self._vertical_grf.read() if self._vertical_grf is not None else None
                )
                control_inputs = ControlInputs(imu=imu_sample, vertical_grf=grf_sample)
                command = self._controller.tick(control_inputs)
                self._actuator.apply(command)
                self._actuator.fault_if_needed()
                yield control_inputs
                if duration_s is not None and (time.monotonic() - start) >= duration_s:
                    break
