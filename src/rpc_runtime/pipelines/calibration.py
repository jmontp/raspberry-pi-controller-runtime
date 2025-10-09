"""Reusable calibration utilities for sensors."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class Zeroable(Protocol):
    def zero(self) -> None:
        ...


@dataclass(slots=True)
class CalibrationRoutine:
    imu: Zeroable | None = None
    grf: Zeroable | None = None

    def run(self) -> None:
        if self.imu is not None:
            self.imu.zero()
        if self.grf is not None:
            self.grf.zero()
