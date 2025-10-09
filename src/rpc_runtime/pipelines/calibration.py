"""Reusable calibration utilities for sensors."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class Zeroable(Protocol):
    def zero(self) -> None:
        ...


@dataclass(slots=True)
class CalibrationRoutine:
    """Container for calibration targets executed before runtime start."""

    imu: Zeroable | None = None
    grf: Zeroable | None = None

    def run(self) -> None:
        """Zero any provided sensor interfaces."""
        if self.imu is not None:
            self.imu.zero()
        if self.grf is not None:
            self.grf.zero()
