"""Helpers for combining sensor sources."""

from __future__ import annotations

from dataclasses import dataclass

from .grf.base import VerticalGRFSample
from .imu.base import IMUSample


@dataclass(slots=True)
class ControlInputs:
    """Bundle of sensor readings consumed by the controller.

    Args:
        imu: Latest IMU sample.
        vertical_grf: Optional vertical GRF sample; `None` when unavailable.
    """

    imu: IMUSample
    vertical_grf: VerticalGRFSample | None = None
