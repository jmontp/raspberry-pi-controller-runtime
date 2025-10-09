"""Helpers for combining sensor sources."""

from __future__ import annotations

from dataclasses import dataclass

from .imu.base import IMUSample
from .grf.base import VerticalGRFSample


@dataclass(slots=True)
class ControlInputs:
    imu: IMUSample
    vertical_grf: VerticalGRFSample | None = None
