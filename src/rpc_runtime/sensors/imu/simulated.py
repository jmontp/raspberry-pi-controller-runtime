"""Simulation and testing helpers for IMU interfaces."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, Iterator

from .base import BaseIMU, IMUSample


@dataclass(slots=True)
class SimIMUTrajectory:
    """Finite sequence of IMU samples used to drive a simulation."""

    samples: Iterable[IMUSample]


class SimulatedIMU(BaseIMU):
    """IMU implementation that replays a scripted trajectory or waveform."""

    supports_batch = True

    def __init__(self, trajectory: SimIMUTrajectory | None = None, hz: float = 200.0):
        """Create a simulated IMU with an optional scripted trajectory."""
        self._trajectory = trajectory
        self._hz = hz
        self._iterator: Iterator[IMUSample] | None = None
        self._start = 0.0

    def start(self) -> None:
        """Initialise the simulation clock and iterator state."""
        self._start = time.monotonic()
        if self._trajectory is not None:
            self._iterator = iter(self._trajectory.samples)

    def stop(self) -> None:
        """Clear any iterators created during :meth:`start`."""
        self._iterator = None

    def read(self) -> IMUSample:
        """Return the next simulated IMU sample."""
        if self._iterator is None:
            t = time.monotonic() - self._start
            angle = math.sin(2 * math.pi * t)
            return IMUSample(
                timestamp=t,
                joint_angles_rad=(angle,),
                joint_velocities_rad_s=(math.cos(2 * math.pi * t),),
                segment_angles_rad=(angle,),
                segment_velocities_rad_s=(math.cos(2 * math.pi * t),),
            )
        try:
            sample = next(self._iterator)
        except StopIteration as exc:  # pragma: no cover - used in tests
            raise RuntimeError("Simulation exhausted") from exc
        return sample
