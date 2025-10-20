"""Simulation and testing helpers for IMU interfaces."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, Iterator

from .base import BaseIMU, BaseIMUConfig, IMUSample


@dataclass(slots=True)
class SimIMUTrajectory:
    """Finite sequence of IMU samples used to drive a simulation."""

    samples: Iterable[IMUSample]


class SimulatedIMU(BaseIMU):
    """IMU implementation that replays a scripted trajectory or waveform."""

    supports_batch = True

    def __init__(self, trajectory: SimIMUTrajectory | None = None, hz: float = 200.0):
        """Create a simulated IMU with an optional scripted trajectory."""
        config = BaseIMUConfig(
            port_map={name: f"sim://{name}" for name in BaseIMU.HARDWARE_FEATURES}
        )
        super().__init__(config)
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
            omega = 2.0 * math.pi * self._hz
            angle = math.sin(omega * t)
            velocity = omega * math.cos(omega * t)
            values = {}
            for feature in self.hardware_features:
                semantics = self.feature_semantics(feature)
                if semantics.derivative >= 1:
                    values[feature] = velocity
                else:
                    values[feature] = angle
            sample = IMUSample(timestamp=t, values=values)
            return self._handle_sample(sample, fresh=True)
        try:
            sample = next(self._iterator)
        except StopIteration as exc:  # pragma: no cover - used in tests
            raise RuntimeError("Simulation exhausted") from exc
        return self._handle_sample(sample, fresh=True)
