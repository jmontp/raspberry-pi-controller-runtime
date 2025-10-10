"""Mock IMU emitting a predefined sequence of samples."""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Iterable, Sequence

from .base import BaseIMU, BaseIMUConfig, IMUSample


def _default_samples(
    config: BaseIMUConfig,
    *,
    amplitude: float = 0.2,
) -> Sequence[IMUSample]:
    """Create a small sequence of random IMU samples for fall-back usage."""
    rng = random.Random(0)
    joints = len(config.joint_names)
    segments = len(config.segment_names)
    samples: list[IMUSample] = []
    for idx in range(10):
        timestamp = idx * 0.01
        joint_angles = tuple(rng.uniform(-amplitude, amplitude) for _ in range(joints))
        joint_vel = tuple(rng.uniform(-amplitude, amplitude) for _ in range(joints))
        seg_angles = tuple(rng.uniform(-amplitude, amplitude) for _ in range(segments))
        seg_vel = tuple(rng.uniform(-amplitude, amplitude) for _ in range(segments))
        samples.append(
            IMUSample(
                timestamp=timestamp,
                joint_angles_rad=joint_angles,
                joint_velocities_rad_s=joint_vel,
                segment_angles_rad=seg_angles,
                segment_velocities_rad_s=seg_vel,
            )
        )
    return samples


@dataclass(slots=True)
class MockIMU(BaseIMU):
    """Finite sample IMU for deterministic testing."""

    samples: Iterable[IMUSample] | None = None
    loop: bool = False
    config_override: BaseIMUConfig | None = None
    _samples: Sequence[IMUSample] = field(init=False, repr=False)
    _index: int = field(init=False, repr=False)

    def __post_init__(self) -> None:
        """Initialise the base class and materialise the sample sequence."""
        BaseIMU.__init__(self, self.config_override)
        if self.samples is None:
            materialized = tuple(_default_samples(self.config))
        else:
            materialized = tuple(self.samples)
            if not materialized:
                raise ValueError("MockIMU requires at least one sample when samples is provided")
        self._samples = materialized
        self._index = 0

    def start(self) -> None:
        """No-op start hook for compatibility with context manager usage."""
        return None

    def stop(self) -> None:
        """No-op stop hook for compatibility with context manager usage."""
        return None

    def read(self) -> IMUSample:
        """Return the next mock sample, optionally cycling when `loop` is set."""
        if not self._samples:
            raise RuntimeError("MockIMU sample sequence is empty")
        if self._index >= len(self._samples):
            if not self.loop:
                raise RuntimeError("MockIMU sample sequence exhausted")
            self._index = 0
        sample = self._samples[self._index]
        self._index += 1
        return self._handle_sample(sample, fresh=True)
