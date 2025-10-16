"""Fault-injection IMU mock for testing dropout scenarios."""

from __future__ import annotations

import random
from typing import Iterable, Tuple

from .base import BaseIMUConfig, IMUSample
from .mock import MockIMU


class MockFaultyIMU(MockIMU):
    """Mock IMU that can inject frame drops and full sensor dropouts."""

    def __init__(
        self,
        *,
        samples: Iterable[IMUSample] | None = None,
        loop: bool = False,
        config_override: BaseIMUConfig | None = None,
        drop_probability: float = 0.0,
        drop_range: Tuple[int, int] = (1, 5),
        dropout_probability: float = 0.0,
        dropout_frames: int | None = None,
        seed: int | None = None,
    ) -> None:
        if isinstance(config_override, dict):
            config_override = BaseIMUConfig(**config_override)
        super().__init__(samples=samples, loop=loop, config_override=config_override)
        if not 0.0 <= drop_probability <= 1.0:
            raise ValueError("drop_probability must be within [0.0, 1.0]")
        if not 0.0 <= dropout_probability <= 1.0:
            raise ValueError("dropout_probability must be within [0.0, 1.0]")
        low, high = drop_range
        if low < 0 or high < 0 or high < low:
            raise ValueError("drop_range must be non-negative with min <= max")
        if dropout_frames is not None and dropout_frames <= 0:
            raise ValueError("dropout_frames must be positive when provided")

        self.drop_probability = drop_probability
        self.drop_range = (low, high)
        self.dropout_probability = dropout_probability
        self.dropout_frames = dropout_frames
        self.seed = seed

        self._rng = random.Random(seed)
        self._frames_to_drop = 0
        self._dropout_remaining: int | None = None

    # ------------------------------------------------------------------
    # Fault control helpers
    # ------------------------------------------------------------------

    def trigger_dropout(self, frames: int | None = None) -> None:
        """Force a dropout for a given number of frames (or indefinitely)."""
        if frames is not None and frames <= 0:
            raise ValueError("dropout frames must be positive when provided")
        self._dropout_remaining = -1 if frames is None else frames

    def restore(self) -> None:
        """Reset any active dropout, resuming normal sampling."""
        self._dropout_remaining = None
        self._frames_to_drop = 0

    def force_drop_frames(self, count: int) -> None:
        """Force a burst drop for the next ``count`` frames."""
        if count < 0:
            raise ValueError("count must be non-negative")
        self._frames_to_drop = count

    # ------------------------------------------------------------------
    # Base overrides
    # ------------------------------------------------------------------

    def start(self) -> None:  # pragma: no cover - parity with parent
        super().start()
        self._frames_to_drop = 0
        self._dropout_remaining = None

    def read(self) -> IMUSample:
        # Ongoing dropout takes precedence
        if self._dropout_remaining is not None:
            if self._dropout_remaining == -1:
                return self._handle_sample(None, fresh=False)
            if self._dropout_remaining > 0:
                self._dropout_remaining -= 1
                return self._handle_sample(None, fresh=False)
            # dropout completed
            self._dropout_remaining = None

        # Possibly initiate a new dropout window
        if self.dropout_probability > 0.0 and self._rng.random() < self.dropout_probability:
            if self.dropout_frames is None:
                self._dropout_remaining = -1
            else:
                frames = max(1, self.dropout_frames)
                self._dropout_remaining = frames - 1
            return self._handle_sample(None, fresh=False)

        # Handle short burst frame drops
        if self._frames_to_drop > 0:
            self._frames_to_drop -= 1
            return self._handle_sample(None, fresh=False)
        if self.drop_probability > 0.0 and self._rng.random() < self.drop_probability:
            low, high = self.drop_range
            drop_count = self._rng.randint(low, high)
            if drop_count > 0:
                self._frames_to_drop = drop_count - 1
                return self._handle_sample(None, fresh=False)

        return super().read()
