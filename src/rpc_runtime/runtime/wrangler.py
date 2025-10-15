"""Data wrangling utilities that normalize sensor inputs into features."""

from __future__ import annotations

import types
from dataclasses import dataclass
from typing import Mapping, MutableMapping

from ..config.models import HardwareAvailabilityError, InputSchema
from ..sensors.combinators import ControlInputs
from ..sensors.grf.base import BaseVerticalGRF
from ..sensors.imu.base import BaseIMU


@dataclass(slots=True)
class FeatureMetadata:
    """Per-tick metadata accompanying a feature view."""

    timestamp: float
    stale: bool = False


@dataclass(slots=True)
class FeatureView:
    """Read-only mapping-like view over a feature dictionary."""

    _data: Mapping[str, float]

    def __getitem__(self, key: str) -> float:
        """Return the value for a canonical feature name."""
        return self._data[key]

    def get(self, key: str, default: float | None = None) -> float | None:
        """Fetch a feature value or return ``default`` when absent."""
        return self._data.get(key, default)

    def items(self):  # pragma: no cover - convenience
        """Iterate over ``(name, value)`` pairs in the view."""
        return self._data.items()

    def as_dict(self) -> Mapping[str, float]:  # pragma: no cover - convenience
        """Expose the underlying mapping without copying."""
        return self._data


class DataWrangler:
    """Plan and serve controller features from sensor inputs.

    Parameters
    ----------
    required_inputs:
        The `InputSchema` describing required canonical signals.
    imu:
        IMU instance providing joint/segment data.
    vertical_grf:
        Optional vertical GRF sensor.
    diagnostics_sink:
        Optional diagnostics sink for readiness/coverage.
    """

    def __init__(
        self,
        required_inputs: InputSchema,
        *,
        imu: BaseIMU,
        vertical_grf: BaseVerticalGRF | None = None,
        diagnostics_sink=None,
    ) -> None:
        """Create a new wrangler and validate required signals against hardware.

        Args:
            required_inputs: Input schema describing canonical controller features.
            imu: IMU instance providing joint/segment data.
            vertical_grf: Optional vertical GRF sensor instance.
            diagnostics_sink: Optional diagnostics sink for readiness/coverage.
        """
        self._schema = required_inputs
        self._imu = imu
        self._vertical_grf = vertical_grf
        self._diagnostics = diagnostics_sink
        # Validate immediately that the configured hardware can, in principle,
        # provide required signals.
        available = {"knee_angle", "knee_velocity", "ankle_angle", "ankle_velocity"}
        if vertical_grf is not None:
            available.add("grf_total")
        # Allow the schema to raise a meaningful error if required channels
        # cannot be satisfied.
        self._schema.validate_signals(available)

    def __enter__(self) -> "DataWrangler":  # pragma: no cover - simple delegation
        """Start hardware streams when entering a context."""
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # pragma: no cover - simple delegation
        """Stop hardware streams when leaving a context."""
        self.stop()

    def probe(self) -> None:
        """Perform lightweight checks that hardware can start.

        In this minimal implementation we trust construction-time validation.
        """
        return None

    def start(self) -> None:
        """Start the IMU and optional GRF streams."""
        self._imu.start()
        if self._vertical_grf is not None:
            self._vertical_grf.start()

    def stop(self) -> None:
        """Stop hardware streams to release resources."""
        if self._vertical_grf is not None:
            self._vertical_grf.stop()
        self._imu.stop()

    def get_sensor_data(self) -> tuple[FeatureView, FeatureMetadata, ControlInputs]:
        """Fetch the latest canonical features and metadata.

        Returns a read-only feature view, lightweight metadata (timestamp,
        staleness), and the raw `ControlInputs` for compatibility with existing
        controllers expecting the legacy shape.
        """
        imu_sample = self._imu.read()
        grf_sample = self._vertical_grf.read() if self._vertical_grf is not None else None
        control_inputs = ControlInputs(imu=imu_sample, vertical_grf=grf_sample)

        try:
            dense_features: MutableMapping[str, float] = self._schema.build_features(control_inputs)
        except HardwareAvailabilityError:
            # Re-raise for the runtime to handle according to policy.
            raise

        # Freeze the view to discourage mutation.
        view = FeatureView(types.MappingProxyType(dict(dense_features)))
        meta = FeatureMetadata(timestamp=imu_sample.timestamp, stale=False)
        return view, meta, control_inputs

