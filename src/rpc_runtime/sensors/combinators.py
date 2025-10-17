"""Helpers for combining sensor sources."""

from __future__ import annotations

from typing import Any, Mapping


class ControlInputs:
    """Bundle of sensor readings consumed by the controller."""

    __slots__ = ("_samples",)

    def __init__(
        self,
        *,
        samples: Mapping[str, Any] | None = None,
        **legacy_samples: Any,
    ) -> None:
        """Normalise provided sensor samples into a unified mapping."""
        data = dict(samples or {})
        data.update(legacy_samples)
        self._samples = data

    def get(self, alias: str, default: Any | None = None) -> Any | None:
        """Return the sample for a given sensor alias."""
        return self._samples.get(alias, default)

    def as_dict(self) -> Mapping[str, Any]:
        """Expose a copy of the samples mapping."""
        return dict(self._samples)

    @property
    def imu(self) -> Any | None:
        """Return the IMU sample when available."""
        return self._samples.get("imu")

    @property
    def vertical_grf(self) -> Any | None:
        """Return the vertical GRF sample when available."""
        return self._samples.get("vertical_grf")
