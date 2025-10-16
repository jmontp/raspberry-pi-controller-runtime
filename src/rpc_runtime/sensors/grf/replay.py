"""Vertical GRF adapter that replays forces from an offline dataset."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import pandas as pd

from .base import BaseVerticalGRF, BaseVerticalGRFConfig, VerticalGRFSample

_CHANNEL_MAP = {
    "vertical_grf_ipsi_N": "vertical_grf_ipsi_BW",
    "vertical_grf_contra_N": "vertical_grf_contra_BW",
}


def _normalise_tasks(tasks: str | Iterable[str] | None) -> tuple[str, ...]:
    if tasks is None:
        return ()
    if isinstance(tasks, str):
        return (tasks,)
    return tuple(tasks)


@dataclass(slots=True)
class ReplayVerticalGRF(BaseVerticalGRF):
    """Replay vertical GRF samples using canonical column names."""

    path: str
    subject: str | None = None
    tasks: str | Iterable[str] | None = None
    start_index: int = 0
    max_samples: int | None = None
    loop: bool = False
    dt: float = 0.01
    body_mass_kg: float | None = None
    time_column: str | None = None
    config_override: BaseVerticalGRFConfig | None = None
    _frame: pd.DataFrame | None = field(init=False, default=None, repr=False)
    _timestamps: list[float] = field(init=False, default_factory=list, repr=False)
    _cursor: int = field(init=False, default=0, repr=False)

    CHANNEL_NAMES = ("vertical_grf_ipsi_N", "vertical_grf_contra_N")
    FORCE_CONVENTIONS = {
        "vertical_grf_ipsi_N": "Positive indicates upward (support) force in newtons (ipsilateral).",
        "vertical_grf_contra_N": "Positive indicates upward (support) force in newtons (contralateral).",
    }

    def __post_init__(self) -> None:
        config = self.config_override or BaseVerticalGRFConfig(channel_names=self.CHANNEL_NAMES)
        BaseVerticalGRF.__init__(self, config)
        self._tasks = _normalise_tasks(self.tasks)
        self._path = Path(self.path).expanduser()

    def probe(self) -> None:
        """Ensure the dataset exists before attempting to start."""
        if not self._path.exists():
            raise FileNotFoundError(self._path)

    def start(self) -> None:
        frame = self._load_frame()
        frame = self._apply_filters(frame)
        frame = self._slice_frame(frame)
        if frame.empty:
            raise ValueError("ReplayVerticalGRF dataset is empty after filtering/slicing")
        self._frame = frame.reset_index(drop=True)
        self._timestamps = self._extract_timestamps(self._frame)
        self._cursor = 0

    def stop(self) -> None:
        self._frame = None
        self._timestamps = []
        self._cursor = 0

    def read(self) -> VerticalGRFSample:
        if self._frame is None:
            raise RuntimeError("ReplayVerticalGRF.start() must be called before read()")
        rows = len(self._frame)
        if rows == 0:
            raise RuntimeError("ReplayVerticalGRF has no samples to replay")
        if self._cursor >= rows:
            if not self.loop:
                raise RuntimeError("ReplayVerticalGRF exhausted dataset samples")
            self._cursor = 0
        row = self._frame.iloc[self._cursor]
        timestamp = self._timestamps[self._cursor]
        self._cursor += 1

        forces = []
        scale = 9.80665 * self.body_mass_kg if self.body_mass_kg is not None else 1.0
        for channel in self.channel_names:
            source_column = _CHANNEL_MAP.get(channel, channel)
            if source_column not in row.index:
                raise KeyError(f"ReplayVerticalGRF missing required column '{source_column}'")
            forces.append(float(row[source_column]) * scale)

        sample = VerticalGRFSample(timestamp=timestamp, forces_newton=tuple(forces))
        return self._handle_sample(sample, fresh=True)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _load_frame(self) -> pd.DataFrame:
        suffix = self._path.suffix.lower()
        if suffix == ".parquet":
            return pd.read_parquet(self._path)
        if suffix in {".feather", ".arrow"}:
            return pd.read_feather(self._path)
        return pd.read_csv(self._path)

    def _apply_filters(self, frame: pd.DataFrame) -> pd.DataFrame:
        filtered = frame
        if self.subject is not None:
            if "subject" not in filtered.columns:
                raise ValueError("ReplayVerticalGRF cannot filter by subject; column missing")
            filtered = filtered[filtered["subject"] == self.subject]
        if self._tasks:
            if "task" not in filtered.columns:
                raise ValueError("ReplayVerticalGRF cannot filter by task; column missing")
            filtered = filtered[filtered["task"].isin(self._tasks)]
        return filtered

    def _slice_frame(self, frame: pd.DataFrame) -> pd.DataFrame:
        start = max(self.start_index, 0)
        if start >= len(frame):
            raise ValueError("ReplayVerticalGRF start_index beyond dataset length")
        if self.max_samples is None:
            return frame.iloc[start:]
        end = min(start + self.max_samples, len(frame))
        return frame.iloc[start:end]

    def _extract_timestamps(self, frame: pd.DataFrame) -> list[float]:
        if self.time_column and self.time_column in frame.columns:
            return [float(value) for value in frame[self.time_column].tolist()]
        return [idx * self.dt for idx in range(len(frame))]
