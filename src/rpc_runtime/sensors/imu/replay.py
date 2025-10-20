"""IMU adapter that replays canonical signals from an offline dataset."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Iterable

import pandas as pd

from .base import BaseIMU, BaseIMUConfig, IMUSample


def _normalise_tasks(tasks: str | Iterable[str] | None) -> tuple[str, ...]:
    if tasks is None:
        return ()
    if isinstance(tasks, str):
        return (tasks,)
    return tuple(tasks)


@dataclass(slots=True)
class ReplayIMU(BaseIMU):
    """Replay IMU data from a pre-recorded dataset with canonical column names."""

    path: str
    subject: str | None = None
    tasks: str | Iterable[str] | None = None
    start_index: int = 0
    max_samples: int | None = None
    loop: bool = False
    dt: float = 0.01
    time_column: str | None = None
    config_override: BaseIMUConfig | dict | None = None
    _frame: pd.DataFrame | None = field(init=False, default=None, repr=False)
    _timestamps: list[float] = field(init=False, default_factory=list, repr=False)
    _feature_columns: tuple[str, ...] = field(init=False, default=(), repr=False)
    _cursor: int = field(init=False, default=0, repr=False)

    def __post_init__(self) -> None:
        """Initialise base configuration; dataset is loaded on start()."""
        override = self.config_override
        if isinstance(override, BaseIMUConfig):
            config = override
        elif isinstance(override, dict):
            config = BaseIMUConfig(**override)
        else:
            config = None
        if config is None:
            config = BaseIMUConfig(port_map=self._default_port_map())
        elif not config.port_map:
            config = replace(config, port_map=self._default_port_map())
        BaseIMU.__init__(self, config)
        self._tasks = _normalise_tasks(self.tasks)
        self._path = Path(self.path).expanduser()

    def _default_port_map(self) -> dict[str, str]:
        return {name: f"replay://{name}" for name in BaseIMU.HARDWARE_FEATURES}

    def probe(self) -> None:
        """Ensure the dataset exists before attempting to start."""
        if not self._path.exists():
            raise FileNotFoundError(self._path)

    def start(self) -> None:
        """Load and filter the dataset to prepare for replay."""
        frame = self._load_frame()
        frame = self._apply_filters(frame)
        frame = self._slice_frame(frame)
        if frame.empty:
            raise ValueError("ReplayIMU dataset is empty after filtering/slicing")
        frame = frame.reset_index(drop=True)
        canonical_columns = [
            column for column in frame.columns if column in self.CANONICAL_FEATURES
        ]
        if not canonical_columns:
            raise ValueError("ReplayIMU dataset does not contain canonical IMU columns")
        required = tuple(self.config.port_map.keys())
        missing = [name for name in required if name not in canonical_columns]
        if missing:
            raise ValueError(
                "ReplayIMU dataset missing required canonical columns: "
                + ", ".join(sorted(missing))
            )
        self._feature_columns = tuple(sorted(set(canonical_columns)))
        self._frame = frame
        self._timestamps = self._extract_timestamps(frame)
        self._cursor = 0

    def stop(self) -> None:
        """Reset cached dataset state."""
        self._frame = None
        self._timestamps = []
        self._feature_columns = ()
        self._cursor = 0

    def read(self) -> IMUSample:
        """Return the next IMU sample, optionally looping when configured."""
        if self._frame is None:
            raise RuntimeError("ReplayIMU.start() must be called before read()")
        rows = len(self._frame)
        if rows == 0:
            raise RuntimeError("ReplayIMU has no samples to replay")
        if self._cursor >= rows:
            if not self.loop:
                raise RuntimeError("ReplayIMU exhausted dataset samples")
            self._cursor = 0
        row = self._frame.iloc[self._cursor]
        timestamp = self._timestamps[self._cursor]
        self._cursor += 1

        values = {}
        for feature in self._feature_columns:
            if feature in row.index:
                values[feature] = float(row[feature])
        sample = IMUSample(timestamp=timestamp, values=values)
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
                raise ValueError("ReplayIMU cannot filter by subject; column missing")
            filtered = filtered[filtered["subject"] == self.subject]
        if self._tasks:
            if "task" not in filtered.columns:
                raise ValueError("ReplayIMU cannot filter by task; column missing")
            filtered = filtered[filtered["task"].isin(self._tasks)]
        return filtered

    def _slice_frame(self, frame: pd.DataFrame) -> pd.DataFrame:
        start = max(self.start_index, 0)
        if start >= len(frame):
            raise ValueError("ReplayIMU start_index beyond dataset length")
        if self.max_samples is None:
            return frame.iloc[start:]
        end = min(start + self.max_samples, len(frame))
        return frame.iloc[start:end]

    def _extract_timestamps(self, frame: pd.DataFrame) -> list[float]:
        if self.time_column and self.time_column in frame.columns:
            return [float(value) for value in frame[self.time_column].tolist()]
        return [idx * self.dt for idx in range(len(frame))]
