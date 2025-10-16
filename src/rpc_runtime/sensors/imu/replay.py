"""IMU adapter that replays canonical signals from an offline dataset."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import pandas as pd

from .base import BaseIMU, BaseIMUConfig, IMUSample

_JOINT_ANGLE_COLUMNS = {
    "hip_r": "hip_flexion_angle_ipsi_rad",
    "knee_r": "knee_flexion_angle_ipsi_rad",
    "ankle_r": "ankle_dorsiflexion_angle_ipsi_rad",
    "hip_l": "hip_flexion_angle_contra_rad",
    "knee_l": "knee_flexion_angle_contra_rad",
    "ankle_l": "ankle_dorsiflexion_angle_contra_rad",
}

_JOINT_VELOCITY_COLUMNS = {
    "hip_r": "hip_flexion_velocity_ipsi_rad_s",
    "knee_r": "knee_flexion_velocity_ipsi_rad_s",
    "ankle_r": "ankle_dorsiflexion_velocity_ipsi_rad_s",
    "hip_l": "hip_flexion_velocity_contra_rad_s",
    "knee_l": "knee_flexion_velocity_contra_rad_s",
    "ankle_l": "ankle_dorsiflexion_velocity_contra_rad_s",
}

_SEGMENT_ANGLE_COLUMNS = {
    "trunk": "trunk_sagittal_angle_rad",
    "thigh_r": "thigh_sagittal_angle_ipsi_rad",
    "shank_r": "shank_sagittal_angle_ipsi_rad",
    "foot_r": "foot_sagittal_angle_ipsi_rad",
    "thigh_l": "thigh_sagittal_angle_contra_rad",
    "shank_l": "shank_sagittal_angle_contra_rad",
    "foot_l": "foot_sagittal_angle_contra_rad",
}

_SEGMENT_VELOCITY_COLUMNS = {
    "trunk": "trunk_sagittal_velocity_rad_s",
    "thigh_r": "thigh_sagittal_velocity_ipsi_rad_s",
    "shank_r": "shank_sagittal_velocity_ipsi_rad_s",
    "foot_r": "foot_sagittal_velocity_ipsi_rad_s",
    "thigh_l": "thigh_sagittal_velocity_contra_rad_s",
    "shank_l": "shank_sagittal_velocity_contra_rad_s",
    "foot_l": "foot_sagittal_velocity_contra_rad_s",
}


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
        BaseIMU.__init__(self, config)
        self._tasks = _normalise_tasks(self.tasks)
        self._path = Path(self.path).expanduser()

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
        self._frame = frame.reset_index(drop=True)
        self._timestamps = self._extract_timestamps(self._frame)
        self._cursor = 0

    def stop(self) -> None:
        """Reset cached dataset state."""
        self._frame = None
        self._timestamps = []
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

        joint_angles = self._extract_joint_tuple(row, _JOINT_ANGLE_COLUMNS, required=True)
        joint_velocities = self._extract_joint_tuple(row, _JOINT_VELOCITY_COLUMNS, required=False)
        segment_angles = self._extract_segment_tuple(row, _SEGMENT_ANGLE_COLUMNS)
        segment_velocities = self._extract_segment_tuple(row, _SEGMENT_VELOCITY_COLUMNS)

        sample = IMUSample(
            timestamp=timestamp,
            joint_angles_rad=joint_angles,
            joint_velocities_rad_s=joint_velocities,
            segment_angles_rad=segment_angles,
            segment_velocities_rad_s=segment_velocities,
        )
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

    def _extract_joint_tuple(
        self,
        row,
        column_map: dict[str, str],
        *,
        required: bool,
    ) -> tuple[float, ...]:
        values: list[float] = []
        for joint in self.joint_names:
            column = column_map.get(joint)
            if column is None:
                values.append(0.0)
                continue
            if column not in row.index:
                if required:
                    raise KeyError(f"ReplayIMU missing required column '{column}'")
                values.append(0.0)
                continue
            values.append(float(row[column]))
        return tuple(values)

    def _extract_segment_tuple(
        self,
        row,
        column_map: dict[str, str],
    ) -> tuple[float, ...]:
        values: list[float] = []
        for segment in self.segment_names:
            column = column_map.get(segment)
            if column is None or column not in row.index:
                values.append(0.0)
            else:
                values.append(float(row[column]))
        return tuple(values)
