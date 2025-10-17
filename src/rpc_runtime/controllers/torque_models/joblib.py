"""Joblib-based torque model loader supporting KNN and VAE exports."""

from __future__ import annotations

import importlib
import json
from pathlib import Path
from typing import Iterable, Mapping

import numpy as np
import pandas as pd

from .base import TorqueModel

try:  # pragma: no cover - optional dependency resolved at runtime
    import joblib
except ImportError:  # pragma: no cover - handled during initialisation
    joblib = None


class JoblibTorqueModel(TorqueModel):
    """Load torque heads exported as joblib bundles (e.g., KNN, VAE)."""

    def __init__(
        self,
        *,
        model_path: str,
        loader: str,
        scaler_path: str | None = None,
        output_map: Mapping[str, str] | None = None,
        subject_mass_kg: float | None = None,
        assistance_fraction: float = 1.0,
        training_metadata: str | None = None,
    ) -> None:
        """Create a joblib-backed torque model runner.

        Args:
            model_path: Path to the `model.joblib` file exported from torque-modeling.
            loader: Import path to a callable returning a loaded model instance
                (e.g., ``torque_modeling.architectures.knn.model.load_model``).
            scaler_path: Optional path to the paired ``scaler.joblib`` used during
                preprocessing. When supplied, incoming features are transformed to match
                the training distribution.
            output_map: Mapping from runtime joint names to torque columns produced by
                the loaded model. Defaults to a 1:1 mapping of the model's torque columns.
            subject_mass_kg: When torque outputs are normalised by mass (Nm/kg), provide
                the subject mass to recover absolute torques.
            assistance_fraction: Scalar applied to all torques (0..1 typical).
            training_metadata: Optional path to ``training_metadata.json`` for auxiliary
                information (torque column ordering) when the model object does not expose
                it directly.
        """
        if joblib is None:
            raise RuntimeError("joblib is required to load torque-modeling joblib bundles")

        self._model_path = Path(model_path).expanduser().resolve()
        if not self._model_path.exists():
            raise FileNotFoundError(self._model_path)
        self._loader = self._resolve_loader(loader)
        self._model = self._loader(str(self._model_path))

        scaler_path_obj = Path(scaler_path).expanduser().resolve() if scaler_path else None
        if scaler_path_obj is not None:
            if not scaler_path_obj.exists():
                raise FileNotFoundError(scaler_path_obj)
            self._scaler = joblib.load(scaler_path_obj)
        else:
            self._scaler = None

        self._features = self._extract_feature_names()
        if not self._features:
            raise RuntimeError("Loaded joblib model does not expose input feature names")

        self._torque_columns = self._extract_torque_columns(training_metadata)
        if not self._torque_columns:
            raise RuntimeError("Unable to determine torque output columns for joblib model")

        if output_map:
            missing = [source for source in output_map.values() if source not in self._torque_columns]
            if missing:
                raise ValueError(f"output_map references unknown torque columns: {missing}")
            self._output_map = dict(output_map)
        else:
            self._output_map = {name: name for name in self._torque_columns}

        self._subject_mass_kg = float(subject_mass_kg) if subject_mass_kg is not None else None
        self._assistance_fraction = float(assistance_fraction)
        if not np.isfinite(self._assistance_fraction):
            raise ValueError("assistance_fraction must be finite")

    # ------------------------------------------------------------------
    # TorqueModel interface
    # ------------------------------------------------------------------

    def expected_features(self) -> Iterable[str]:
        return self._features

    def run(self, features: dict[str, float]) -> dict[str, float]:
        row = {name: float(features.get(name, 0.0)) for name in self._features}
        df = pd.DataFrame([row], columns=self._features)
        if self._scaler is not None:
            scaled = self._scaler.transform(df.to_numpy(dtype=np.float32, copy=False))
            df = pd.DataFrame(scaled, columns=self._features)

        predictions = self._model.predict(df)
        torque_df = getattr(predictions, "torque", None)
        if torque_df is None or torque_df.empty:
            return {}

        torque_row = torque_df.iloc[0]
        outputs: dict[str, float] = {}
        for joint, source in self._output_map.items():
            value = float(torque_row.get(source, 0.0))
            if self._subject_mass_kg is not None and source.endswith("_Nm_kg"):
                value *= self._subject_mass_kg
            outputs[joint] = value * self._assistance_fraction
        return outputs

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _resolve_loader(path: str):
        if "." not in path:
            raise ValueError("loader must be a fully-qualified import path")
        module_name, attr = path.rsplit(".", 1)
        module = importlib.import_module(module_name)
        loader = getattr(module, attr, None)
        if loader is None:
            raise AttributeError(f"{path} is not importable")
        return loader

    def _extract_feature_names(self) -> tuple[str, ...]:
        spec = getattr(self._model, "get_input_spec", lambda: None)()
        if spec is not None and getattr(spec, "feature_names", None):
            return tuple(spec.feature_names)
        feature_names = getattr(self._model, "feature_names", None)
        if feature_names:
            return tuple(feature_names)
        return ()

    def _extract_torque_columns(self, training_metadata: str | None) -> tuple[str, ...]:
        columns = getattr(self._model, "torque_cols", None)
        if not columns:
            columns = getattr(self._model, "torque_columns", None)
        if columns:
            return tuple(columns)

        metadata_path = None
        if training_metadata:
            metadata_path = Path(training_metadata).expanduser().resolve()
        else:
            candidate = self._model_path.parent / "training_metadata.json"
            if candidate.exists():
                metadata_path = candidate
        if metadata_path and metadata_path.exists():
            try:
                payload = json.loads(metadata_path.read_text())
                cols = payload.get("torque_cols")
                if cols:
                    return tuple(cols)
            except Exception:  # pragma: no cover - best effort fallback
                pass
        return ()
