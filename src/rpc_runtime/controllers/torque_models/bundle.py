"""Bundle-aware torque model that applies scaling and joint mapping."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Iterable, Mapping

from .base import TorqueModel


class BundleTorqueModel(TorqueModel):
    """Load an exported torque bundle and map outputs to runtime joints."""

    def __init__(
        self,
        bundle_path: str | Path,
        *,
        output_map: Mapping[str, str],
        subject_mass_kg: float | None = None,
        assistance_fraction: float = 1.0,
        backend: TorqueModel | None = None,
    ) -> None:
        """Create a bundle-backed torque model.

        Args:
            bundle_path: Directory containing `metadata.json`, `preprocessing.json`,
                and the model artifact.
            output_map: Mapping of runtime joint names to exported torque column names.
            subject_mass_kg: Participant mass in kilograms. Required when the exported
                torques are normalised by body mass (Nm/kg).
            assistance_fraction: Scalar applied after mass conversion (0..1 typical).
            backend: Optional pre-built torque model (used for testing). When omitted,
                the backend is inferred from the bundle's `format` field.
        """
        self._path = Path(bundle_path).expanduser().resolve()
        if not self._path.exists():
            raise FileNotFoundError(self._path)
        metadata_path = self._path / "metadata.json"
        preprocessing_path = self._path / "preprocessing.json"
        if not metadata_path.exists():
            raise FileNotFoundError(metadata_path)
        if not preprocessing_path.exists():
            raise FileNotFoundError(preprocessing_path)

        self._metadata = json.loads(metadata_path.read_text())
        preprocessing = json.loads(preprocessing_path.read_text())
        feature_names = preprocessing.get("feature_names")
        if not isinstance(feature_names, list) or not feature_names:
            raise ValueError("preprocessing.json missing feature_names")
        self._features = tuple(str(name) for name in feature_names)

        raw_torque_entries = self._metadata.get("torque_outputs", [])
        torque_units: Dict[str, str] = {}
        torque_order: list[str] = []
        for entry in raw_torque_entries:
            if isinstance(entry, Mapping):
                name = entry.get("name")
                unit = entry.get("unit", "Nm")
            else:
                name = entry
                unit = "Nm"
            if not name:
                continue
            name_str = str(name)
            torque_units[name_str] = str(unit)
            torque_order.append(name_str)
        self._torque_units = torque_units
        self._torque_order = tuple(torque_order) if torque_order else tuple(torque_units.keys())
        self._torque_index = {name: idx for idx, name in enumerate(self._torque_order)}
        self._torque_normalised = bool(self._metadata.get("torque_normalised_by_mass"))

        self._output_map = {str(joint): str(source) for joint, source in output_map.items()}
        if not self._output_map:
            raise ValueError("BundleTorqueModel requires a non-empty output_map")
        missing = [name for name in self._output_map.values() if name not in torque_units]
        if missing:
            raise ValueError(
                "Bundle torque bundle does not provide required torque outputs: "
                + ", ".join(sorted(set(missing)))
            )

        self._subject_mass_kg = float(subject_mass_kg) if subject_mass_kg is not None else None
        if self._torque_normalised and self._subject_mass_kg is None:
            raise ValueError(
                "subject_mass_kg must be provided when torque outputs are normalised by mass"
            )
        self._assistance_fraction = float(assistance_fraction)

        self._backend = backend or self._build_backend()
        if not isinstance(self._backend, TorqueModel):
            raise TypeError("backend must implement TorqueModel")

    def _build_backend(self) -> TorqueModel:
        format_name = str(self._metadata.get("format", "")).lower()
        if format_name == "onnx":
            try:  # pragma: no cover - optional dependency
                from .onnx_runtime import ONNXTorqueModel as OnnxTorqueModel
            except Exception as exc:
                raise RuntimeError("onnxruntime is not available for ONNX bundles") from exc
            return OnnxTorqueModel(self._path)
        if format_name == "torchscript":
            try:  # pragma: no cover - optional dependency
                from .torchscript import TorchscriptTorqueModel as TorchTorqueModel
            except Exception as exc:
                raise RuntimeError("PyTorch is not available for TorchScript bundles") from exc
            return TorchTorqueModel(self._path)
        raise ValueError(f"Unsupported bundle format '{format_name}'")

    def expected_features(self) -> Iterable[str]:
        """Return the ordered feature names required by the bundle."""
        return self._features

    def run(self, features: dict[str, float]) -> dict[str, float]:
        """Map incoming features through the underlying bundle backend."""
        backend_outputs = self._backend.run(dict(features))
        torques: Dict[str, float] = {}
        for joint, source in self._output_map.items():
            value_obj = backend_outputs.get(source)
            if value_obj is None:
                idx = self._torque_index.get(source)
                if idx is not None:
                    for fallback in (f"torque_{idx}", f"output_{idx}"):
                        if fallback in backend_outputs:
                            value_obj = backend_outputs[fallback]
                            break
            if value_obj is None:
                raise KeyError(f"Backend did not provide output '{source}'")
            value = float(value_obj)
            unit = self._torque_units.get(source, "Nm")
            if unit == "Nm_per_kg":
                if self._subject_mass_kg is None:
                    raise RuntimeError("subject_mass_kg required for Nm/kg outputs")
                value *= self._subject_mass_kg
            value *= self._assistance_fraction
            torques[joint] = value
        return torques
