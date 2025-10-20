"""ONNX torque model runtime following torque-modeling/docs/raspberry_pi.md."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

import numpy as np

from .base import TorqueModel

try:  # pragma: no cover - optional dependency
    import onnxruntime as ort
except ImportError:  # pragma: no cover - dev environments without onnxruntime
    ort = None


class ONNXTorqueModel(TorqueModel):
    """Torque model backed by an ONNX inference session."""

    def __init__(self, bundle_path: Path):
        """Load an exported ONNX bundle produced by the torque-modeling project.

        Args:
            bundle_path: Directory containing `metadata.json`, `preprocessing.json`,
                and an ONNX model file.

        Raises:
            RuntimeError: If `onnxruntime` is unavailable in the environment.
            FileNotFoundError: When the bundle is missing an ONNX model file.
        """
        if ort is None:
            raise RuntimeError("onnxruntime is required for ONNX torque models")
        bundle_path = Path(bundle_path)
        metadata = json.loads((bundle_path / "metadata.json").read_text())
        preprocessing = json.loads((bundle_path / "preprocessing.json").read_text())
        self._features = tuple(preprocessing["feature_names"])
        scaler = preprocessing.get("scaler", {})
        self._mean = np.array(scaler.get("mean", [0.0] * len(self._features)), dtype=np.float32)
        self._scale = np.array(scaler.get("scale", [1.0] * len(self._features)), dtype=np.float32)
        model_files = list(bundle_path.glob("*.onnx"))
        if not model_files:
            raise FileNotFoundError(f"No ONNX model found in {bundle_path}")
        self._session = ort.InferenceSession(model_files[0].read_bytes())
        self._input_name = self._session.get_inputs()[0].name
        self._outputs = tuple(output.name for output in self._session.get_outputs())
        self._metadata = metadata

    def expected_features(self) -> Iterable[str]:
        """Return the ordered feature list expected by the model."""
        return self._features

    def run(self, features: dict[str, float]) -> dict[str, float]:
        """Execute a forward pass using the provided feature mapping."""
        ordered = np.array([features[name] for name in self._features], dtype=np.float32)
        ordered = (ordered - self._mean) / self._scale
        result = self._session.run(self._outputs, {self._input_name: ordered.reshape(1, -1)})
        return {name: float(array[0]) for name, array in zip(self._outputs, result, strict=False)}

    def warmup(self) -> None:
        """Run a dummy inference to prime caches before realtime execution."""
        dummy = dict.fromkeys(self._features, 0.0)
        self.run(dummy)
