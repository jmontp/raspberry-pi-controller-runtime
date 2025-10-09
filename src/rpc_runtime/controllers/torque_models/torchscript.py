"""TorchScript torque model runtime."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

import torch

from .base import TorqueModel


class TorchscriptTorqueModel(TorqueModel):
    def __init__(self, bundle_path: Path):
        bundle_path = Path(bundle_path)
        metadata = json.loads((bundle_path / "metadata.json").read_text())
        preprocessing = json.loads((bundle_path / "preprocessing.json").read_text())
        self._features = tuple(preprocessing["feature_names"])
        scaler = preprocessing.get("scaler", {})
        self._mean = torch.tensor(scaler.get("mean", [0.0] * len(self._features)), dtype=torch.float32)
        self._scale = torch.tensor(scaler.get("scale", [1.0] * len(self._features)), dtype=torch.float32)
        model_files = list(bundle_path.glob("*.ts"))
        if not model_files:
            raise FileNotFoundError(f"No TorchScript module found in {bundle_path}")
        self._module = torch.jit.load(model_files[0])
        self._metadata = metadata

    def expected_features(self) -> Iterable[str]:
        return self._features

    def run(self, features: dict[str, float]) -> dict[str, float]:
        ordered = torch.tensor([features[name] for name in self._features], dtype=torch.float32)
        ordered = (ordered - self._mean) / self._scale
        with torch.no_grad():
            output = self._module(ordered.unsqueeze(0)).squeeze(0)
        return {f"torque_{i}": float(value) for i, value in enumerate(output)}

    def warmup(self) -> None:
        dummy = {name: 0.0 for name in self._features}
        self.run(dummy)
