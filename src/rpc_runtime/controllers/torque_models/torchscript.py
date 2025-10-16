"""TorchScript torque model runtime."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

import torch

from .base import TorqueModel


class TorchscriptTorqueModel(TorqueModel):
    """Torque model executed via a TorchScript module."""

    def __init__(self, bundle_path: Path):
        """Load a TorchScript torque model exported for Raspberry Pi use.

        Args:
            bundle_path: Directory containing metadata, preprocessing, and `.ts` file.

        Raises:
            FileNotFoundError: When no TorchScript module is found in the bundle.
        """
        bundle_path = Path(bundle_path)
        metadata = json.loads((bundle_path / "metadata.json").read_text())
        preprocessing = json.loads((bundle_path / "preprocessing.json").read_text())
        self._features = tuple(preprocessing["feature_names"])
        scaler = preprocessing.get("scaler", {})
        self._mean = torch.tensor(
            scaler.get("mean", [0.0] * len(self._features)),
            dtype=torch.float32,
        )
        self._scale = torch.tensor(
            scaler.get("scale", [1.0] * len(self._features)),
            dtype=torch.float32,
        )
        model_files = list(bundle_path.glob("*.ts"))
        if not model_files:
            raise FileNotFoundError(f"No TorchScript module found in {bundle_path}")
        self._module = torch.jit.load(model_files[0])
        self._metadata = metadata
        torque_entries = metadata.get("torque_outputs", [])
        names: list[str] = []
        for entry in torque_entries:
            if isinstance(entry, dict):
                name = entry.get("name")
            else:
                name = entry
            if name is not None:
                names.append(str(name))
        self._torque_names = tuple(names)

    def expected_features(self) -> Iterable[str]:
        """Return the ordered feature list consumed by the TorchScript model."""
        return self._features

    def run(self, features: dict[str, float]) -> dict[str, float]:
        """Execute a forward pass through the TorchScript module."""
        ordered = torch.tensor([features[name] for name in self._features], dtype=torch.float32)
        ordered = (ordered - self._mean) / self._scale
        output = self._module(ordered.unsqueeze(0)).squeeze(0)
        values = output.detach().tolist()
        names = (
            self._torque_names
            if self._torque_names and len(self._torque_names) == len(values)
            else tuple(f"torque_{i}" for i in range(len(values)))
        )
        return {name: float(value) for name, value in zip(names, values, strict=False)}

    def warmup(self) -> None:
        """Run a dummy inference to pre-load TorchScript weights."""
        dummy = dict.fromkeys(self._features, 0.0)
        self.run(dummy)
