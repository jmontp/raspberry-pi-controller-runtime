from __future__ import annotations

import json
from pathlib import Path

import pytest

from rpc_runtime.controllers.torque_models.base import TorqueModel
from rpc_runtime.controllers.torque_models.bundle import BundleTorqueModel


class DummyBackend(TorqueModel):
    def __init__(self, outputs: dict[str, float], features: tuple[str, ...]):
        self._outputs = dict(outputs)
        self._features = features

    def run(self, features: dict[str, float]) -> dict[str, float]:
        return dict(self._outputs)

    def expected_features(self):
        return self._features


def _write_bundle(tmp_path: Path, *, format_name: str = "torchscript") -> Path:
    metadata = {
        "format": format_name,
        "arch": "test_arch",
        "run_name": "demo_run",
        "run_id": "demo123",
        "input_features": ["feature_a", "feature_b"],
        "torque_outputs": [
            {"name": "knee_flexion_moment_ipsi_Nm_kg", "unit": "Nm_per_kg"},
            {"name": "ankle_dorsiflexion_moment_ipsi_Nm", "unit": "Nm"},
        ],
        "torque_normalised_by_mass": True,
    }
    preprocessing = {"feature_names": ["feature_a", "feature_b"]}
    bundle_dir = tmp_path / "bundle"
    bundle_dir.mkdir()
    (bundle_dir / "metadata.json").write_text(json.dumps(metadata))
    (bundle_dir / "preprocessing.json").write_text(json.dumps(preprocessing))
    # Create dummy artefact so filesystem looks complete
    (bundle_dir / "energy_shaping.ts").write_text("stub")
    return bundle_dir


def test_bundle_torque_model_scales_mass_and_assistance(tmp_path):
    bundle_dir = _write_bundle(tmp_path)
    backend = DummyBackend(
        outputs={
            "knee_flexion_moment_ipsi_Nm_kg": 0.5,
            "ankle_dorsiflexion_moment_ipsi_Nm": 10.0,
        },
        features=("feature_a", "feature_b"),
    )
    model = BundleTorqueModel(
        bundle_dir,
        output_map={
            "knee_flexion_moment_ipsi_Nm": "knee_flexion_moment_ipsi_Nm_kg",
            "ankle_dorsiflexion_moment_ipsi_Nm": "ankle_dorsiflexion_moment_ipsi_Nm",
        },
        subject_mass_kg=70.0,
        assistance_fraction=0.4,
        backend=backend,
    )
    torques = model.run({"feature_a": 0.0, "feature_b": 0.0})
    assert pytest.approx(torques["knee_flexion_moment_ipsi_Nm"]) == pytest.approx(0.5 * 70.0 * 0.4)
    assert pytest.approx(torques["ankle_dorsiflexion_moment_ipsi_Nm"]) == pytest.approx(10.0 * 0.4)


def test_bundle_torque_model_falls_back_to_torque_index_keys(tmp_path):
    bundle_dir = _write_bundle(tmp_path)
    backend = DummyBackend(outputs={"torque_0": 1.0, "torque_1": 2.0}, features=("feature_a", "feature_b"))
    model = BundleTorqueModel(
        bundle_dir,
        output_map={"knee": "knee_flexion_moment_ipsi_Nm_kg"},
        subject_mass_kg=60.0,
        assistance_fraction=1.0,
        backend=backend,
    )
    torques = model.run({"feature_a": 0.0, "feature_b": 0.0})
    assert pytest.approx(torques["knee"]) == pytest.approx(1.0 * 60.0)


def test_bundle_torque_model_requires_mass_for_normalised_outputs(tmp_path):
    bundle_dir = _write_bundle(tmp_path)
    backend = DummyBackend(outputs={"knee_flexion_moment_ipsi_Nm_kg": 0.5}, features=("feature_a", "feature_b"))
    with pytest.raises(ValueError, match="subject_mass_kg"):
        BundleTorqueModel(
            bundle_dir,
            output_map={"knee": "knee_flexion_moment_ipsi_Nm_kg"},
            assistance_fraction=1.0,
            backend=backend,
        )
