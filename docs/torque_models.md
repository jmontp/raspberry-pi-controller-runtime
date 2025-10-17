# Torque Model Integration

Torque models follow the export workflow defined in `torque-modeling/docs/raspberry_pi.md`
with two additional steps to keep deployable bundles in this repository.

1. Export the trained run to a portable bundle (`onnx` or `torchscript`).
2. Copy or sync the exported folder into `models/torque_bundles/<run_id>/`.
3. Reference the bundle from the controller configuration (see `docs/config_profiles.md`).
4. Commit both the bundle and profile change; deploy by pulling this repo onto the Pi.

Bundles contain `metadata.json`, `preprocessing.json`, and the architecture-specific
artifact (`*.onnx`, `*.ts`, ...). Runtime loaders validate feature ordering via
`preprocessing.json` and reuse the saved scaler statistics. See the torque-modeling
document for installation steps and optimisation tips (quantisation, batching,
thermal monitoring).

## Runtime configuration

The runtime uses `BundleTorqueModel` to read the bundle metadata, convert Nm/kg
torques to Nm, and apply an assistance fraction before the feed-forward controller
scales the command. Configure the controller profile like so:

```yaml
controllers:
  pi_bundle:
    implementation: rpc_runtime.controllers.pi_controller.PIController
    joints:
      - knee_flexion_moment_ipsi_Nm
      - ankle_dorsiflexion_moment_ipsi_Nm
    config:
      dt: 0.01
      torque_scale: 1.0
      torque_limit_nm: 30.0
    torque_model:
      implementation: rpc_runtime.controllers.torque_models.bundle.BundleTorqueModel
      config:
        bundle_path: models/torque_bundles/demo123
        output_map:
          knee_flexion_moment_ipsi_Nm: knee_flexion_moment_ipsi_Nm_kg
          ankle_dorsiflexion_moment_ipsi_Nm: ankle_dorsiflexion_moment_ipsi_Nm
        subject_mass_kg: 72.5
        assistance_fraction: 0.35
```

- `bundle_path` points at the version-controlled folder in `models/torque_bundles/`.
- `output_map` maps runtime joint names to the exported dataset torque columns.
- `subject_mass_kg` converts Nm/kg predictions to Nm; it is required whenever the
  bundle indicates `torque_normalised_by_mass`.
- `assistance_fraction` applies lab-level down-scaling for actuator limits.

Additional heads (phase, task, etc.) remain in the bundle metadata for future
controllers but are ignored by the current feed-forward implementation.
