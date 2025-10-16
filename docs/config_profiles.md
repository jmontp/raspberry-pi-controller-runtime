# Controller Profiles

Runtime profiles live in YAML files (e.g. `scripts/mock_hardware_config.yaml`) and
describe sensors, actuators, and controller wiring. The `controllers.<name>`
section now supports torque bundles exported from the `torque-modeling` project.

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
      gains: {...}
    torque_model:
      implementation: rpc_runtime.controllers.torque_models.bundle.BundleTorqueModel
      config:
        bundle_path: models/torque_bundles/demo123
        output_map:
          knee_flexion_moment_ipsi_Nm: knee_flexion_moment_ipsi_Nm_kg
          ankle_dorsiflexion_moment_ipsi_Nm: ankle_dorsiflexion_moment_ipsi_Nm
        subject_mass_kg: 72.5       # required when bundle outputs Nm/kg
        assistance_fraction: 0.35   # scales feedforward torque after mass conversion
```

Key fields:

- `bundle_path`: relative or absolute path to the exported bundle inside this repo.
- `output_map`: maps runtime joint names to dataset torque columns found in the bundle.
- `subject_mass_kg`: participant mass for Nm/kg → Nm conversion.
- `assistance_fraction`: lab-level scaling factor applied to all torque outputs.

All other controller configuration remains unchanged; gains, torque limits, and
filter parameters still live under `controllers.<name>.config`. Replace the
previous mock torque model with the bundle implementation when ready to deploy
trained models on the Raspberry Pi.
