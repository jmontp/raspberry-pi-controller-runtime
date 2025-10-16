Torque Bundles
==============

This folder stores deployable torque-model bundles that the runtime can load
directly on the Raspberry Pi. Each bundle is exported from the
`torque-modeling` project and checked into version control so the Pi only needs
to `git pull` to pick up new controllers.

Layout
------

```
models/torque_bundles/
  <run_id>/
    metadata.json
    preprocessing.json
    torque.onnx | energy_shaping.ts | ...
```

- `metadata.json` – enriched export metadata (see `docs/torque_models.md`).
- `preprocessing.json` – ordered feature names and scaler statistics.
- Model artifact (`*.onnx` or `*.ts`) – architecture-specific payload.

Workflow
--------

1. Train a model on the workstation via the `torque-modeling` repo.
2. Export the chosen run with
   `python -m torque_modeling.export --run <run_dir> --format <fmt>`.
3. Use the `--sync-runtime` flag to mirror the exported folder into this
   directory, e.g.
   `python -m torque_modeling.export --run <run_dir> --format torchscript --sync-runtime ../raspberry-pi-controller-runtime/models/torque_bundles`.
4. Commit the new/updated bundle alongside the runtime profile changes that
   reference it.
5. On the Pi, pull the repo and restart the runtime loop.

Temporary exports or large analysis artifacts should **not** be placed here;
only the minimal runtime bundle is versioned. If bundles become too large we
can migrate this directory to Git LFS in the future.
