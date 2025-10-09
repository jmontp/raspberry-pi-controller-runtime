# Torque Model Integration

Torque models follow the export workflow defined in `torque-modeling/docs/raspberry_pi.md`.

1. Export the trained run to a portable bundle (`onnx` or `torchscript`).
2. Copy the exported folder to the Raspberry Pi.
3. Point `ONNXTorqueModel` or `TorchscriptTorqueModel` at the bundle path.

Both implementations validate feature ordering via `preprocessing.json` and reuse the saved scaler statistics. See the original document for installation steps and optimisation tips (quantisation, batching, thermal monitoring).
