"""Torque model loaders and runners."""

from __future__ import annotations

from .base import TorqueModel
from .bundle import BundleTorqueModel
from .joblib import JoblibTorqueModel
from .mock import MockTorqueModel

__all__ = [
    "TorqueModel",
    "MockTorqueModel",
    "BundleTorqueModel",
    "JoblibTorqueModel",
    "ONNXTorqueModel",
    "TorchscriptTorqueModel",
]


def __getattr__(name: str):
    if name == "ONNXTorqueModel":
        from .onnx_runtime import ONNXTorqueModel as _ONNXTorqueModel

        return _ONNXTorqueModel
    if name == "TorchscriptTorqueModel":
        from .torchscript import TorchscriptTorqueModel as _TorchscriptTorqueModel

        return _TorchscriptTorqueModel
    raise AttributeError(name)
