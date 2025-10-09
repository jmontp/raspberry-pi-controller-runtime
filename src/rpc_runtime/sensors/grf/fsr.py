"""Bluetooth FSR vertical GRF adapter."""

from __future__ import annotations

import time
from dataclasses import dataclass

from .base import BaseVerticalGRF, VerticalGRFSample

try:  # pragma: no cover - hardware dependency
    from MBLUE.device_side.FSR_BT_thread import FSRThread
except ImportError:  # noqa: F401 pragma: no cover - optional
    FSRThread = None


@dataclass(slots=True)
class BluetoothFSRConfig:
    """Configuration options for the Bluetooth FSR adapter."""

    address: str
    sampling_rate: str = "100Hz"
    max_history: int = 500
    channel_scale_newton: float = 1.0


class BluetoothFSR(BaseVerticalGRF):
    """Wrap the legacy Bluetooth FSR thread with the abstract interface."""

    def __init__(self, config: BluetoothFSRConfig):
        """Create the adapter.

        Args:
            config: Bluetooth address and scaling configuration.

        Raises:
            RuntimeError: If the FSR thread dependency is unavailable.
        """
        if FSRThread is None:
            raise RuntimeError(
                "MBLUE.device_side.FSR_BT_thread.FSRThread is unavailable. "
                "Ensure the dependency is on PYTHONPATH."
            )
        self._config = config
        self._thread = FSRThread(
            address=config.address,
            max_history=config.max_history,
            sampling_rate=config.sampling_rate,
        )
        self._baseline = [0.0] * 8

    def start(self) -> None:
        """Start the worker thread and wait for the first packet.

        Raises:
            TimeoutError: If the device does not initialise within 5 seconds.
        """
        self._thread.start()
        # Block until initial packet arrives or timeout
        start = time.monotonic()
        while not self._thread.initialized.wait(timeout=0.1):
            if time.monotonic() - start > 5.0:
                raise TimeoutError("FSR thread failed to initialize")

    def stop(self) -> None:
        """Signal the worker thread to stop and join."""
        self._thread.stopped = True
        if self._thread.thread.is_alive():  # pragma: no cover - hardware path
            self._thread.thread.join(timeout=1.0)

    def read(self) -> VerticalGRFSample:
        """Read the latest FSR sample from the history buffer."""
        with self._thread.data_lock:
            if not self._thread.fsr_history:
                raise RuntimeError("No FSR samples available yet")
            last = self._thread.fsr_history[-1]
        forces = tuple(
            (float(last["fsr"][name]) - base) * self._config.channel_scale_newton
            for name, base in zip(self._thread.FSR_NAMES, self._baseline, strict=False)
        )
        timestamp = float(last.get("timestamp", time.monotonic()))
        return VerticalGRFSample(timestamp=timestamp, forces_newton=forces)

    def zero(self) -> None:
        """Capture the current sample as the baseline offset."""
        with self._thread.data_lock:
            if not self._thread.fsr_history:
                return
            last = self._thread.fsr_history[-1]
            self._baseline = [float(last["fsr"][name]) for name in self._thread.FSR_NAMES]
