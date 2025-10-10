"""Bluetooth FSR vertical GRF adapter."""

from __future__ import annotations

import time
from dataclasses import dataclass

from .base import BaseVerticalGRF, BaseVerticalGRFConfig, VerticalGRFSample

try:  # pragma: no cover - hardware dependency
    from MBLUE.device_side.FSR_BT_thread import FSRThread
except ImportError:  # noqa: F401 pragma: no cover - optional
    FSRThread = None


@dataclass(slots=True)
class BluetoothFSRConfig(BaseVerticalGRFConfig):
    """Configuration options for the Bluetooth FSR adapter."""

    channel_names: tuple[str, ...] = (
        "hallux",
        "toes",
        "met1",
        "met3",
        "met5",
        "arch",
        "heel_medial",
        "heel_lateral",
    )
    address: str = ""
    sampling_rate: str = "100Hz"
    max_history: int = 500
    channel_scale_newton: float = 1.0


class BluetoothFSR(BaseVerticalGRF):
    """Wrap the legacy Bluetooth FSR thread with the abstract interface."""

    CHANNEL_NAMES = BluetoothFSRConfig.channel_names

    def __init__(self, config: BluetoothFSRConfig | None = None):
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
        cfg = config or BluetoothFSRConfig()
        super().__init__(cfg)
        self._thread = FSRThread(
            address=cfg.address,
            max_history=cfg.max_history,
            sampling_rate=cfg.sampling_rate,
        )
        self._baseline = [0.0] * len(self.channel_names)
        self._last_timestamp: float | None = None

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
                return self._handle_sample(None, fresh=False)
            last = self._thread.fsr_history[-1]
        timestamp = float(last.get("timestamp", time.monotonic()))
        fresh = self._last_timestamp is None or timestamp != self._last_timestamp
        self._last_timestamp = timestamp
        forces = tuple(
            (float(last["fsr"][name]) - base) * self.config.channel_scale_newton
            for name, base in zip(self.channel_names, self._baseline, strict=False)
        )
        sample = VerticalGRFSample(timestamp=timestamp, forces_newton=forces)
        return self._handle_sample(sample, fresh=fresh)

    def zero(self) -> None:
        """Capture the current sample as the baseline offset."""
        with self._thread.data_lock:
            if not self._thread.fsr_history:
                return
            last = self._thread.fsr_history[-1]
            self._baseline = [float(last["fsr"][name]) for name in self.channel_names]
