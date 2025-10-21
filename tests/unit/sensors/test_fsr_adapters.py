import math
import threading
import time
from collections import deque

import pytest

from rpc_runtime.sensors.grf.fsr import (
    BluetoothFSR,
    BluetoothFSRConfig,
    WiredFSR,
    WiredFSRConfig,
)


class _StubBluetoothWorker:
    requires_bleak = False

    def __init__(self) -> None:
        self.ready = threading.Event()
        self.first_packet = threading.Event()
        self.error = None
        self._lock = threading.Lock()
        self._history = deque()
        self.stopped = False

    def start(self) -> None:
        self.ready.set()
        self.append_sample(time.monotonic(), (100,) * 8, 80)

    def stop(self) -> None:
        self.stopped = True

    def append_sample(self, timestamp: float, counts, battery: int | None) -> None:
        with self._lock:
            self._history.append((timestamp, tuple(counts), battery))
        self.first_packet.set()

    def get_latest(self):
        with self._lock:
            if not self._history:
                return None
            return self._history[-1]

    @property
    def last_battery_percent(self):
        with self._lock:
            if not self._history:
                return None
            return self._history[-1][2]


class _StubWiredWorker:
    requires_serial = False

    def __init__(self, initial_count: int = 200) -> None:
        self.ready = threading.Event()
        self.first_packet = threading.Event()
        self.error = None
        self._lock = threading.Lock()
        self._history = deque()
        self.stopped = False
        self._initial_count = initial_count

    def start(self) -> None:
        self.ready.set()
        self.append_sample(time.monotonic(), (self._initial_count,) * 8)

    def stop(self) -> None:
        self.stopped = True

    def append_sample(self, timestamp: float, counts) -> None:
        with self._lock:
            self._history.append((timestamp, tuple(counts), None))
        self.first_packet.set()

    def get_latest(self):
        with self._lock:
            if not self._history:
                return None
            return self._history[-1]


def test_bluetooth_fsr_read_and_zero():
    worker = _StubBluetoothWorker()
    cfg = BluetoothFSRConfig(address="AA:BB:CC:DD:EE:FF", channel_scale_newton=0.5)
    fsr = BluetoothFSR(cfg, worker=worker)

    fsr.start()
    sample = fsr.read()
    assert fsr.last_battery_percent == 80
    assert sample.channels == len(cfg.channel_names)
    assert all(math.isclose(force, 50.0) for force in sample.forces_newton)

    fsr.zero()
    # Push a new packet with different counts
    worker.append_sample(time.monotonic(), (120,) * 8, 70)
    sample = fsr.read()
    assert fsr.last_battery_percent == 70
    assert all(math.isclose(force, 10.0) for force in sample.forces_newton)

    # Subsequent read without new data should be marked stale
    sample_stale = fsr.read()
    assert not fsr.last_sample_fresh
    assert sample_stale.forces_newton == sample.forces_newton

    fsr.stop()
    assert worker.stopped is True


def test_wired_fsr_inversion_and_zero():
    worker = _StubWiredWorker(initial_count=256)
    cfg = WiredFSRConfig(channel_scale_newton=1.0, invert_counts=True)
    fsr = WiredFSR(cfg, worker=worker)

    fsr.start()
    sample = fsr.read()
    # With invert=True and counts=256 the force should be 1/256 for each channel
    assert all(math.isclose(force, 1 / 256.0) for force in sample.forces_newton)

    fsr.zero()
    worker.append_sample(time.monotonic(), (512,) * 8)
    sample = fsr.read()
    expected = (1 / 512.0) - (1 / 256.0)
    assert all(math.isclose(force, expected) for force in sample.forces_newton)

    fsr.stop()
    assert worker.stopped is True


def test_bluetooth_requires_bleak_when_worker_missing(monkeypatch):
    from rpc_runtime.sensors.grf import fsr as fsr_module

    monkeypatch.setattr(fsr_module, "BleakClient", None)
    adapter = BluetoothFSR(BluetoothFSRConfig(address="AA:BB"), worker=None)
    with pytest.raises(RuntimeError):
        adapter.start()
