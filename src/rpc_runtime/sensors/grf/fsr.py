"""GRF adapters for wired and Bluetooth force-sensing resistor arrays."""

from __future__ import annotations

import asyncio
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple, cast

from .base import BaseVerticalGRF, BaseVerticalGRFConfig, VerticalGRFSample

try:  # pragma: no cover - optional dependency
    from bleak import BleakClient  # type: ignore
except ImportError:  # pragma: no cover
    BleakClient = None  # type: ignore

try:  # pragma: no cover - optional dependency
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    serial = None  # type: ignore


DEFAULT_FSR_CHANNEL_NAMES: tuple[str, ...] = (
    "hallux",
    "toes",
    "met1",
    "met3",
    "met5",
    "arch",
    "heel_medial",
    "heel_lateral",
)


def _counts_to_force(count: int, scale: float, invert: bool) -> float:
    if invert:
        return (scale / count) if count else 0.0
    return count * scale


@dataclass(slots=True)
class BluetoothFSRConfig(BaseVerticalGRFConfig):
    """Configuration describing a Bluetooth FSR insole."""

    channel_names: tuple[str, ...] = DEFAULT_FSR_CHANNEL_NAMES
    address: str = ""
    sampling_rate_hz: int = 100
    max_history: int = 512
    connection_timeout_s: float = 10.0
    first_packet_timeout_s: float = 5.0
    channel_scale_newton: float = 1.0
    invert_counts: bool = False


class _BluetoothFSRWorker:
    """Async BLE worker that streams FSR packets in a background thread."""

    requires_bleak = True

    RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
    TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
    CMD_SET_SAMPLING = bytes([0x00, 0x01])
    CMD_MODE = bytes([0x00, 0x00])
    RATE_CODES = {25: 0x00, 100: 0x01, 200: 0x02}
    FSR_OFFSET = 52
    FSR_LENGTH = 16

    def __init__(
        self,
        address: str,
        sampling_rate_hz: int,
        max_history: int,
        connection_timeout_s: float,
        client_factory=None,
    ) -> None:
        if sampling_rate_hz not in self.RATE_CODES:
            raise ValueError(
                f"Unsupported sampling rate {sampling_rate_hz}Hz. "
                f"Allowed values: {sorted(self.RATE_CODES)}"
            )
        self._address = address
        self._rate_code = self.RATE_CODES[sampling_rate_hz]
        self._connection_timeout = connection_timeout_s
        self._client_factory = client_factory
        self._history: Deque[Tuple[float, Tuple[int, ...], Optional[int]]] = deque(
            maxlen=max_history
        )
        self._lock = threading.Lock()
        self._stop_requested = threading.Event()
        self.ready = threading.Event()
        self.first_packet = threading.Event()
        self.error: Optional[Exception] = None
        self.thread = threading.Thread(target=self._thread_entry, name="BluetoothFSR", daemon=True)
        self._client = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._last_battery: Optional[int] = None

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self._stop_requested.set()
        loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(lambda: None)
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)

    def get_latest(self) -> Optional[Tuple[float, Tuple[int, ...], Optional[int]]]:
        with self._lock:
            if not self._history:
                return None
            return self._history[-1]

    @property
    def last_battery_percent(self) -> Optional[int]:
        return self._last_battery

    def _thread_entry(self) -> None:
        try:
            asyncio.run(self._run())
        except Exception as exc:  # pragma: no cover - defensive guard
            self.error = exc
            self.ready.set()

    async def _run(self) -> None:
        self._loop = asyncio.get_running_loop()
        try:
            await self._connect_and_stream()
        except Exception as exc:
            self.error = exc
            self.ready.set()
        finally:
            await self._disconnect()

    async def _connect_and_stream(self) -> None:
        client_factory = self._client_factory or BleakClient
        if client_factory is None:  # pragma: no cover - handled by caller
            raise RuntimeError("bleak is required for Bluetooth FSR support")

        self._client = client_factory(self._address)
        await self._client.connect(timeout=self._connection_timeout)

        await self._client.write_gatt_char(
            self.RX_UUID, self.CMD_SET_SAMPLING + bytes([self._rate_code]), response=False
        )
        await asyncio.sleep(0.1)
        await self._client.write_gatt_char(self.RX_UUID, self.CMD_MODE + b"\x00", response=False)
        await asyncio.sleep(0.1)
        await self._client.start_notify(self.TX_UUID, self._handle_notification)
        self.ready.set()

        while not self._stop_requested.is_set():
            await asyncio.sleep(0.05)

    async def _disconnect(self) -> None:
        client = self._client
        if client is None:
            return
        try:
            if client.is_connected:
                try:
                    await client.write_gatt_char(self.RX_UUID, self.CMD_MODE + b"\x01", response=False)
                except Exception:  # pragma: no cover - best effort
                    pass
                try:
                    await client.stop_notify(self.TX_UUID)
                except Exception:  # pragma: no cover
                    pass
                await client.disconnect()
        finally:
            self._client = None

    def _handle_notification(self, _handle: int, data: bytearray | bytes) -> None:
        if len(data) < self.FSR_OFFSET + self.FSR_LENGTH:
            return
        fsr_counts = struct.unpack_from("<8H", data, self.FSR_OFFSET)
        battery = (data[1] >> 4) * 10 if len(data) > 1 else None
        timestamp = time.monotonic()
        with self._lock:
            self._history.append((timestamp, fsr_counts, battery))
            self._last_battery = battery
        if not self.first_packet.is_set():
            self.first_packet.set()


class BluetoothFSR(BaseVerticalGRF):
    """Adapter that streams Bluetooth FSR data using Bleak."""

    CHANNEL_NAMES = DEFAULT_FSR_CHANNEL_NAMES

    def __init__(
        self,
        config: BluetoothFSRConfig | None = None,
        *,
        worker: Optional[_BluetoothFSRWorker] = None,
    ) -> None:
        cfg = config or BluetoothFSRConfig()
        super().__init__(cfg)
        self._worker = worker
        self._baseline = [0.0] * len(self.channel_names)
        self._last_timestamp: Optional[float] = None
        self._channel_scale = float(cfg.channel_scale_newton)
        self._invert = bool(cfg.invert_counts)

    def start(self) -> None:
        cfg = self.config
        if self._worker is None:
            if BleakClient is None:
                raise RuntimeError("bleak must be installed to use BluetoothFSR.")
            self._worker = _BluetoothFSRWorker(
                address=cfg.address,
                sampling_rate_hz=cfg.sampling_rate_hz,
                max_history=cfg.max_history,
                connection_timeout_s=cfg.connection_timeout_s,
            )
        if getattr(self._worker, "requires_bleak", True) and BleakClient is None:
            raise RuntimeError("bleak must be installed to use BluetoothFSR.")
        self._worker.start()
        if not self._worker.ready.wait(timeout=cfg.connection_timeout_s):
            self._worker.stop()
            raise TimeoutError("Bluetooth FSR failed to initialise.")
        if self._worker.error is not None:
            error = self._worker.error
            self._worker.stop()
            raise RuntimeError(f"Bluetooth FSR initialisation failed: {error}") from error
        if not self._worker.first_packet.wait(timeout=cfg.first_packet_timeout_s):
            self._worker.stop()
            raise TimeoutError("Bluetooth FSR did not produce an initial packet.")

    def stop(self) -> None:
        if self._worker is None:
            return
        self._worker.stop()

    def read(self) -> VerticalGRFSample:
        if self._worker is None:
            raise RuntimeError("BluetoothFSR not started.")
        latest = self._worker.get_latest()
        if latest is None:
            return self._handle_sample(None, fresh=False)
        timestamp, counts, _battery = latest
        fresh = self._last_timestamp is None or timestamp != self._last_timestamp
        self._last_timestamp = timestamp
        forces = tuple(
            _counts_to_force(count, self._channel_scale, self._invert) - base
            for count, base in zip(counts, self._baseline, strict=False)
        )
        sample = VerticalGRFSample(timestamp=timestamp, forces_newton=forces)
        return self._handle_sample(sample, fresh=fresh)

    def zero(self) -> None:
        if self._worker is None:
            return
        latest = self._worker.get_latest()
        if latest is None:
            return
        _, counts, _ = latest
        self._baseline = [
            _counts_to_force(count, self._channel_scale, self._invert) for count in counts
        ]

    @property
    def last_battery_percent(self) -> Optional[int]:
        if self._worker is None:
            return None
        return self._worker.last_battery_percent

    @property
    def config(self) -> BluetoothFSRConfig:
        return cast(BluetoothFSRConfig, super().config)


@dataclass(slots=True)
class WiredFSRConfig(BaseVerticalGRFConfig):
    """Configuration for the wired (ActiSense) FSR insole."""

    channel_names: tuple[str, ...] = DEFAULT_FSR_CHANNEL_NAMES
    port: str = "/dev/ttyUSB0"
    baudrate: int = 230400
    clip_direction: str = "forward"
    max_history: int = 512
    channel_scale_newton: float = 1.0
    invert_counts: bool = True


class _WiredFSRWorker:
    """Threaded serial worker mirroring the ActiSense USB reader."""

    requires_serial = True

    FRAME_SIZE = 39
    START_COMMAND = bytes([0x6F])
    STOP_COMMAND = bytes([0x68])

    def __init__(
        self,
        port: str,
        baudrate: int,
        clip_direction: str,
        max_history: int,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._clip_direction = clip_direction
        self._history: Deque[Tuple[float, Tuple[int, ...], Optional[int]]] = deque(
            maxlen=max_history
        )
        self._lock = threading.Lock()
        self._stop_requested = threading.Event()
        self.ready = threading.Event()
        self.first_packet = threading.Event()
        self.error: Optional[Exception] = None
        self.thread = threading.Thread(target=self._run, name="WiredFSR", daemon=True)
        self._serial = None

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self._stop_requested.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)

    def get_latest(self) -> Optional[Tuple[float, Tuple[int, ...], Optional[int]]]:
        with self._lock:
            if not self._history:
                return None
            return self._history[-1]

    def _run(self) -> None:
        while not self._stop_requested.is_set():
            try:
                if self._serial is None:
                    if serial is None:  # pragma: no cover - handled by caller
                        raise RuntimeError("pyserial is required for wired FSR support")
                    self._serial = serial.Serial(
                        self._port,
                        baudrate=self._baudrate,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS,
                        timeout=0.1,
                    )
                    self._serial.write(self.START_COMMAND)
                    self.ready.set()
                frame = self._serial.read(self.FRAME_SIZE)
                if len(frame) != self.FRAME_SIZE:
                    continue
                counts = self._decode_frame(frame)
                timestamp = time.monotonic()
                with self._lock:
                    self._history.append((timestamp, counts, None))
                if not self.first_packet.is_set():
                    self.first_packet.set()
            except Exception as exc:
                self.error = exc
                self._close_serial()
                if not self.ready.is_set():
                    self.ready.set()
                time.sleep(0.2)
        self._close_serial()

    def _decode_frame(self, frame: bytes) -> Tuple[int, ...]:
        pairs_forward = (
            (5, 4),
            (7, 6),
            (9, 8),
            (11, 10),
            (13, 12),
            (15, 14),
            (17, 16),
            (19, 18),
        )
        pairs_reverse = (
            (11, 10),
            (9, 8),
            (7, 6),
            (5, 4),
            (17, 16),
            (19, 18),
            (13, 12),
            (15, 14),
        )
        pairs = pairs_forward if self._clip_direction != "reverse" else pairs_reverse

        counts = []
        for high_idx, low_idx in pairs:
            value = (frame[high_idx] << 8) + frame[low_idx]
            counts.append(value)
        return tuple(counts)

    def _close_serial(self) -> None:
        if self._serial is None:
            return
        try:
            self._serial.write(self.STOP_COMMAND)
        except Exception:  # pragma: no cover - best effort
            pass
        try:
            self._serial.close()
        except Exception:  # pragma: no cover
            pass
        self._serial = None


class WiredFSR(BaseVerticalGRF):
    """Adapter for wired ActiSense-style FSR insoles."""

    CHANNEL_NAMES = DEFAULT_FSR_CHANNEL_NAMES

    def __init__(
        self,
        config: WiredFSRConfig | None = None,
        *,
        worker: Optional[_WiredFSRWorker] = None,
    ) -> None:
        cfg = config or WiredFSRConfig()
        super().__init__(cfg)
        self._worker = worker
        self._baseline = [0.0] * len(self.channel_names)
        self._last_timestamp: Optional[float] = None
        self._channel_scale = float(cfg.channel_scale_newton)
        self._invert = bool(cfg.invert_counts)

    def start(self) -> None:
        cfg = self.config
        if self._worker is None:
            if serial is None:
                raise RuntimeError("pyserial must be installed to use WiredFSR.")
            self._worker = _WiredFSRWorker(
                port=cfg.port,
                baudrate=cfg.baudrate,
                clip_direction=cfg.clip_direction,
                max_history=cfg.max_history,
            )
        if getattr(self._worker, "requires_serial", True) and serial is None:
            raise RuntimeError("pyserial must be installed to use WiredFSR.")
        self._worker.start()
        if not self._worker.ready.wait(timeout=cfg.startup_timeout_s or 5.0):
            self._worker.stop()
            raise TimeoutError("Wired FSR failed to initialise.")
        if self._worker.error is not None:
            error = self._worker.error
            self._worker.stop()
            raise RuntimeError(f"Wired FSR initialisation failed: {error}") from error
        if not self._worker.first_packet.wait(timeout=cfg.startup_timeout_s or 5.0):
            self._worker.stop()
            raise TimeoutError("Wired FSR did not produce an initial packet.")

    def stop(self) -> None:
        if self._worker is None:
            return
        self._worker.stop()

    def read(self) -> VerticalGRFSample:
        if self._worker is None:
            raise RuntimeError("WiredFSR not started.")
        latest = self._worker.get_latest()
        if latest is None:
            return self._handle_sample(None, fresh=False)
        timestamp, counts, _ = latest
        fresh = self._last_timestamp is None or timestamp != self._last_timestamp
        self._last_timestamp = timestamp
        forces = tuple(
            _counts_to_force(count, self._channel_scale, self._invert) - base
            for count, base in zip(counts, self._baseline, strict=False)
        )
        sample = VerticalGRFSample(timestamp=timestamp, forces_newton=forces)
        return self._handle_sample(sample, fresh=fresh)

    def zero(self) -> None:
        if self._worker is None:
            return
        latest = self._worker.get_latest()
        if latest is None:
            return
        _, counts, _ = latest
        self._baseline = [
            _counts_to_force(count, self._channel_scale, self._invert) for count in counts
        ]

    @property
    def config(self) -> WiredFSRConfig:
        return cast(WiredFSRConfig, super().config)
