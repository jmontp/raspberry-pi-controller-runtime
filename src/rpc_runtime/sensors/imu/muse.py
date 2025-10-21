"""Muse Bluetooth IMU adapter built on top of the muse_api BLE helpers."""

from __future__ import annotations

import asyncio
import logging
import math
import sys
import queue
import threading
import time
from pathlib import Path
from dataclasses import dataclass, field, replace
from typing import Any, Dict, Mapping, cast

from .base import (
    LOGGER,
    BaseIMU,
    BaseIMUConfig,
    CanonicalFeatureSemantics,
    IMUSample,
)

try:  # pragma: no cover - optional dependency
    from bleak import BleakClient, BleakScanner
except ImportError:  # pragma: no cover - handled at runtime
    BleakClient = None  # type: ignore[assignment]
    BleakScanner = None  # type: ignore[assignment]

_MUSE_API_HINTS = (
    Path(__file__).resolve().parents[3] / "muse_api",
    Path(__file__).resolve().parents[3] / "third_party" / "muse_api",
)
for _hint in _MUSE_API_HINTS:
    marker = _hint / "Muse_HW.py"
    if marker.exists() and str(_hint) not in sys.path:
        sys.path.append(str(_hint))
        break

try:  # pragma: no cover - optional dependency
    import Muse_HW  # type: ignore
    import Muse_Utils  # type: ignore
    from Muse_Data import CommandResponse  # type: ignore
except ImportError:  # pragma: no cover - handled at runtime
    MH = None
    MU = None
    CommandResponse = None  # type: ignore[assignment]
else:  # pragma: no cover - executed when muse_api is installed
    MH = Muse_HW.Muse_HW
    MU = Muse_Utils.Muse_Utils


DEFAULT_COMMAND_UUID = "d5913036-2d8a-41ee-85b9-4e361aa5c8a7"
DEFAULT_DATA_UUID = "09bf2c52-d1d9-c0b7-4145-475964544307"
DEFAULT_STREAM_MODE = 0x00000033  # IMU (gyro+accel) + orientation + timestamp
DEFAULT_STREAM_FREQUENCY = 0x08  # 200 Hz
DEFAULT_HEADER_OFFSET = 8

STREAM_MODE_ALIASES: Mapping[str, int] = {
    "imu": 0x00000003,
    "imu_orientation": 0x00000013,
    "imu_orientation_timestamp": DEFAULT_STREAM_MODE,
    "imu_timestamp": 0x00000023,
}

STREAM_FREQUENCY_ALIASES: Mapping[str, int] = {
    "25hz": 0x01,
    "50hz": 0x02,
    "100hz": 0x04,
    "200hz": DEFAULT_STREAM_FREQUENCY,
    "400hz": 0x10,
    "800hz": 0x20,
    "1600hz": 0x40,
}


@dataclass(slots=True)
class MuseDeviceConfig:
    """Per-device configuration used by the Muse IMU adapter."""

    name: str | None = None
    address: str | None = None
    adapter: str | None = None
    command_uuid: str | None = None
    data_uuid: str | None = None
    stream_mode: int | str | None = None
    frequency: int | str | None = None
    discovery_timeout_s: float | None = None


@dataclass(slots=True)
class MuseIMUConfig(BaseIMUConfig):
    """Configuration describing a fleet of Muse IMUs streamed over BLE."""

    devices: Dict[str, MuseDeviceConfig] = field(default_factory=dict)
    command_uuid: str = DEFAULT_COMMAND_UUID
    data_uuid: str = DEFAULT_DATA_UUID
    stream_mode: int | str = DEFAULT_STREAM_MODE
    frequency: int | str = DEFAULT_STREAM_FREQUENCY
    queue_size: int = 64
    read_timeout_s: float = 0.0
    calibration_samples: int = 200
    calibration_interval_s: float = 0.002
    discovery_timeout_s: float = 5.0
    data_header_offset: int = DEFAULT_HEADER_OFFSET


@dataclass(slots=True)
class _FeatureBinding:
    device_key: str
    component: str  # "angle" or "velocity"
    sign: float


@dataclass(slots=True)
class _DeviceRuntime:
    key: str
    config: MuseDeviceConfig
    command_uuid: str
    data_uuid: str
    stream_mode: int
    frequency: int
    gyr_res: float = 0.0
    axl_res: float = 0.0
    mag_res: float = 0.0
    hdr_res: float = 0.0
    client: BleakClient | None = None


class MuseIMU(BaseIMU):
    """Adapter that streams canonical IMU data from Muse BLE devices."""

    def __init__(self, config: MuseIMUConfig | None = None, **overrides: Any) -> None:
        cfg = self._prepare_config(config, overrides)
        super().__init__(cfg)
        self._muse_config: MuseIMUConfig = cast(MuseIMUConfig, self.config)
        self._feature_bindings: Dict[str, _FeatureBinding] = self._build_feature_bindings()
        self._device_features: Dict[str, list[str]] = {}
        for feature, binding in self._feature_bindings.items():
            self._device_features.setdefault(binding.device_key, []).append(feature)
        self._queue: queue.Queue[tuple[float, Dict[str, float]]] = queue.Queue(
            maxsize=max(1, self._muse_config.queue_size)
        )
        self._last_values: Dict[str, float] = {}
        self._zero_offsets: Dict[str, float] = dict.fromkeys(self.port_map, 0.0)
        self._calibration_totals: Dict[str, float] = dict.fromkeys(self.port_map, 0.0)
        self._calibration_counts: Dict[str, int] = dict.fromkeys(self.port_map, 0)
        self._calibration_target = max(0, int(self._muse_config.calibration_samples))
        self._calibration_done = self._calibration_target == 0
        self._lock = threading.Lock()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_thread: threading.Thread | None = None
        self._devices: Dict[str, _DeviceRuntime] = self._build_device_runtimes()
        self._started = False

    # ------------------------------------------------------------------
    # BaseIMU interface
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open BLE connections, calibrate sensors, and start streaming."""
        if self._started:
            return
        self._ensure_dependencies_available()
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._run_loop, args=(self._loop,), name="MuseIMULoop", daemon=True
        )
        self._loop_thread.start()
        try:
            future = asyncio.run_coroutine_threadsafe(self._connect_all(), self._loop)
            future.result()
        except Exception:
            self._stop_loop()
            raise
        self._started = True

    def stop(self) -> None:
        """Stop streaming and release BLE resources."""
        if not self._started:
            return
        loop = self._loop
        if loop is not None:
            future = asyncio.run_coroutine_threadsafe(self._disconnect_all(), loop)
            future.result()
        self._stop_loop()
        self._started = False
        with self._lock:
            self._queue = queue.Queue(maxsize=max(1, self._muse_config.queue_size))
            self._last_values.clear()
            self._zero_offsets = dict.fromkeys(self.port_map, 0.0)
            self._calibration_totals = dict.fromkeys(self.port_map, 0.0)
            self._calibration_counts = dict.fromkeys(self.port_map, 0)
            self._calibration_done = self._calibration_target == 0

    def read(self) -> IMUSample:
        """Return the latest IMU sample, falling back to the last measurement if stale."""
        if not self._started:
            raise RuntimeError("MuseIMU not started; call start() before read().")
        try:
            timestamp, canonical = self._queue.get_nowait()
        except queue.Empty:
            if not self._last_values:
                return self._handle_sample(None, fresh=False)
            canonical = dict(self._last_values)
            timestamp = time.monotonic()
            sample = IMUSample(timestamp=timestamp, values=canonical)
            return self._handle_sample(sample, fresh=False)
        self._last_values = dict(canonical)
        sample = IMUSample(timestamp=timestamp, values=canonical)
        return self._handle_sample(sample, fresh=True)

    def reset(self) -> None:
        """Capture the current signed readings as the new zero offsets."""
        if not self._last_values:
            return
        with self._lock:
            for feature, value in self._last_values.items():
                self._zero_offsets[feature] = value
            self._calibration_done = True

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _prepare_config(
        self,
        config: MuseIMUConfig | None,
        overrides: Mapping[str, Any],
    ) -> MuseIMUConfig:
        if config is None:
            cfg = MuseIMUConfig(**overrides) if overrides else MuseIMUConfig()
        else:
            cfg = replace(config, **overrides) if overrides else config
        return cfg

    @classmethod
    def _validate_config(cls, config: BaseIMUConfig) -> BaseIMUConfig:
        if not isinstance(config, MuseIMUConfig):
            raise TypeError("MuseIMU requires a MuseIMUConfig instance.")
        cfg = cast(MuseIMUConfig, super()._validate_config(config))
        if not cfg.devices:
            raise ValueError("MuseIMUConfig.devices must map transport identifiers to device configs.")
        queue_size = int(cfg.queue_size)
        if queue_size <= 0:
            raise ValueError("MuseIMUConfig.queue_size must be positive.")
        cfg.queue_size = queue_size
        cfg.data_header_offset = max(0, int(cfg.data_header_offset))
        for key, device in cfg.devices.items():
            if not key or not key.strip():
                raise ValueError("Device mapping keys in MuseIMUConfig.devices must be non-empty strings.")
            if device.name is None and device.address is None:
                raise ValueError(
                    f"Muse device '{key}' must define either a BLE name or address."
                )
        missing_devices = sorted(
            {
                device_key
                for device_key in cfg.port_map.values()
                if device_key not in cfg.devices
            }
        )
        if missing_devices:
            joined = ", ".join(missing_devices)
            raise ValueError(
                f"Port map references unknown Muse devices: {joined}. "
                "Add entries to MuseIMUConfig.devices."
            )
        cfg.stream_mode = cls._resolve_stream_mode(cfg.stream_mode)
        cfg.frequency = cls._resolve_stream_frequency(cfg.frequency)
        for key, device in cfg.devices.items():
            stream_mode = device.stream_mode if device.stream_mode is not None else cfg.stream_mode
            device.stream_mode = cls._resolve_stream_mode(stream_mode)
            frequency = device.frequency if device.frequency is not None else cfg.frequency
            device.frequency = cls._resolve_stream_frequency(frequency)
        cfg.calibration_samples = max(0, int(cfg.calibration_samples))
        cfg.calibration_interval_s = max(0.0, float(cfg.calibration_interval_s))
        cfg.discovery_timeout_s = max(0.1, float(cfg.discovery_timeout_s or 5.0))
        cfg.read_timeout_s = max(0.0, float(cfg.read_timeout_s))
        return cfg

    @staticmethod
    def _resolve_stream_mode(value: int | str | None) -> int:
        if value is None:
            return DEFAULT_STREAM_MODE
        if isinstance(value, int):
            return value
        alias = value.strip().lower()
        resolved = STREAM_MODE_ALIASES.get(alias)
        if resolved is None:
            raise ValueError(f"Unknown Muse stream mode alias '{value}'.")
        return resolved

    @staticmethod
    def _resolve_stream_frequency(value: int | str | None) -> int:
        if value is None:
            return DEFAULT_STREAM_FREQUENCY
        if isinstance(value, int):
            return value
        alias = value.strip().lower()
        resolved = STREAM_FREQUENCY_ALIASES.get(alias)
        if resolved is None:
            raise ValueError(f"Unknown Muse stream frequency alias '{value}'.")
        return resolved

    def _build_feature_bindings(self) -> Dict[str, _FeatureBinding]:
        bindings: Dict[str, _FeatureBinding] = {}
        for feature, device_key in self.port_map.items():
            semantics = self.feature_semantics(feature)
            component = self._determine_component(semantics, feature)
            sign = self._determine_sign(semantics)
            bindings[feature] = _FeatureBinding(device_key=device_key, component=component, sign=sign)
        return bindings

    @staticmethod
    def _determine_component(semantics: CanonicalFeatureSemantics, feature: str) -> str:
        if semantics.quantity == "angle" and semantics.derivative == 0:
            return "angle"
        if semantics.quantity == "velocity" and semantics.derivative == 1:
            return "velocity"
        raise ValueError(
            f"MuseIMU currently supports sagittal angles and velocities; '{feature}' "
            "does not map to a supported quantity."
        )

    @staticmethod
    def _determine_sign(semantics: CanonicalFeatureSemantics) -> float:
        return -1.0 if semantics.side == "contra" else 1.0

    def _build_device_runtimes(self) -> Dict[str, _DeviceRuntime]:
        runtimes: Dict[str, _DeviceRuntime] = {}
        for key, device_cfg in self._muse_config.devices.items():
            command_uuid = device_cfg.command_uuid or self._muse_config.command_uuid
            data_uuid = device_cfg.data_uuid or self._muse_config.data_uuid
            stream_mode = int(device_cfg.stream_mode or self._muse_config.stream_mode)
            frequency = int(device_cfg.frequency or self._muse_config.frequency)
            runtimes[key] = _DeviceRuntime(
                key=key,
                config=device_cfg,
                command_uuid=command_uuid,
                data_uuid=data_uuid,
                stream_mode=stream_mode,
                frequency=frequency,
            )
        return runtimes

    def _ensure_dependencies_available(self) -> None:
        if BleakClient is None or BleakScanner is None:
            raise RuntimeError(
                "bleak is required to use MuseIMU; install it via 'pip install bleak'."
            )
        if MH is None or MU is None or CommandResponse is None:
            raise RuntimeError(
                "Muse API modules (Muse_HW.py, Muse_Utils.py, Muse_Data.py) are required. "
                "Ensure the muse_api package is installed or available on PYTHONPATH."
            )

    # ------------------------------------------------------------------
    # Async device lifecycle
    # ------------------------------------------------------------------

    @staticmethod
    def _run_loop(loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def _stop_loop(self) -> None:
        loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(loop.stop)
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=2.0)
        self._loop = None
        self._loop_thread = None

    async def _connect_all(self) -> None:
        await asyncio.gather(*(self._connect_device(runtime) for runtime in self._devices.values()))

    async def _disconnect_all(self) -> None:
        await asyncio.gather(*(self._disconnect_device(runtime) for runtime in self._devices.values()))

    async def _connect_device(self, runtime: _DeviceRuntime) -> None:
        device_cfg = runtime.config
        identifier = device_cfg.address
        if identifier is None:
            identifier = await self._discover_device(device_cfg)
        if identifier is None:
            raise RuntimeError(
                f"Unable to discover Muse device named '{device_cfg.name}'. "
                "Check that it is powered on and advertising."
            )
        client = BleakClient(identifier, adapter=device_cfg.adapter)
        await client.connect()
        runtime.client = client
        await self._prime_device(runtime)
        await client.start_notify(runtime.data_uuid, self._notification_handler(runtime))
        LOGGER.info("Connected to Muse device '%s' (%s)", runtime.key, identifier)

    async def _disconnect_device(self, runtime: _DeviceRuntime) -> None:
        client = runtime.client
        if client is None:
            return
        try:
            await client.write_gatt_char(runtime.command_uuid, MU.Cmd_StopAcquisition(), True)
        except Exception:  # pragma: no cover - best effort shutdown
            LOGGER.debug("Failed to send stop command to %s", runtime.key, exc_info=True)
        try:
            await client.stop_notify(runtime.data_uuid)
        except Exception:  # pragma: no cover - best effort shutdown
            LOGGER.debug("Failed to stop notifications for %s", runtime.key, exc_info=True)
        try:
            await client.disconnect()
        except Exception:  # pragma: no cover - best effort shutdown
            LOGGER.debug("Failed to disconnect Muse client %s", runtime.key, exc_info=True)
        runtime.client = None

    async def _discover_device(self, config: MuseDeviceConfig) -> str | None:
        assert config.name is not None
        timeout = config.discovery_timeout_s or self._muse_config.discovery_timeout_s
        devices = await BleakScanner.discover(timeout=timeout)
        for device in devices:
            if device.name == config.name:
                return device.address
        return None

    async def _prime_device(self, runtime: _DeviceRuntime) -> None:
        client = runtime.client
        if client is None:
            raise RuntimeError("BLE client missing while priming Muse device.")
        await client.write_gatt_char(runtime.command_uuid, MU.Cmd_GetSensorsFullScale(), True)
        response = await client.read_gatt_char(runtime.command_uuid)
        command_response = CommandResponse(bytearray(response))
        gyr_cfg, axl_cfg, mag_cfg, hdr_cfg = MU.Dec_SensorsFullScales(command_response)
        runtime.gyr_res = float(gyr_cfg.Sensitivity) * math.pi / 180.0
        runtime.axl_res = float(axl_cfg.Sensitivity)
        runtime.mag_res = float(mag_cfg.Sensitivity)
        runtime.hdr_res = float(hdr_cfg.Sensitivity)
        mode_enum = MH.DataMode(runtime.stream_mode)
        freq_enum = MH.DataFrequency(runtime.frequency)
        start_cmd = MU.Cmd_StartStream(mode_enum, freq_enum, enableDirect=True)
        await client.write_gatt_char(runtime.command_uuid, start_cmd, True)

    def _notification_handler(self, runtime: _DeviceRuntime):
        header_offset = self._muse_config.data_header_offset
        mode_value = runtime.stream_mode
        gyr_res = runtime.gyr_res or (float(MH.Gyroscope_CFG[0x01].Sensitivity) * math.pi / 180.0)
        axl_res = runtime.axl_res or float(MH.Accelerometer_CFG[0x08].Sensitivity)
        mag_res = runtime.mag_res or float(MH.Magnetometer_CFG[0x00].Sensitivity)
        hdr_res = runtime.hdr_res or float(MH.AccelerometerHDR_CFG[0x00].Sensitivity)

        def _handler(_: int, data: bytearray | bytes) -> None:
            payload = bytearray(data)
            if len(payload) <= header_offset:
                return
            try:
                sample = MU.DecodePacket(
                    payload[header_offset:],
                    0,
                    mode_value,
                    gyr_res / (math.pi / 180.0),
                    axl_res,
                    mag_res,
                    hdr_res,
                )
            except Exception:  # pragma: no cover - defensive guard
                LOGGER.warning("Failed to decode Muse packet from %s", runtime.key, exc_info=True)
                return
            if sample is None:
                return
            self._handle_muse_sample(runtime.key, sample)

        return _handler

    # ------------------------------------------------------------------
    # Data handling
    # ------------------------------------------------------------------

    def _handle_muse_sample(self, device_key: str, sample: Any) -> None:
        timestamp = time.monotonic()
        canonical: Dict[str, float] = {}
        features = self._device_features.get(device_key, [])
        if not features:
            return
        for feature in features:
            binding = self._feature_bindings[feature]
            raw_value = self._extract_measurement(sample, binding)
            if raw_value is None:
                continue
            signed = binding.sign * raw_value
            offset = self._update_calibration(feature, signed)
            canonical[feature] = signed - offset
        if not canonical:
            return
        with self._lock:
            if self._queue.full():
                try:
                    self._queue.get_nowait()
                except queue.Empty:  # pragma: no cover - unlikely
                    pass
            self._queue.put_nowait((timestamp, canonical))
            self._last_values.update(canonical)

    @staticmethod
    def _extract_measurement(sample: Any, binding: _FeatureBinding) -> float | None:
        if binding.component == "angle":
            euler = getattr(sample, "euler", None)
            if not euler or len(euler) < 2:
                return None
            return math.radians(float(euler[1]))
        if binding.component == "velocity":
            gyro = getattr(sample, "gyr", None)
            if not gyro or len(gyro) < 2:
                return None
            return math.radians(float(gyro[1]))
        return None

    def _update_calibration(self, feature: str, value: float) -> float:
        if self._calibration_done:
            return self._zero_offsets.get(feature, 0.0)
        total = self._calibration_totals.get(feature, 0.0) or 0.0
        count = self._calibration_counts.get(feature, 0) or 0
        if count < self._calibration_target:
            total += value
            count += 1
            self._calibration_totals[feature] = total
            self._calibration_counts[feature] = count
            if count == self._calibration_target:
                self._zero_offsets[feature] = total / count if count else 0.0
        remaining = [
            self._calibration_counts.get(name, 0) or 0 for name in self.port_map
        ]
        self._calibration_done = all(count >= self._calibration_target for count in remaining)
        return self._zero_offsets.get(feature, 0.0)
