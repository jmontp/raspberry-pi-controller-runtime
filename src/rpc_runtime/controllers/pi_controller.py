"""PI controller skeleton derived from `jose_pi_controller.py`."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from ..actuators.base import TorqueCommand
from ..sensors.combinators import ControlInputs
from .torque_models.base import TorqueModel


@dataclass(slots=True)
class PIControllerGains:
    """Per-joint proportional and integral gains."""

    kp: Dict[str, float]
    ki: Dict[str, float]


@dataclass(slots=True)
class PIControllerConfig:
    """Configuration block describing controller timing and limits."""

    dt: float
    torque_scale: float
    torque_limit_nm: float
    joints: tuple[str, ...]
    velocity_filter_alpha: float = 0.1
    torque_filter_alpha: float = 0.1


class LowPassFilter:
    """Single-pole low-pass filter preserving previous state between updates."""

    def __init__(self, alpha: float):
        """Create a new filter.

        Args:
            alpha: Weight applied to the new sample (0..1).
        """
        self._alpha = alpha
        self._state: float | None = None

    def reset(self) -> None:
        """Clear the filter state."""
        self._state = None

    def update(self, value: float) -> float:
        """Blend a new sample with the smoothed history.

        Args:
            value: New scalar measurement.

        Returns:
            float: The filtered output.
        """
        if self._state is None:
            self._state = value
        else:
            self._state = self._alpha * value + (1.0 - self._alpha) * self._state
        return self._state


class PIController:
    """Composable PI controller with torque model feedforward."""

    def __init__(
        self,
        config: PIControllerConfig,
        gains: PIControllerGains,
        torque_model: TorqueModel,
    ) -> None:
        """Initialise controller state and allocate per-joint filters.

        Args:
            config: Control loop timing, limits, and joint ordering.
            gains: Per-joint PI gains.
            torque_model: Feed-forward torque provider.
        """
        self._config = config
        self._torque_model = torque_model
        self._kp = gains.kp
        self._ki = gains.ki
        self._integral_error = dict.fromkeys(config.joints, 0.0)
        self._torque_filters = {
            joint: LowPassFilter(config.torque_filter_alpha) for joint in config.joints
        }

    def reset(self) -> None:
        """Zero integrator and filter state for all joints."""
        for joint in self._config.joints:
            self._integral_error[joint] = 0.0
            self._torque_filters[joint].reset()

    def tick(self, inputs: ControlInputs) -> TorqueCommand:
        """Advance the controller by one tick.

        Args:
            inputs: Current IMU and optional GRF measurements.

        Returns:
            TorqueCommand: Torque command annotated with the IMU timestamp.
        """
        features = self._build_features(inputs)
        torque_ff = self._torque_model.run(features)
        torques: Dict[str, float] = {}
        for joint in self._config.joints:
            ref = features.get(f"{joint}_desired_angle", 0.0)
            meas = features.get(f"{joint}_angle", 0.0)
            error = ref - meas
            integral = self._integral_error[joint] + error * self._config.dt
            self._integral_error[joint] = integral
            p = self._kp.get(joint, 0.0) * error
            i = self._ki.get(joint, 0.0) * integral
            feedforward = torque_ff.get(joint, 0.0)
            torque = (p + i + feedforward) * self._config.torque_scale
            torque = max(min(torque, self._config.torque_limit_nm), -self._config.torque_limit_nm)
            torques[joint] = self._torque_filters[joint].update(torque)
        return TorqueCommand(timestamp=inputs.imu.timestamp, torques_nm=torques)

    def _build_features(self, inputs: ControlInputs) -> Dict[str, float]:
        """Compose the feature dictionary expected by the torque model.

        Args:
            inputs: IMU sample and optional GRF measurements.

        Returns:
            Dict[str, float]: Feature values keyed by feature name.
        """
        imu = inputs.imu
        features: Dict[str, float] = {}
        for idx, joint in enumerate(self._config.joints):
            try:
                angle = imu.joint_angles_rad[idx]
                velocity = imu.joint_velocities_rad_s[idx]
            except IndexError:
                angle = 0.0
                velocity = 0.0
            features[f"{joint}_angle"] = angle
            features[f"{joint}_velocity"] = velocity
            features.setdefault(f"{joint}_desired_angle", angle)
            features.setdefault(f"{joint}_desired_velocity", velocity)
        if inputs.vertical_grf is not None:
            for i, value in enumerate(inputs.vertical_grf.forces_newton):
                features[f"grf_{i}"] = value
        return features
