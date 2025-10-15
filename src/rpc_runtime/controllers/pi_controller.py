"""PI controller skeleton derived from `jose_pi_controller.py`."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping

from rpc_runtime.config.models import InputSchema

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
        input_schema: InputSchema | None = None,
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
        self._input_schema = input_schema
        self._integral_error = dict.fromkeys(config.joints, 0.0)
        self._torque_filters = {
            joint: LowPassFilter(config.torque_filter_alpha) for joint in config.joints
        }

    def reset(self) -> None:
        """Zero integrator and filter state for all joints."""
        for joint in self._config.joints:
            self._integral_error[joint] = 0.0
            self._torque_filters[joint].reset()

    def compute_torque(self, features: Mapping[str, float], *, timestamp: float) -> TorqueCommand:
        """Compute joint torques from canonical features.

        Args:
            features: Mapping of canonical feature name to value.
            timestamp: Monotonic time for the resulting torque command.

        Returns:
            TorqueCommand: Safe torque command for the configured joints.
        """
        # Accept either a plain mapping or our FeatureView wrapper.
        feature_map: Mapping[str, float]
        if hasattr(features, "as_dict"):
            feature_map = getattr(features, "as_dict")()
        else:
            feature_map = dict(features)
        torque_ff = self._torque_model.run(dict(feature_map))
        torques: Dict[str, float] = {}
        for joint in self._config.joints:
            ref = features.get(f"{joint}_desired_angle", features.get(f"{joint}_angle", 0.0))
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
        return TorqueCommand(timestamp=timestamp, torques_nm=torques)

    # Backwards compatibility wrapper retained temporarily; raise to fail fast
    def tick(self, inputs: ControlInputs) -> TorqueCommand:  # pragma: no cover - transitional
        raise NotImplementedError(
            "PIController.tick is deprecated. Use compute_torque(features, timestamp)."
        )

    # _build_features removed in favor of schema-driven DataWrangler
