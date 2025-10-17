"""Feed-forward torque controller built around configurable torque models."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from rpc_runtime.runtime.wrangler import InputSchema

from ..actuators.base import TorqueCommand
from .base import ControllerFault
from .torque_models.base import TorqueModel


@dataclass(slots=True)
class PIControllerConfig:
    """Configuration block describing controller timing and limits."""

    dt: float
    torque_scale: float
    torque_limit_nm: float
    joints: tuple[str, ...]


class PIController:
    """Feed-forward controller that scales torque-model outputs."""

    def __init__(
        self,
        config: PIControllerConfig,
        torque_model: TorqueModel,
        input_schema: InputSchema | None = None,
    ) -> None:
        """Initialise controller state.

        Args:
            config: Control loop timing, limits, and joint ordering.
            torque_model: Feed-forward torque provider.
            input_schema: Optional input schema describing canonical controller
                features. When provided, the runtime constructs a
                `DataWrangler` that materialises features in this order every
                tick.
        """
        self._config = config
        self._torque_model = torque_model
        self._input_schema = input_schema

    def reset(self) -> None:
        """Reset any internal state (no-op for feed-forward controller)."""
        return None

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
            feature_map = features.as_dict()  # type: ignore[assignment]
        else:
            feature_map = dict(features)
        try:
            torque_ff = self._torque_model.run(dict(feature_map))
        except Exception as exc:  # pragma: no cover - defensive guard
            raise ControllerFault(
                f"{self.__class__.__name__} failed to compute torques: {exc}"
            ) from exc
        torques: dict[str, float] = {}
        limit = float(self._config.torque_limit_nm)
        apply_limit = limit > 0
        for joint in self._config.joints:
            feedforward = float(torque_ff.get(joint, 0.0))
            torque = feedforward * self._config.torque_scale
            if apply_limit:
                torque = max(min(torque, limit), -limit)
            torques[joint] = torque
        return TorqueCommand(timestamp=timestamp, torques_nm=torques)

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Return the ordered joint names governed by this controller."""
        return tuple(self._config.joints)

    # tick() removed in favor of compute_torque(features, timestamp)

    # _build_features removed in favor of schema-driven DataWrangler
