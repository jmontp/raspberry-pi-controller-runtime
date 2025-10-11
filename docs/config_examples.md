# Configuration Examples

The runtime consumes sensor and actuator configuration objects that can be
constructed directly in Python or loaded from YAML. The following snippets show
how to wire common options.

```yaml
sensors:
  imu:
    class: rpc_runtime.sensors.imu.mock.MockIMU
    config:
      joint_names: ["knee", "ankle"]
      segment_names: ["thigh", "shank", "foot"]
      max_stale_samples: 3
      max_stale_time_s: 0.1
      fault_strategy: warn
  vertical_grf:
    class: rpc_runtime.sensors.grf.mock.MockVerticalGRF
    config:
      channel_names: ["heel", "toe"]
      max_stale_samples: 5
      fault_strategy: fallback
actuators:
  class: rpc_runtime.actuators.osl_actuator.OSLActuator
  config:
    controller_hz: 500
    joints:
      - name: knee
        gear_ratio: 9.0
        port: /dev/ttyActPackRightKnee
      - name: ankle
        gear_ratio: 9.0
        port: /dev/ttyActPackRightAnkle
  safety:
    torque_limits_nm:
      knee: 40.0
      ankle: 35.0
    clamp_torque: true
```

Sample code to load and apply the configuration:

```python
from pathlib import Path
import yaml

from rpc_runtime.actuators.osl_actuator import OSLActuator, OSLLegConfig, OSLJoint
from rpc_runtime.actuators.base import BaseActuatorConfig
from rpc_runtime.sensors.imu.base import BaseIMUConfig
from rpc_runtime.sensors.grf.base import BaseVerticalGRFConfig
from rpc_runtime.sensors.imu.mock import MockIMU
from rpc_runtime.sensors.grf.mock import MockVerticalGRF

config = yaml.safe_load(Path("config/runtime.yaml").read_text())

imu_cfg = BaseIMUConfig(**config["sensors"]["imu"]["config"])
grf_cfg = BaseVerticalGRFConfig(**config["sensors"]["vertical_grf"]["config"])
imu = MockIMU(config_override=imu_cfg)
vertical_grf = MockVerticalGRF(config_override=grf_cfg)

leg_config = OSLLegConfig(
    controller_hz=config["actuators"]["config"]["controller_hz"],
    joints=tuple(
        OSLJoint(**joint) for joint in config["actuators"]["config"]["joints"]
    ),
)
# If the actuator adapter exposes a config override (e.g., MockActuator), pass the
# normalised BaseActuatorConfig to enforce torque limits:
actuator_config = BaseActuatorConfig(
    joint_names=tuple(j["name"] for j in config["actuators"]["config"]["joints"]),
    torque_limits_nm=config["actuators"].get("safety", {}).get("torque_limits_nm"),
    clamp_torque=config["actuators"].get("safety", {}).get("clamp_torque", False),
)
```

Use these snippets as a starting point when crafting deployment-specific
configuration files.
