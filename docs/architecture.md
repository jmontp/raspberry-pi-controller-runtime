# Architecture Overview

The runtime splits responsibilities into four layers:

1. **Sensors** — `rpc_runtime.sensors` exposes hardware‑agnostic modality bases (e.g., IMU/GRF) and hardware drivers. Adapters translate vendor payloads into standardized samples; modality bases aggregate multiple drivers and compute canonical signals.
2. **Controllers** — `rpc_runtime.controllers` implements the PI controller skeleton with replaceable torque models (`onnxruntime`, `torchscript`). Controllers declare required input/output signals.
3. **Actuators** — `rpc_runtime.actuators` provides safe torque application and fault reporting. Adapters validate joint membership and enforce optional torque limits.
4. **Pipelines** — `rpc_runtime.pipelines.RuntimeLoop` orchestrates the read→compute→actuate loop; `rpc_runtime.pipelines.scheduler` abstracts timing. A lightweight DataWrangler compiles requested signals into a model‑ready feature packet.

Configuration lives in a single `hardware_config.yaml`. Controllers declare the
canonical signals they consume and produce; the hardware config maps measured
signals to concrete drivers and establishes defaults. Derived signals are
computed in code by the modality pipeline (no config fields required).

Example `hardware_config.yaml`:

```yaml
defaults:
  dtype: float32
  fault_strategy: raise
  include_mask: false

drivers:
  thigh_imu:
    class: rpc_runtime.sensors.imu.microstrain_3dm_gx5.Microstrain3DMGX5IMU
    config:
      port_map:
        thigh: /dev/ttyIMU_thigh
      max_stale_samples: 3
  shank_imu:
    class: rpc_runtime.sensors.imu.mock.MockIMU
    config:
      loop: true
  fsr:
    class: rpc_runtime.sensors.grf.fsr.BluetoothFSR
    config:
      address: E8:EA:71:E8:37:D1

measured_signals:
  thigh_angle: thigh_imu
  thigh_rate: thigh_imu
  shank_angle: shank_imu
  shank_rate: shank_imu
  grf_total: fsr
```

Startup:
- Controller exposes `expected_inputs` and `expected_outputs` (ordered).
- Load `hardware_config.yaml`; instantiate drivers; build one SensorType per modality/scope.
- DataWrangler validates capability and hardware, compiles a plan, and allocates
  a reusable feature buffer (dtype from defaults).
- Runtime loop calls `wrangler.get_packet` each tick and forwards the feature
  vector to the controller; torques route to actuators via the runtime loop.

## High‑level flow

```mermaid
flowchart TD
    start((Start)) --> I0[Load hardware config]
    subgraph Init [Initialize]
        direction TB
        I0 --> I1[Initialize controller/model]
        I1 --> I2[Get expected inputs / outputs]
        I2 --> I3[Initialize DataWrangler]
        I3 --> I4[Initialize actuators]
    end
    style Init fill:#fffbe6,stroke:#fadb14,stroke-width:2px

    subgraph RuntimeLoop [Runtime Loop]
        direction LR
        R1[Get feature packet from DataWrangler] --> R2[Run controller/model]
        R2 --> R3[Apply torques]
        R3 --> R1
    end
    I4 --> R1
    style RuntimeLoop fill:#f0f5ff,stroke:#2b6cb0,stroke-width:2px
```

DataWrangler performs configuration validation and planning once at
initialisation, so the runtime loop only asks it for feature packets each tick.

### DataWrangler initialisation sequence

```mermaid
sequenceDiagram
    participant Runtime as Runtime
    participant Controller as Controller
    participant Wrangler as DataWrangler
    participant Config as Hardware Config
    participant SensorType as Sensor Type
    participant Driver as Hardware Driver

    Runtime->>Controller: load controller
    Runtime->>Wrangler: init(controller, config_path)
    Wrangler->>Controller: expected inputs / outputs
    Wrangler->>Config: load hardware_config.yaml
    Config-->>Wrangler: driver specs and measured signals

    loop per sensor type
        Wrangler->>SensorType: build with assigned drivers
        Wrangler->>SensorType: can_provide(required signals)?
        SensorType-->>Wrangler: supported / unsupported
        Wrangler->>SensorType: probe_hardware(required signals)
        SensorType->>Driver: instantiate and probe
        Driver-->>SensorType: readiness status
        SensorType-->>Wrangler: readiness per signal
    end

    Wrangler-->>Runtime: validation plan and feature buffer
```

### DataWrangler runtime sequence

```mermaid
sequenceDiagram
    participant Runtime as Runtime Loop
    participant Wrangler as DataWrangler
    participant SensorType as Sensor Type
    participant Driver as Hardware Driver
    participant Controller as Controller
    participant Actuator as Actuator

    Runtime->>Wrangler: get_packet
    Wrangler->>SensorType: fetch required signals
    SensorType->>Driver: read_native
    Driver-->>SensorType: measured values
    SensorType-->>Wrangler: canonical signals
    Wrangler-->>Controller: feature vector
    Controller-->>Actuator: torque command
    Actuator-->>Runtime: apply complete
```

### DataWrangler relationships

```mermaid
classDiagram
%% Legend: [A] abstract  [I] implemented

class DataWrangler {
  +required_inputs
  +feature_order
  +feature_buffer
  +sensor_types
  +validate_plan() [I]
  +get_packet() [I]
}
class Controller {
  +expected_inputs()[I]
  +expected_outputs()[I]
}
class SensorTypeBase {
  +provided_signals()[I]
  +can_provide(signals)[I]
  +probe_hardware(signals)[I]
  +fetch(signals)[I]
}
class SensorHardwareDriver {
  +start() [A]
  +stop() [A]
  +probe_hardware(requirements)[A]
  +read_native()[A]
}
class Actuator {
  +start() [A]
  +stop() [A]
  +apply(command)[I]
  +fault_if_needed()[I]
}

DataWrangler --> Controller : queries requirements
DataWrangler *-- SensorTypeBase : owns
SensorTypeBase *-- SensorHardwareDriver : owns
Controller --> Actuator : sends torque command
```

## Control loop sequence

```mermaid
sequenceDiagram
    participant App as App Launcher
    participant Runtime as Runtime Loop
    participant Profile as Profile Schemas
    participant Registry as Canonical Registry
    participant Wrangler as DataWrangler
    participant SensorA as Sensor Type A
    participant DriverA as Hardware Driver A
    participant SensorB as Sensor Type B
    participant DriverB as Hardware Driver B
    participant Ctl as Controller
    participant Model as Torque Model
    participant Act as Actuator

    App->>Runtime: Load controller and hardware config
    Runtime->>Ctl: expected_inputs / expected_outputs
    Runtime->>Wrangler: init(required_inputs, hardware config)

    Note over Wrangler: validate and compile plan (capability + hardware)
    Wrangler->>Registry: validate signal names
    Wrangler->>SensorA: can_provide and probe_hardware
    Wrangler->>SensorB: can_provide and probe_hardware (optional)
    Wrangler-->>Runtime: OK or error

    Runtime->>SensorA: start
    SensorA->>DriverA: open/connect
    Runtime->>SensorB: start (optional)
    SensorB->>DriverB: open/connect (optional)
    Runtime->>Act: start
    Runtime->>Ctl: reset

    loop Each tick
        Runtime->>Wrangler: get_packet
        Wrangler->>SensorA: fetch subset
        SensorA->>DriverA: read_native
        DriverA-->>SensorA: values
        Wrangler->>SensorB: fetch subset (optional)
        SensorB->>DriverB: read_native
        DriverB-->>SensorB: values
        Wrangler-->>Runtime: features (ordered)

        Runtime->>Ctl: tick(features)
        Ctl->>Model: run(features)
        Model-->>Ctl: feedforward torques
        Ctl-->>Runtime: TorqueCommand
        Runtime->>Act: apply(command)
        Runtime->>Act: fault_if_needed
    end

    Runtime->>SensorA: stop
    SensorA->>DriverA: close
    Runtime->>SensorB: stop (optional)
    SensorB->>DriverB: close (optional)
    Runtime->>Act: stop
```

<!-- Legacy interface overview removed in favor of DataWrangler-centric diagrams. -->

## Measured vs. derived signals

Some canonical signals are measured directly by drivers (e.g., segment angles);
others are derived by the sensor type from prerequisites (e.g., joint angle
derived as the difference of two segment angles). The sensor type uses a small
SignalPipeline to compute derived values across one or more drivers.

```mermaid
classDiagram
class SignalResolver {
      +requires
      +compute(values)
}
class SignalPipeline {
      +resolvers
      +dependency_graph
      +resolve(requested, providers)
}
    class SensorTypeBase {
      +pipeline
      read() [I]
    }
class SensorHardwareDriver {
      read_native() [A]
      provided_measurements()
}

    SensorTypeBase *-- SignalPipeline
    SignalPipeline ..> SignalResolver : uses
    SignalPipeline ..> SensorHardwareDriver : pulls measured prereqs
```

### Per‑tick resolution flow

```mermaid
sequenceDiagram
    participant ST as SignalPipeline (in SensorTypeBase)
    participant D1 as Driver A
    participant D2 as Driver B

    Note over ST: Determine closure of dependencies for requested signals
    ST->>D1: fetch measured prerequisites (subset)
    D1-->>ST: measurement values
    ST->>D2: fetch measured prerequisites (subset)
    D2-->>ST: measurement values
    ST->>ST: compute derived signals via resolvers (topological order)
    ST-->>Runtime: canonical signal map / normalized sample
```
