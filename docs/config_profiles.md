# Configuration Profiles

Profiles are lightweight YAML files that declare which sensors, actuators, and
controller bundle the runtime should start. Keep only three questions in mind:

- **Hardware bindings:** which adapter classes should run, and with what port or
  address configuration?
- **Controller schema:** which named channels feed the controller, and which
  hardware binding satisfies each channel?
- **Outputs:** which actuator receives each controller output?

## Minimal profile

```yaml
profile:
  name: right_leg_mock
  controller: pi_mock
  actuator: mock_leg
  sensors:
    - name: imu
      binding: imu_mock
    - name: grf
      binding: grf_mock

schemas:
  pi_schema:
    channels:
      - knee_angle
      - knee_velocity
      - ankle_angle
      - ankle_velocity
      - { signal: grf_total, required: false }

controllers:
  pi_mock:
    implementation: rpc_runtime.controllers.pi_controller.PIController
    input_schema: pi_schema
    joints: ["knee", "ankle"]
    config: {...}
    torque_model: {...}

sensors:
  imu_mock:
    driver: rpc_runtime.sensors.imu.mock.MockIMU
    provides:
      - knee_angle
      - knee_velocity
    config: {}
  grf_mock:
    driver: rpc_runtime.sensors.grf.mock.MockVerticalGRF
    provides:
      - grf_total
    config: {}
```

Optional channels can be marked with `required: false`; the controller will
fall back to the declared `default` value when the hardware is absent.

## Canonical signals

All controllers and hardware bindings reference a shared registry of canonical
signal names. The current catalogue includes:

| Signal | Description | Typical units |
|--------|-------------|---------------|
| `knee_angle` | Sagittal-plane knee angle from the IMU | radians |
| `knee_velocity` | Sagittal-plane knee angular velocity | rad/s |
| `ankle_angle` | Sagittal-plane ankle angle from the IMU | radians |
| `ankle_velocity` | Sagittal-plane ankle angular velocity | rad/s |
| `grf_total` | Total vertical GRF measurement | newtons |
| `knee_torque` | Controller torque command for the knee | newton-metres |
| `ankle_torque` | Controller torque command for the ankle | newton-metres |

When you introduce a new signal, add it to `CANONICAL_SIGNAL_REGISTRY` so it can
inherit defaults and documentation across profiles.

## Hardware naming

Use descriptive, lowercase identifiers that encode the device and placement, for
example:

- `microstrain_left_shank_imu` (provides the knee/ankle signals)
- `bluetooth_fsr_right_foot` (provides `grf_total`)
- `osl_leg_right` (consumes `knee_torque`/`ankle_torque`)

## Built-in adapters

- IMU: `rpc_runtime.sensors.imu.mock.MockIMU`, `...microstrain_3dm_gx5.Microstrain3DMGX5IMU`, `...simulated.SimulatedIMU`
- GRF: `rpc_runtime.sensors.grf.mock.MockVerticalGRF`, `...grf.fsr.BluetoothFSR`
- Actuator: `rpc_runtime.actuators.mock.MockActuator`, `rpc_runtime.actuators.osl_actuator.OSLActuator`

Swap hardware by pointing a binding at the desired driver string and updating
its `provides` list. The controller schema stays unchanged, keeping the profile
easy to reason about.

## Control loop sequence

```mermaid
sequenceDiagram
    participant App as App/Launcher
    participant Runtime as Runtime Loop
    participant Profile as Profile (schemas & bindings)
    participant Registry as Canonical Signal Registry
    participant SensorA as Sensor Type Adapter (A)
    participant DriverA as Sensor Hardware Driver (A)
    participant SensorB as Sensor Type Adapter (B)
    participant DriverB as Sensor Hardware Driver (B)
    participant Ctl as Controller
    participant Model as Torque Model
    participant Act as Actuator

    App->>Runtime: Build from profile (schemas + bindings)

    opt Configuration validation
        Runtime->>Registry: Validate canonical signal names
        Runtime->>Profile: Check required signals are provided

        Note over Runtime,SensorA: Capability vs. Hardware checks (A)
        Runtime->>SensorA: can_provide(required input signals)?
        SensorA-->>Runtime: supported, unsupported
        SensorA->>DriverA: probe_hardware(segments/resources)
        DriverA-->>SensorA: readiness per requirement
        SensorA-->>Runtime: readiness per signal

        Note over Runtime,SensorB: Optional sensor follows same pattern (B)
        Runtime->>SensorB: can_provide(required input signals)?
        SensorB-->>Runtime: supported, unsupported
        SensorB->>DriverB: probe_hardware(segments/resources)
        DriverB-->>SensorB: readiness per requirement
        SensorB-->>Runtime: readiness per signal

        Note over Runtime,Act: Outputs capability + hardware
        Runtime->>Act: can_accept(required output signals)?
        Act-->>Runtime: supported, unsupported
        Runtime->>Act: probe_hardware(supported outputs)
        Act-->>Runtime: readiness per output
        Runtime-->>App: Raise error on validation failure
    end

    Runtime->>SensorA: start()
    SensorA->>DriverA: open/connect
    Runtime->>SensorB: start() (optional)
    SensorB->>DriverB: open/connect (optional)
    Runtime->>Act: start()
    Runtime->>Ctl: reset()

    loop Each tick
        Runtime->>SensorA: read()
        SensorA->>DriverA: read_native()
        DriverA-->>SensorA: segment/state sample
        SensorA-->>Runtime: normalized sample (A)

        Runtime->>SensorB: read() (optional)
        SensorB->>DriverB: read_native() (optional)
        DriverB-->>SensorB: sample (optional)
        SensorB-->>Runtime: normalized sample (B)

        Runtime->>Ctl: tick(inputs)
        Ctl->>Model: run(features)
        Model-->>Ctl: feedforward torques
        Ctl-->>Runtime: TorqueCommand
        Runtime->>Act: apply(TorqueCommand)
        Runtime->>Act: fault_if_needed()
    end

    Runtime->>SensorA: stop()
    SensorA->>DriverA: close
    Runtime->>SensorB: stop() (optional)
    SensorB->>DriverB: close (optional)
    Runtime->>Act: stop()
```

<!-- Profile wiring sequence removed in favor of configuration validation block above. -->

## Interfaces overview (proposed)

```mermaid
classDiagram
    class InputSchema {
      +name: str
      +signals: [str]
      +required_signals() set
    }
    class ControllerManifest {
      +input_schema: InputSchema
      +output_schema: InputSchema
      +joints: [str]
    }
    class SensorDiagnostics {
      +last_timestamp: float?
      +hz_estimate: float?
      +hz_min/hz_max/hz_mean: float?
      +stale_samples: int
      +total_samples: int
    }
    class BaseSensorConfig {
      +max_stale_samples: int
      +max_stale_time_s: float
      +fault_strategy: str
      +diagnostics_window: int
    }
    class BaseSensor {
      <<abstract>>
      +config: BaseSensorConfig
      +diagnostics: SensorDiagnostics
      +start() (abstract)
      +stop() (abstract)
      +provided_signals() set (abstract)
      +can_provide(signals) (supported, unsupported) (abstract)
      +probe_hardware(signals) map (abstract)
      +read() AnySample (abstract)
    }
    class SensorTypeBase {
      <<abstract>>
      %% Implements BaseSensor abstract methods for a modality (e.g., IMU/GRF)
      +provided_signals() set (implemented)
      +can_provide(signals) (supported, unsupported) (implemented)
      +probe_hardware(signals) map (implemented)
      +read() AnySample (implemented)
    }
    class SensorHardwareDriver {
      <<abstract>>
      +start()/stop() (abstract)
      +probe_hardware(requirements) map (abstract)
      +read_native() NativeSample (abstract)
    }
    class BaseActuator {
      <<abstract>>
      +start()/stop() (abstract)
      +apply(TorqueCommand) (implemented)
      +fault_if_needed() (optional in subclass)
      +accepts_outputs() set (abstract)
      +can_accept(outputs) (supported, unsupported) (abstract)
      +probe_hardware(outputs) map (abstract)
    }

    SensorTypeBase --|> BaseSensor
    SensorTypeBase --> SensorHardwareDriver : delegates
    BaseSensor *-- SensorDiagnostics
    BaseSensor --> BaseSensorConfig
    ControllerManifest --> InputSchema
```

### Notation legend
- `*` abstract method (must be implemented by subclass)
- `•` implemented method (provided by the class shown)
- `°` optional method (subclass may override/implement)

## Measured vs. derived signals

Some canonical signals are measured directly by drivers (e.g., segment angles);
others are derived by the sensor type from prerequisites (e.g., joint angle
derived as the difference of two segment angles). The sensor type acts as an
orchestrator across multiple hardware drivers.

```mermaid
classDiagram
    class SignalResolver <<strategy>> {
      +requires: set~str~
      +compute(values: map) float
    }
    class SignalOrchestrator {
      +resolvers: map~str,SignalResolver~
      +dependency_graph: DAG
      +resolve(requested: set~str~, providers: map~driver,set~str~~) map
    }
    class SensorTypeBase {
      +orchestrator: SignalOrchestrator
      •read() AnySample
    }
    class SensorHardwareDriver {
      *read_native() NativeSample
      *provided_measurements(): set~str~
    }

    SensorTypeBase *-- SignalOrchestrator
    SignalOrchestrator ..> SignalResolver : uses
    SignalOrchestrator ..> SensorHardwareDriver : pulls measured prereqs
```

### Per‑tick resolution flow

```mermaid
sequenceDiagram
    participant ST as Sensor Type Orchestrator
    participant D1 as Driver A
    participant D2 as Driver B

    Note over ST: Determine closure of dependencies for requested signals
    ST->>D1: fetch measured prerequisites (subset)
    D1-->>ST: segment/measurement values
    ST->>D2: fetch measured prerequisites (subset)
    D2-->>ST: segment/measurement values
    ST->>ST: compute derived signals via resolvers (topological order)
    ST-->>Runtime: canonical signal map / normalized sample
```

During configuration validation, the orchestrator verifies capability (all
dependencies resolvable given driver-provided measurements) and hardware
availability (drivers can actually read the needed measurements).

Adapters should raise during initialisation if they cannot provide a requested
signal, preventing the runtime from entering the control loop with missing
channels.
