# Diagnostics Sinks

This runtime supports pluggable diagnostics sinks. For most development cases,
the CSV sink is a simple, dependency-free way to persist feature/torque logs.

## CSV sink usage

```python
from rpc_runtime.runtime.diagnostics import CSVDiagnosticsSink
from rpc_runtime.runtime.loop import RuntimeLoop, RuntimeLoopConfig

# Choose an output file; the parent directory is created if missing.
sink = CSVDiagnosticsSink("diagnostics/run.csv", include_segments=False)

loop = RuntimeLoop(
    RuntimeLoopConfig(frequency_hz=100.0),
    imu=imu,
    actuator=actuator,
    controller=controller,  # controller must support compute_torque(features, timestamp)
    vertical_grf=vertical_grf,  # optional
    diagnostics=sink,
)

with loop:
    for _ in loop.run(duration_s=2.0):
        pass
```

The CSV columns are constructed on the first tick and include:
- `timestamp`
- `imu_joint_angle_{i}` and `imu_joint_vel_{i}` for each joint index
- Optional `seg_angle_{i}` and `seg_vel_{i}` (when `include_segments=True`)
- Optional `grf_force_{i}` if a vertical GRF sensor is present
- `torque_raw_{joint}` and `torque_safe_{joint}` for each joint key
- `scheduler_{key}` for scheduler metrics

For in-memory logging during tests, use `InMemoryDiagnosticsSink` instead.
