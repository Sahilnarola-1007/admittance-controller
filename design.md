# Admittance Controller — Design Document

## Purpose

This document captures the engineering rationale behind every design choice in the 6-DOF
admittance controller. It's intended for code reviewers, future contributors, and anyone
evaluating the technical depth of this system.

---

## Control Law

### Velocity-Based Admittance (Not Impedance)

Impedance control computes **torques**: `τ = M·ẍ + D·ẋ + K·Δx`. It requires an accurate
dynamic model of the arm (inertia matrix, Coriolis, gravity) and direct torque-level access
to the joints. The Kinova Gen3 Kortex API does not expose joint torque commands — it exposes
position, velocity, and trajectory interfaces.

Admittance control inverts the problem: **measure force → compute velocity**. The arm's
internal servo tracks the velocity. No dynamic model needed, inherently stable with high
servo bandwidth, and safe because the position controller enforces physical limits.

```
Linear:   v_base = R · (F_tool / D_linear)  + Kp · (x_desired − x_current)
Angular:  ω_base = R · (τ_tool / D_angular) + Kp_angular · 2 · q_err.vec()
```

The first term (F/D or τ/D) provides **compliance** — the arm yields to external loads.
The second term (Kp · error) provides **regulation** — the arm returns to the home pose.

At steady state with constant force F: `displacement = F / (D × Kp)`.

### Why Quaternions for Orientation Error

Three representations were considered for orientation error:

| Method | Pros | Cons |
|--------|------|------|
| Euler subtraction | Simple | Gimbal lock at pitch ±90°, wrapping at ±180°, order-dependent |
| Rotation matrix log | Singularity-free | Expensive (matrix log), axis extraction noisy near identity |
| Quaternion error | Singularity-free, cheap, smooth | Double-cover requires sign check |

The quaternion approach computes `q_err = q_desired · q_current⁻¹`, giving the unique
shortest rotation from current to desired. The vector part scaled by 2 approximates the
angular error in radians (exact for small angles via `sin(θ/2) ≈ θ/2`):

```cpp
Eigen::Quaterniond q_err = q_desired_ * q_current.inverse();
if (q_err.w() < 0.0) q_err.coeffs() *= -1.0;  // short-path fix
Eigen::Vector3d orient_error = 2.0 * q_err.vec();
```

The short-path fix handles the **double-cover problem**: quaternions `q` and `-q` represent
the same rotation, but produce opposite correction directions. When `w < 0`, the rotation
angle exceeds 180° — negating all coefficients selects the equivalent sub-180° rotation.

### Per-Axis Gains

Linear damping `D_x, D_y, D_z` and angular damping `D_x_ang, D_y_ang, D_z_ang` are
independent per axis. This allows, for example, stiff vertical compliance (high D_z for
surface contact) with free lateral motion (low D_x, D_y for wiping).

Similarly, orientation regulation gains `Kp_x_ang, Kp_y_ang, Kp_z_ang` can be tuned
independently — tight roll/pitch control with compliant yaw, for example.

---

## Gravity Compensation

### Model-Based Approach

Gravity compensation projects the known payload weight into the tool frame using the
current end-effector orientation:

```
F_gravity_tool = R_tool_to_base^T · [0, 0, -m·g]
T_gravity_tool = r_cog × F_gravity_tool
```

Where `R` is rebuilt every cycle from Kortex ZYX Euler angles via Eigen. Only the third
row of R is needed for the force projection (optimization: no full matrix multiply).

### Tare Captures Residual Error

The gravity model is imperfect: mass is approximate, CoG may be off, sensor mounting
introduces offsets, and internal friction creates static bias. Tare samples 100 readings
at startup, subtracts the model prediction, and averages the residual. This bias is
subtracted every control cycle.

Tare is valid near the tare orientation. Large orientation changes (>30°) may introduce
uncompensated gravity torque. Re-tare via the `~/tare` service after repositioning.

### Torque Gravity Compensation

With CoG at `[0, 0, 0]` (unknown for current test handle), the gravity torque model
produces zero. Tare absorbs the actual gravity torque at the tare pose. When the Robotiq
2F-85 is mounted (mass=0.93kg, cog_z=0.058m), the model will actively compensate
orientation-dependent gravity torque.

---

## Frame Conventions

```
Base frame (Kortex world)
  │
  │  R_tool_to_base = Rz(θz) · Ry(θy) · Rx(θx)   [ZYX intrinsic]
  │  Updated every cycle from Kortex Euler angles
  ▼
Tool frame (end-effector)
  │
  │  Rz(90°) rotation applied in mae_sensor_node.py
  │
  ▼
Sensor frame (MAE SensuReal native)
```

The MAE sensor is mounted rotated 90° CCW about Z relative to the tool frame convention.
The Python driver applies this rotation before publishing, so the C++ node always works
in tool frame.

Kortex `SendTwistCommand` with `CARTESIAN_REFERENCE_FRAME_BASE` expects velocities in
the base frame. Forces measured in tool frame are rotated: `F_base = R_tool_to_base · F_tool`.

**Critical unit convention:** Kortex expects angular velocity in **degrees/s**, not rad/s.
The controller computes in rad/s internally and converts at the send boundary:
`kinova_->setCartesianVelocity(vx, vy, vz, wx * RAD2DEG, wy * RAD2DEG, wz * RAD2DEG)`.

---

## Signal Processing Pipeline

### EMA Filter (All 6 Channels)

Exponential Moving Average with configurable `alpha` (default 0.1):

```
filtered += alpha * (raw - filtered)
```

Applied identically to Fx, Fy, Fz, Tx, Ty, Tz. Lower alpha = smoother but laggier.
At 100Hz with alpha=0.1, the effective cutoff frequency is ~1.7Hz — sufficient for
human-interaction forces, too slow for impact detection (future work).

### Dead Zone (Separate Force/Torque Thresholds)

Forces below 0.3N and torques below 0.1Nm are zeroed. The dead zone also subtracts
the threshold value so motion starts smoothly from zero velocity:

```cpp
if (|value| < threshold) return 0.0;
return (value > 0) ? (value - threshold) : (value + threshold);
```

This prevents a velocity discontinuity at the dead zone boundary.

### Sub-Threshold Velocity Suppression

Velocities below 0.1mm/s (1e-4 m/s) are zeroed. This eliminates floating-point noise
from quaternion conversions and prevents micro-jerk when the arm is nominally at rest.
Below Kortex's minimum velocity resolution — the arm physically cannot execute these.

---

## Safety Architecture

### Layered Defense

| Layer | Mechanism | Scope |
|-------|-----------|-------|
| 1 | Dead zone (force + torque) | Rejects sensor noise and thermal drift |
| 2 | Position + orientation deadzone | Zero twist when at home and no external load |
| 3 | Velocity clamp (linear + angular) | Hard limits on commanded velocity |
| 4 | Workspace boundary (in KinovaInterface) | Zero velocity at workspace edges |
| 5 | Watchdog thread (in KinovaInterface) | Auto-stop if no command for ~100ms |
| 6 | Disabled by default | No motion until explicit `~/enable` service call |
| 7 | Tare disables controller | Prevents stale-data velocity spikes during re-tare |

### Concurrent Session Protection

The admittance node takes exclusive API control via the Kortex session. Running
ros2_kortex or the Kortex Web App simultaneously causes silent control mode conflicts:
the arm gets stuck in "Angular Trajectory" mode and ignores `SendTwistCommand`. The
`~/enable` service calls `stopMotion()` before activating to clear any stale mode.

---

## Sensor Integration

### MAE SensuReal F/T Sensor

The MAE sensor streams 6-axis wrench data over UDP at up to 1kHz. A Python ROS2
lifecycle node (`mae_sensor_node`) handles:

- UDP connection management (connect on activate, disconnect on deactivate)
- Sampling period configuration
- Hardware tare (BIAS_SET command)
- Sensor-to-tool frame rotation
- Rate decimation (1kHz sensor → 500Hz publish)
- Lifecycle transitions for clean startup/shutdown

The admittance node subscribes to `/wrench_raw` and caches the latest reading behind
a mutex. The 100Hz control loop reads the most recent value — 5:1 oversampling ensures
fresh data every cycle.

### Fallback to Kortex Wrench

If no MAE sensor data arrives (node not running, sensor disconnected), the controller
falls back to `KinovaInterface::getWrench()` which reads the Kortex built-in F/T
estimation. This is lower quality (~10Hz, model-based rather than measured) but allows
basic testing without the external sensor.

---

## Thread Model

Single-threaded ROS2 executor. `spin_some()` drains all ready callbacks (timer,
subscriber, service) sequentially each iteration. The control loop timer fires at 100Hz.
The wrench subscriber callback runs at up to 500Hz between timer fires, updating the
cached wrench behind a mutex.

The MAE sensor node uses a dedicated background thread for the blocking UDP read loop
(`waits_response_bytes()`), preventing it from starving the ROS2 executor.

---

## Known Limitations

1. **Gravity torque compensation at large orientations** — With CoG at [0,0,0],
   gravity torque is not modeled. Tare absorbs it at the tare pose only. Large
   orientation changes introduce uncompensated torque bias.

2. **Single tare pose** — Tare is pose-dependent. Moving the arm significantly
   from the tare configuration degrades compensation accuracy.

3. **No impact detection** — EMA filter with alpha=0.1 is too slow to detect
   sudden contacts. Future work: dual-rate filter or threshold-based detector.

4. **No joint-limit awareness** — The admittance law operates in Cartesian space.
   Near joint limits, Kortex may reject velocity commands or produce unexpected
   motions. Future work: manipulability monitoring.

---

## Revision History

| Date | Change | Author |
|------|--------|--------|
| May 2026 | Initial 3-DOF admittance controller (force only) | Sahil Narola |
| May 2026 | Extend to 6-DOF: quaternion orientation error, torque admittance, angular damping/regulation, dead zone for torques, angular velocity clamp, rad/s→deg/s fix | Sahil Narola |
