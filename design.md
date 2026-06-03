# Admittance Controller — Design Document

## Purpose

This document captures the engineering rationale behind every design choice in the 6-DOF
admittance controller with integrated PI force control for surface wiping. It is intended
for code reviewers, future contributors, and anyone evaluating the technical depth of this
system.

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

---

## Force Control (PI with Anti-Windup)

### Architecture: Brain/Executor Split

Force control is split across two nodes:
- **wipe_node (brain):** sequences the state machine, publishes WipeSetpoint
- **admittance_node (executor):** runs the PI controller, sends velocity commands

This separation keeps the force control loop as tight as possible — the PI runs inside the
node that owns the Kortex API connection, with no inter-process latency.

### PI Controller Design

The ForceControllerPI is a single-axis PI regulator with conditional-integration anti-windup:

```cpp
e       = force_desired - force_measured
i_cand  = integral_ + e * dt_
v_unsat = -(kp * e + ki * i_cand)
v_sat   = clamp(v_unsat, -v_max, +v_max)

if (!saturated || sameSign(e, v_sat))
    integral_ = i_cand    // only accumulate when productive
```

The negation in `v_unsat = -(...)` is the sign flip: positive error (need more force)
produces negative velocity (move down into surface).

### Sign Convention (Hardware-Verified)

The sign chain was verified on hardware (June 2026):

```
Physical: press tool UP into surface
  → wrench_corrected Fz = NEGATIVE (e.g. -5 N)
  → admittance_node calls: force_pi_->update(+5.0, -(-5.0)) = update(5, 5)
  → error e = 5 - 5 = 0 (at setpoint, no correction needed) ✓

Physical: too little force (Fz = -1 N, desired = -5 N)
  → update(5, 1) → e = +4 → v_unsat = negative → arm presses DOWN ✓

Physical: too much force (Fz = -7 N, desired = -5 N)
  → update(5, 7) → e = -2 → v_unsat = positive → arm lifts UP ✓
```

### Anti-Windup Behavior

The conditional-integration scheme freezes the integral when the output is saturated AND
continued integration would make it worse (error and output have opposite signs). This
prevents runaway integral accumulation during sustained contact or velocity-limited phases.

**Known limitation:** At wipe direction reversals, the integral accumulated during steady
wiping causes force spikes (measured up to -7 N against -5 N target). The anti-windup
catches saturation-induced windup but not within-band accumulation. Future fix: add a
direct integral clamp or reduce Ki.

### Validated Gains

| Parameter | Value | Result |
|-----------|-------|--------|
| Kp | 0.001 | Low proportional — smooth approach, no jerk |
| Ki | 0.01 | Steady-state tracking within ±0.5 N of target |
| F_desired | 5.0 N | Appropriate for surface wiping contact |
| Wipe speed | 0.02 m/s | Smooth lateral motion during force control |
| Steady-state | [-5.5, -4.5] N | ±10% of target |
| Max overshoot | -7.0 N | At direction reversals (Ki windup) |
| Max undershoot | -3.0 N | During lateral trajectory disturbances |

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

---

## Loop Rate Analysis

### Measured Timing Breakdown

Timing instrumentation was added to the control loop to measure each section:

| Section | Time (ms) | Operation |
|---------|-----------|-----------|
| wrench | 0.0 | Mutex read of cached F/T sensor data |
| pose | 0.0 | Cached getCurrentPose() (background thread) |
| math | 0.2 | Gravity comp + EMA + admittance + quaternion error |
| twist | **73.3** | setCartesianVelocity() → Kortex SendTwistCommand() |
| **TOTAL** | **73.5** | **→ ~13 Hz effective loop rate** |

### Why setCartesianVelocity Cannot Be Threaded

The velocity command is the control output. Moving it to a background thread creates
two failure modes:

1. **Queueing:** The background thread sends command N while the loop has already computed
   N+1. The arm executes stale commands, lagging behind reality → oscillation.
2. **Dropping:** Command N is skipped. The arm receives nothing for one cycle → jerk or
   watchdog trigger.

Unlike pose reading (which tolerates ~40ms staleness), the velocity command must be the
last synchronous step in the loop.

### Pose Caching Optimization

The `getCurrentPose()` call (originally ~20ms) was moved to a background thread that
polls at ~25 Hz and caches the result behind a mutex. The control loop reads the cached
value in <1μs. Staleness analysis: at 0.15 m/s max velocity, the arm moves 6mm in the
worst-case 40ms window — within the 3mm deadzone tolerance.

### Adequacy for Surface Wiping

At 13 Hz with 20 mm/s wipe speed: `20 × 0.077 = 1.54 mm` per command update. For
centimeter-scale wipe strokes, this granularity is acceptable — motion appears smooth
and the PI force controller maintains ±0.5N tracking at 13 Hz.

### Upgrade Path

Kortex low-level servoing API operates at 1 kHz with joint-space commands. Migration
requires: Jacobian computation every cycle, Cartesian→joint velocity conversion via
DLS pseudo-inverse, 1ms real-time deadline (missed cycles fault actuators), and low-level
session lifecycle management. Target: Week 5-6 or when visual servoing requires >100 Hz.

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
At the effective 13Hz loop rate with alpha=0.1, the cutoff frequency is ~0.2Hz.

### Dead Zone (Separate Force/Torque Thresholds)

Forces below 0.3N and torques below 0.2Nm are zeroed. The dead zone also subtracts
the threshold value so motion starts smoothly from zero velocity:

```cpp
if (|value| < threshold) return 0.0;
return (value > 0) ? (value - threshold) : (value + threshold);
```

This prevents a velocity discontinuity at the dead zone boundary.

### Sub-Threshold Velocity Suppression

Velocities below 0.1mm/s (1e-4 m/s) are zeroed. This eliminates floating-point noise
from quaternion conversions and prevents micro-jerk when the arm is nominally at rest.

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
| 8 | Stale-setpoint fallback | Reverts to admittance if wipe_node stops publishing |

---

## Thread Model

MultiThreadedExecutor with a reentrant callback group for I/O-bound subscribers
(wrench + setpoint). The control loop timer fires at 100Hz (effective ~13Hz due to
blocking Kortex call). A dedicated pose-caching thread polls getCurrentPose() at ~25Hz
independently.

The MAE sensor node uses a dedicated background thread for the blocking UDP read loop
(`waits_response_bytes()`), preventing it from starving the ROS2 executor.

---

## Known Limitations

1. **13 Hz effective loop rate** — Kortex high-level API SendTwistCommand blocks for
   ~73ms. Adequate for slow contact tasks (wiping, polishing), insufficient for fast
   visual servoing (>100Hz required). Upgrade: low-level servoing API.

2. **Ki integral windup at direction reversals** — accumulated integral during steady
   wiping causes force spikes up to 2× target at trajectory reversals. Fix: direct
   integral clamp or reduced Ki (0.005).

3. **Single tare pose** — gravity compensation accuracy degrades >30° from tare
   orientation. Re-tare after repositioning.

4. **No impact detection** — EMA filter too slow to detect sudden contacts. Future:
   dual-rate filter or threshold-based detector.

5. **No joint-limit awareness** — admittance law in Cartesian space. Near joint limits,
   Kortex may reject commands. Future: manipulability monitoring.

---

## Revision History

| Date | Change | Author |
|------|--------|--------|
| May 2026 | Initial 3-DOF admittance controller (force only) | Sahil Narola |
| May 2026 | Extend to 6-DOF: quaternion orientation error, torque admittance | Sahil Narola |
| June 2026 | PI force control, wipe integration, pose caching, loop rate diagnosis | Sahil Narola |
| June 2026 | Surface wiping demo validated (5N contact, 0.02m/s, Kp=0.001 Ki=0.01) | Sahil Narola |
