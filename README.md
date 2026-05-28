# Admittance Controller — Kinova Gen3 7-DOF

Full 6-DOF velocity-based admittance controller for the Kinova Gen3 arm with
MAE SensuReal F/T sensor. Push or twist the end-effector and it yields
compliantly. Release it and it springs back to the original pose — both
position and orientation.

Built for force-controlled contact tasks: surface wiping, polishing, guided
manipulation, and as the foundation for learned contact policies.

## What It Does

The controller maps external forces **and torques** to compliant Cartesian
motion using a velocity-based admittance law:

```
Linear:   v = R · (F / D_linear)  + Kp · (pos_desired − pos_current)
Angular:  ω = R · (τ / D_angular) + Kp_angular · 2 · q_err.vec()
```

Orientation error uses **quaternion representation** — no Euler angle
subtraction, no gimbal lock, no discontinuities. The short-path fix
(`if q_err.w < 0 → negate`) prevents >180° corrections.

At steady state with constant 5N force (D=150, Kp=2): equilibrium
displacement = 16.7mm. Remove the force → arm returns to home.

## Architecture

```
MAE SensuReal F/T Sensor (UDP, 1kHz)
  │
  ▼
mae_sensor_node (Python lifecycle, publishes /wrench_raw at 500Hz)
  │
  ▼
┌─────────────────────────────────────────────────────────────────┐
│                      admittance_node (C++, 100Hz)               │
│                                                                 │
│  /wrench_raw ---→ Gravity Comp ---→ EMA Filter ---→ Dead Zone   │
│                  (model-based)    (6 channels)   (F: 0.3N,      │
│                                                   T: 0.2Nm)     │
│                       │                                         │
│                       ▼                                         │
│              ┌─────────────────┐     ┌──────────────────────┐   │
│              │     LINEAR      │     │       ANGULAR        │   │
│              │F_base = R·F_tool│     │  q_err = q_d·q_c⁻¹   │   │
│              │  v = F/D + Kp·e │     │  ω = τ/D + Kp·2·q.v  │   │
│              └────────┬────────┘     └──────────┬───────────┘   │
│                       │                         │               │
│                       ▼                         ▼               │
│                 ┌─────────────────────────────────┐             │
│                 │  Safety Clamp + Dead Zone Check │             │
│                 │  v < 0.2m/s   ω < 0.9rad/s      │             │
│                 └──────────────┬──────────────────┘             │
│                                │                                │
│                                ▼                                │
│                   SendTwistCommand (6D, base frame)             │
│                   (angular: rad/s → deg/s for Kortex)           │
└─────────────────────────────────────────────────────────────────┘
```

## Hardware

| Component | Model | Role |
|-----------|-------|------|
| Arm | Kinova Gen3 7-DOF | Manipulator |
| F/T Sensor | MAE Robotics SensuReal | 6-axis wrench at 1kHz via UDP |
| End-Effector | Test handle (175g) | Validation payload (Robotiq 2F-85 next) |

The MAE sensor mounts between the flange and the tool. A Python lifecycle
node (`mae_sensor_node`) handles UDP streaming, sensor→tool frame rotation
(Rz 90°), and publishes `WrenchStamped` at 500Hz.

## Key Design Decisions

**Quaternion orientation error** — Euler angle subtraction breaks at gimbal
lock (pitch ±90°), wraps at ±180°, and doesn't represent a unique rotation.
Quaternion error `q_err = q_desired · q_current⁻¹` gives a single, smooth,
singularity-free rotation with `2 · q_err.vec() ≈ θ_error` in radians.

**Velocity-based admittance (not impedance)** — The arm's internal position
controller handles trajectory tracking. We command Cartesian velocities, not
torques. This is safer and doesn't require a dynamic model of the arm.

**Model-based gravity compensation** — `F_tool = R^T · [0, 0, −mg]` updated
every cycle from Kortex Euler angles. Residual error (imprecise mass, CoG,
friction) captured by tare at startup.

**Separate angular/linear gains** — Per-axis damping and regulation gains
allow independent tuning of translational compliance and rotational stiffness.

**Rad/s → deg/s conversion** — Kortex `SendTwistCommand` expects angular
velocity in degrees/s, not rad/s. Missing this conversion causes the arm to
appear frozen rotationally (0.05 rad/s interpreted as 0.05°/s ≈ nothing).

## Build

```bash
# CRITICAL: Build kinova_wrapper for real hardware first
colcon build --packages-select kinova_wrapper \
  --cmake-args -DUSE_KORTEX_MOCK=OFF

# Build admittance controller
colcon build --packages-select admittance_controller

source install/setup.bash
```

> **Mock build warning:** `kinova_wrapper` defaults to `USE_KORTEX_MOCK=ON`.
> Mock builds silently accept velocity commands without sending them to the
> arm. Always pass `-DUSE_KORTEX_MOCK=OFF` for hardware.

## Run

```bash
# Launch both nodes (mae_sensor_node + admittance_node)
ros2 launch admittance_controller admittance.launch.py

# Transition MAE sensor to active (separate terminal)
ros2 lifecycle set /mae_sensor_node configure
ros2 lifecycle set /mae_sensor_node activate

# After tare completes — enable the controller
ros2 service call /admittance_node/enable std_srvs/srv/Trigger
```

The node connects to the arm, captures the current pose as the home target
(position + orientation as quaternion), tares the F/T sensor, and starts the
100Hz control loop in disabled state. Call `~/enable` to begin compliant motion.

## Services

| Service | Type | Description |
|---------|------|-------------|
| `~/tare` | `std_srvs/Trigger` | Re-zero sensor bias at current orientation. Disables controller during tare. |
| `~/enable` | `std_srvs/Trigger` | Start compliant motion. Calls `stopMotion()` first to clear stale Kortex control modes. |
| `~/disable` | `std_srvs/Trigger` | Stop velocity commands. Watchdog auto-stops arm (~100ms). |

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/wrench_raw` | `WrenchStamped` | Subscribe | Raw F/T from MAE sensor (tool frame, 500Hz) |
| `~/wrench_corrected` | `WrenchStamped` | Publish | After gravity comp + EMA filter + dead zone |

## Parameters

All parameters are dynamically reconfigurable at runtime via `ros2 param set`.

### Linear Control

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `damping_x/y/z` | 150.0 | N·s/m | Force needed to produce 1 m/s velocity. Higher = stiffer. |
| `position_gain` | 2.0 | 1/s | Spring-back speed. Equilibrium displacement = F / (D × Kp). |

### Angular Control

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `damping_x/y/z_angular` | 5.0 | Nm·s/rad | Torque needed to produce 1 rad/s angular velocity. |
| `Kp_x/y/z_angular` | 0.5 | 1/s | Orientation regulation gain (per radian of quaternion error). |

### Safety

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `max_velocity` | 0.2 | m/s | Linear velocity clamp. |
| `max_angular_velocity` | 0.9 | rad/s | Angular velocity clamp (~28.6°/s). |
| `dead_zone_force` | 0.3 | N | Force readings below this are zeroed. |
| `dead_zone_torque` | 0.2 | Nm | Torque readings below this are zeroed. |

### Gravity Model

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `gripper_mass` | 0.175 | kg | Payload mass (test handle). Robotiq 2F-85: 0.93 kg. |
| `cog_x/y/z` | 0.0 | m | Center of gravity in tool frame. |

### Filter

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `filter_alpha` | 0.1 | 0.01–1.0 | EMA coefficient. Lower = smoother, laggier. Applied to all 6 channels. |

## Live Tuning

```bash
# Make more compliant (linear)
ros2 param set /admittance_node damping_x 80.0

# Make angular control softer
ros2 param set /admittance_node damping_x_angular 3.0
ros2 param set /admittance_node Kp_x_angular 0.5

# Adjust dead zone
ros2 param set /admittance_node dead_zone_force 0.5
ros2 param set /admittance_node dead_zone_torque 0.2

# Adjust filter responsiveness
ros2 param set /admittance_node filter_alpha 0.2
```

## Tuning Guide

Start conservative and loosen gradually. Always have the e-stop in hand.

| Step | Parameter | Value | What to Watch |
|------|-----------|-------|---------------|
| 1 | Defaults | D=150, Kp=2 | Arm should not move on its own |
| 2 | Lower linear D | D=100 | Push gently — slight compliance |
| 3 | Lower further | D=60 | Feels like a firm spring |
| 4 | Lower angular D | D_ang=5 | Twist EE — should rotate and spring back |
| 5 | Adjust dead zone | ±0.1N | Creep → raise. Unresponsive → lower. |
| 6 | Adjust filter | ±0.05 | Jittery → lower alpha. Laggy → raise. |

## Safety

The controller has multiple safety layers:

- **Velocity clamp**: linear (0.2 m/s) and angular (0.5 rad/s) hard limits
- **Dead zones**: force (0.3N) and torque (0.1Nm) reject sensor noise/drift
- **Position + orientation deadzone**: no force + close to home → zero twist
  (3mm position, ~0.86° orientation)
- **Sub-threshold suppression**: velocities below 0.1mm/s zeroed to prevent
  floating-point jerk
- **Watchdog**: `KinovaInterface` auto-stops arm if no velocity command
  received for ~100ms (node crash protection)
- **Disabled by default**: requires explicit `~/enable` service call
- **Tare safety**: controller disabled during tare to prevent stale-data spikes
- **Exclusive API control**: do NOT run alongside ros2_kortex or the Kortex
  Web App — concurrent sessions cause silent control mode conflicts

## File Structure

```
admittance_controller/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── admittance_params.yaml          # All tunable parameters
├── include/
│   └── admittance_controller/
│       └── types.hpp                   # Wrench, WrenchFilter, apply_deadzone
├── launch/
│   └── admittance.launch.py            # Launch mae_sensor_node + admittance_node
└── src/
    └── admittance_node.cpp             # 6-DOF admittance controller

mae_sensor_driver/                      # Separate package
├── mae_sensor_node.py                  # Python lifecycle node (UDP → ROS2)
└── ...
```

## Dependencies

- [kinova_wrapper](https://github.com/Sahilnarola-1007/kinova-wrapper) — C++ wrapper for Kortex SDK
- [mae_fts_sdk](https://github.com/MAE-Robotics) — MAE SensuReal Python SDK
- ROS2 Jazzy, Eigen3, geometry_msgs, std_srvs

## Author

Sahil Narola — MEng, Carleton University
Advanced Biomechatronics and Locomotion Lab, Ottawa
