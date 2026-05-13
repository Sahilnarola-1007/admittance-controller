# Admittance Controller — Kinova Gen3

**Day 22: Force/Torque Sensing Fundamentals**

A ROS2 node that makes your Kinova Gen3 compliant — push the end-effector
and it yields. Let go and it springs back.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   admittance_node                       │
│                                                         │
│  Kortex API ──→ Read Wrench ──→ Subtract Bias          │
│       ▲              │              │                   │
│       │              │         Filter (EMA)             │
│       │              │              │                   │
│       │              │         Dead Zone                │
│       │              │              │                   │
│       │              │     Admittance: F/K = dx         │
│       │              │              │                   │
│       │              │      Clamp + Rate Limit          │
│       │              │              │                   │
│       └──────────────┼── x_desired + dx = x_command     │
│                      │                                  │
│                      ▼                                  │
│              Publish WrenchStamped                      │
│           ~/wrench_corrected (debug)                    │
│           ~/wrench_raw (debug)                          │
└─────────────────────────────────────────────────────────┘
```

## Why Direct Kortex API?

The ros2_kortex driver doesn't expose `tool_external_wrench` data through
ros2_control state interfaces (see GitHub issue #186). So this node talks
to the Kortex API directly to access the wrench feedback.

**IMPORTANT:** Do NOT run this node alongside the ros2_kortex driver.
They will both try to control the arm and conflict.

## Build

```bash
# In your ROS2 workspace
cd ~/ros2_ws/src
cp -r /path/to/admittance_controller .

# Set the Kortex API path (adjust to your installation)
export KORTEX_DIR=~/kortex_api

# Build
cd ~/ros2_ws
colcon build --packages-select admittance_controller
source install/setup.bash
```

## Run

```bash
# With default parameters
ros2 launch admittance_controller admittance.launch.py

# With custom robot IP
ros2 launch admittance_controller admittance.launch.py robot_ip:=192.168.1.20

# With custom params file
ros2 launch admittance_controller admittance.launch.py \
  params_file:=/path/to/my_params.yaml
```

## Usage

Once running, the node will:

1. Connect to the Kinova at the specified IP
2. Read the current EE position (this becomes `x_desired`)
3. Tare the sensor (1 second, arm must be still)
4. Start the admittance loop

**Push the end-effector** → it moves in the direction of your push.
**Let go** → it returns to the original position.

## Services

```bash
# Re-tare the sensor (call after changing arm configuration)
ros2 service call /admittance_node/tare std_srvs/srv/Trigger

# Disable admittance (hold position rigidly)
ros2 service call /admittance_node/disable std_srvs/srv/Trigger

# Re-enable admittance
ros2 service call /admittance_node/enable std_srvs/srv/Trigger
```

## Live Parameter Tuning

You can change parameters while the node is running:

```bash
# Make it more compliant (lower K = more yielding)
ros2 param set /admittance_node stiffness_x 100.0
ros2 param set /admittance_node stiffness_y 100.0
ros2 param set /admittance_node stiffness_z 100.0

# Make it stiffer
ros2 param set /admittance_node stiffness_x 400.0

# Adjust dead zone
ros2 param set /admittance_node dead_zone_force 2.0

# Adjust filter (higher = more responsive, noisier)
ros2 param set /admittance_node filter_alpha 0.2

# Increase max displacement (careful!)
ros2 param set /admittance_node max_displacement_x 0.08
```

## Debugging with Topics

```bash
# Watch corrected wrench (after bias + filter + deadzone)
ros2 topic echo /admittance_node/wrench_corrected

# Watch raw wrench (straight from Kortex)
ros2 topic echo /admittance_node/wrench_raw

# Plot in real-time with rqt_plot
ros2 run rqt_plot rqt_plot \
  /admittance_node/wrench_corrected/wrench/force/x \
  /admittance_node/wrench_corrected/wrench/force/y \
  /admittance_node/wrench_corrected/wrench/force/z
```

## Tuning Guide

Start conservative and loosen gradually:

| Step | What to change | Value | What to watch |
|------|---------------|-------|---------------|
| 1 | Start with defaults | K=200, dead=1.5 | Verify arm doesn't move on its own |
| 2 | Lower K slightly | K=150 | Push gently — should feel slight give |
| 3 | Lower K more | K=100 | Should feel like a firm spring |
| 4 | Try soft | K=50 | Should feel like a soft spring |
| 5 | Adjust deadzone | ±0.5 | If arm creeps → raise. If unresponsive → lower |
| 6 | Adjust filter | ±0.05 | If jittery → lower alpha. If laggy → raise alpha |
| 7 | Increase max disp | +1cm | Only after you trust the system |

## Safety Notes

- **max_displacement** is your hard safety limit — the arm will NEVER
  move further than this from `x_desired`, no matter what.
- **max_velocity** rate-limits how fast the offset can change.
- Start with K=200 (very stiff). A K that's too low with a noisy
  wrench signal will make the arm oscillate.
- Always test with the e-stop in hand.
- The arm holds orientation constant — only position is admittance-controlled.

## File Structure

```
admittance_controller/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── admittance_params.yaml      # All tunable parameters
├── include/
│   └── admittance_controller/
│       └── types.hpp               # Wrench, CartesianPos, WrenchFilter
├── launch/
│   └── admittance.launch.py        # Launch file
└── src/
    └── admittance_node.cpp         # Main node (the whole pipeline)
```
