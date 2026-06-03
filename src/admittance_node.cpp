// =============================================================================
// Admittance Controller Node — Kinova Gen3 + MAE SensuReal F/T Sensor
// =============================================================================
//
// Full 6-DOF velocity-based admittance controller and force-control executor
// that maps external forces AND torques to compliant end-effector motion
// (linear + angular). Designed for force-controlled contact tasks such as
// surface wiping, polishing, and guided manipulation.
//
// Successfully demonstrated: surface wiping with PI force control maintaining
// ~5 N contact force (steady-state band: [-5.5, -4.5] N) while executing
// lateral zigzag trajectories at 0.02 m/s. June 2026.
//
// Architecture:
//   MAE SensuReal F/T sensor (UDP, 1kHz)
//     → mae_sensor_node (Python lifecycle driver, publishes /wrench_raw at 500Hz)
//       → this node (subscribes, compensates gravity, filters, controls)
//         → Kortex SendTwistCommand (streaming 6D Cartesian twist)
//
// Control law (6-DOF velocity-based admittance):
//
//   Linear:
//     v_base = R · (F_tool / D_linear) + Kp_linear · (pos_desired − pos_current)
//
//   Angular:
//     q_err  = q_desired · q_current⁻¹           (quaternion orientation error)
//     ω_base = R · (τ_tool / D_angular) + Kp_angular · 2 · q_err.vec()
//
//   Quaternion error uses short-path fix: if q_err.w < 0, negate all coeffs
//   to avoid >180° corrections. The 2× scaling on q_err.vec() recovers the
//   angular error in radians (exact for small angles, well-behaved for large).
//
// Gravity compensation (model-based, orientation-independent):
//   F_contact = F_measured − R^T · [0, 0, −m·g] − bias_residual
//   R updates every cycle from Kortex Euler angles → works at any configuration.
//   Tare captures residual model error (imprecise mass, CoG, friction).
//   Torque gravity compensation: T_gravity = r_cog × F_gravity (tool frame).
//
// Sign convention (HARDWARE-VERIFIED):
//   Press UP (into surface, contact direction)  →  wrench_corrected Fz = NEGATIVE
//   Pull DOWN (away from surface)               →  wrench_corrected Fz = POSITIVE
//   PI call: vz = force_pi_->update(F_desired, -fz)
//   The -fz flips the sensor's negative contact reading to positive, matching
//   the positive F_desired. The PI's internal negation then produces negative
//   vz (downward) when more force is needed.
//
// Frame conventions:
//   - Sensor frame: MAE SensuReal native axes
//   - Tool frame:   Sensor readings rotated by Rz(90°) in mae_sensor_node.py
//   - Base frame:   Kortex world frame, used for SendTwistCommand
//   - R_tool_to_base built from Kortex ZYX intrinsic Euler angles using Eigen
//
// LOOP RATE LIMITATION (measured, not theoretical):
//   The control loop is configured at 100 Hz but achieves only ~13 Hz (~75 ms
//   per cycle) due to Kortex high-level API latency. Timing breakdown:
//     wrench read:           0.0 ms  (mutex-cached from 500 Hz subscriber)
//     getCurrentPose():      0.0 ms  (cached by background pose thread)
//     control math:          0.2 ms  (gravity comp + EMA + admittance + quaternion)
//     setCartesianVelocity: 73.0 ms  (Kortex SendTwistCommand gRPC round-trip)
//   The SendTwistCommand call is a synchronous gRPC request to the arm's internal
//   controller. It cannot be moved to a background thread because the velocity
//   command is the control output — fire-and-forget would cause stale or dropped
//   commands. Kinova's own documentation states 40 Hz max for high-level commands.
//   At 13 Hz with 20 mm/s wipe speed, the arm moves 1.5 mm between updates —
//   acceptable for surface wiping. Upgrade path: Kortex low-level servoing API
//   (1 kHz, joint-space, requires Jacobian + DLS IK).
//
// End-effector payload:
//   Currently configured for a lightweight test handle (~175g) mounted
//   directly on the F/T sensor flange for controller validation. No gripper
//   attached. Sequence: robot → flange → F/T sensor → test handle.
//   When Robotiq 2F-85 is installed, update gravity model parameters:
//     gripper_mass: 0.93 kg, cog_z: 0.058 m
//
// Safety:
//   - Linear velocity clamped to max_velocity (default 0.15 m/s)
//   - Angular velocity clamped to max_angular_velocity (default 0.5 rad/s)
//   - Dead zones for both force (0.3N) and torque (0.1Nm)
//   - Position deadzone (3mm) and orientation deadzone (0.5°) prevent oscillation
//   - Watchdog thread in KinovaInterface auto-stops arm if node crashes (~100ms)
//   - Disabled by default — requires explicit ~/enable service call
//   - Tare sets enabled_=false to prevent stale-data velocity spikes
//   - stopMotion() called on enable to clear any active Kortex control mode
//   - E-stop integration via KinovaInterface::emergencyStop()
//
// Services:
//   ~/tare     std_srvs/Trigger  Re-zero sensor bias at current orientation
//   ~/enable   std_srvs/Trigger  Start compliant motion (clears control mode first)
//   ~/disable  std_srvs/Trigger  Stop velocity commands, watchdog halts arm
//
// Topics published:
//   ~/wrench_corrected  WrenchStamped  After gravity comp + EMA filter + dead zone
// Topics subscribed:
//   /wrench_raw         WrenchStamped  Raw F/T from mae_sensor_node (tool frame)
//   /wipe_node/wipe_setpoint

// =============================================================================
// WIPE INTEGRATION (surface-wiping demo)
//
// This node also acts as the EXECUTOR for the surface-wiping task. The
// wipe_node ("brain") sequences a state machine and publishes WipeSetpoint
// commands; this node applies them on top of the admittance loop:
//
//   mode == IDLE      → normal 6-DOF admittance (unchanged)
//   mode == APPROACH  → Z velocity-commanded (slow descent), X/Y = 0
//   mode == WIPE      → Z force-controlled via ForceControllerPI (PI + anti-
//                       windup) toward force_desired_z; X/Y commanded from
//                       the wipe trajectory; position regulation OFF on X/Y/Z
//   mode == RETRACT   → Z velocity-commanded (lift off), X/Y = 0
//
// In every wipe mode the ANGULAR (orientation hold) control is left untouched —
// the tool stays normal to the surface via the existing quaternion controller.
//
// Bumpless transfer: the PI integral is reset on the rising edge of
// z_force_control (APPROACH→WIPE), so force control starts from a clean state.
//
// Stale-setpoint safety: if no WipeSetpoint arrives within setpoint_timeout
// seconds (brain crashed / stopped), the node reverts to normal admittance —
// it does NOT keep pressing or wiping unsupervised.
//
// Force feedback for the PI is the corrected vertical force (post gravity-comp,
// filter, dead zone) — the SAME signal published on ~/wrench_corrected and used
// by the wipe_node for contact detection, so contact threshold and force target
// share one frame and sign.
//
// New services:  (none — wipe is driven entirely by the setpoint topic)
// New parameters:
//   force_kp / force_ki   PI gains for Z-axis force control
//   setpoint_timeout      Stale-setpoint fallback threshold [s]
// Topics subscribed (added):
//   /wipe_node/wipe_setpoint   WipeSetpoint  Commands from the wipe_node brain
// =============================================================================

// Parameters (all dynamically reconfigurable via `ros2 param set`):
//   damping_x/y/z              Linear damping D [N·s/m]
//   damping_x/y/z_angular      Angular damping D [Nm·s/rad]
//   position_gain              Linear spring-back Kp [1/s]
//   Kp_x/y/z_angular           Orientation regulation gain [1/s]
//   dead_zone_force/torque      Noise thresholds [N] / [Nm]
//   max_velocity               Linear safety clamp [m/s]
//   max_angular_velocity       Angular safety clamp [rad/s]
//   filter_alpha               EMA coefficient [0–1]
//   gripper_mass               Payload mass [kg]
//   cog_x/y/z                  Center of gravity, tool frame [m]
//
// Build:
//   colcon build --packages-select admittance_controller
//   CRITICAL: kinova_wrapper must be built with -DUSE_KORTEX_MOCK=OFF for
//   real hardware. Mock builds silently accept commands without sending them.
//
// IMPORTANT: This node takes exclusive API control of the arm. Do NOT run
// ros2_kortex or the Kortex Web App simultaneously — concurrent sessions
// cause control mode conflicts (arm stuck in "Angular Trajectory", silently
// ignoring twist commands).
//
// Author:  Sahil Narola
// Date:    May 2026
// =============================================================================

#include <chrono>
#include <memory>
#include <atomic>
#include <csignal>
#include <cmath>
#include <mutex>
#include <thread>
#include <memory>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "admittance_controller/types.hpp"
#include "admittance_controller/force_controller_pi.hpp"   
#include "kinova_wrapper/KinovaInterface.hpp"
#include "wipe_msgs/msg/wipe_setpoint.hpp"                 

using namespace std::chrono_literals;

// Global flag for clean SIGINT shutdown
static std::atomic<bool> g_running{true};
void signal_handler(int) { g_running = false; }

// =============================================================================
// AdmittanceNode
// =============================================================================
class AdmittanceNode : public rclcpp::Node {
public:
  AdmittanceNode() : Node("admittance_node") {
    // ------------------------------------------------------------------
    // Parameter declarations
    //
    // Defaults here are overridden by YAML (via launch file) or CLI.
    // All gains are dynamically reconfigurable at runtime:
    //   ros2 param set /admittance_node damping_x 300.0
    // ------------------------------------------------------------------
    declare_parameter("robot_ip", "192.168.1.10");

    declare_parameter("loop_rate_hz", 100);

    // Linear damping gains D [N·s/m]: force required to produce 1 m/s velocity
    declare_parameter("damping_x", 150.0);
    declare_parameter("damping_y", 150.0);
    declare_parameter("damping_z", 150.0);

    // Angular damping gains D [Nm·s/rad]: torque required to produce 1 rad/s
    declare_parameter("damping_x_angular", 10.0);
    declare_parameter("damping_y_angular", 10.0);
    declare_parameter("damping_z_angular", 10.0);

    // Linear position regulation gain Kp [1/s]: controls spring-back speed
    // Equilibrium displacement = F / (D × Kp)
    declare_parameter("position_gain", 2.0);

    // Orientation regulation gains [1/s]: proportional angular velocity
    // per radian of orientation error
    declare_parameter("Kp_x_angular", 2.0);
    declare_parameter("Kp_y_angular", 2.0);
    declare_parameter("Kp_z_angular", 2.0);

    // Dead zones: readings below these are zeroed
    declare_parameter("dead_zone_force", 0.3);    // N  (MAE drift: ~0.1–0.3N)
    declare_parameter("dead_zone_torque", 0.1);   // Nm

    // Safety limits
    declare_parameter("max_velocity", 0.15);            // m/s
    declare_parameter("max_angular_velocity", 0.5);     // rad/s

    // EMA filter coefficient: 0.1 = heavy smoothing, 1.0 = no filtering
    declare_parameter("filter_alpha", 0.1);

    // Tare configuration
    declare_parameter("tare_samples", 100);
    declare_parameter("tare_sample_interval_ms", 10);

    // Gravity model — end-effector payload
    declare_parameter("gripper_mass", 0.175);
    declare_parameter("cog_x", 0.0);
    declare_parameter("cog_y", 0.0);
    declare_parameter("cog_z", 0.0);

    // Force control (Z-axis during WIPE)
    declare_parameter("force_kp", 0.001);
    declare_parameter("force_ki", 0.01);
    declare_parameter("setpoint_timeout", 0.5);   // s, stale-setpoint safety

    load_params();

    // ------------------------------------------------------------------
    // Publisher: corrected wrench for debugging and visualization
    // Published AFTER gravity comp + EMA filter + dead zone
    // ------------------------------------------------------------------
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/wrench_corrected", 10);

    io_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions io_opts;
    io_opts.callback_group = io_cb_group_;

    // ------------------------------------------------------------------
    // Subscriber: raw wrench from MAE sensor driver (tool frame, 500Hz)
    //
    // Caches the latest reading behind a mutex. The 100Hz control loop
    // reads the most recent value each cycle — intermediate readings
    // are overwritten (5:1 oversampling ensures fresh data every cycle).
    // ------------------------------------------------------------------
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench_raw", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(wrench_mutex_);

        // Readings arrive in tool frame (sensor→tool rotation applied
        // in mae_sensor_node.py via Rz(90°) before publishing)
        latest_wrench_.fx = msg->wrench.force.x;
        latest_wrench_.fy = msg->wrench.force.y;
        latest_wrench_.fz = msg->wrench.force.z;
        latest_wrench_.tx = msg->wrench.torque.x;
        latest_wrench_.ty = msg->wrench.torque.y;
        latest_wrench_.tz = msg->wrench.torque.z;
        wrench_received_ = true;
      },io_opts);

      // Subscriber: wipe setpoints from the wipe_node (the "brain")
      setpoint_sub_ = create_subscription<wipe_msgs::msg::WipeSetpoint>(
        "/wipe_node/wipe_setpoint", 10,
        [this](const wipe_msgs::msg::WipeSetpoint::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(setpoint_mutex_);
          latest_setpoint_     = *msg;
          setpoint_received_   = true;
          last_setpoint_time_  = now();
        },io_opts);

    // ------------------------------------------------------------------
    // Services
    // ------------------------------------------------------------------

    // ~/tare: re-zero sensor at current orientation.
    // Sets enabled_=false first — prevents queued control_loop callbacks
    // from computing velocities with stale data during the ~1s blocking tare.
    tare_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/tare",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        RCLCPP_INFO(get_logger(), "Tare requested — hold the arm still...");
        enabled_=false;
        tare_sensor();
        response->success = true;
        response->message = "Sensor tared successfully.";
      });

    // ~/enable: start compliant motion.
    // Calls stopMotion() to clear any active Kortex control mode
    // (e.g. "Angular Trajectory" from a previous session or the Web App)
    // which would silently block SendTwistCommand.
    enable_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/enable",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {

        kinova_->stopMotion();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        enabled_ = true;
        response->success = true;
        response->message = "Admittance control enabled.";
        RCLCPP_INFO(get_logger(), "Admittance control ENABLED");
      });

    // ~/disable: stop velocity commands. Watchdog auto-stops arm (~100ms).
    disable_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/disable",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        enabled_ = false;
        response->success = true;
        response->message = "Admittance control disabled. Holding position.";
        RCLCPP_INFO(get_logger(), "Admittance control DISABLED — holding position");
      });

    // ------------------------------------------------------------------
    // Dynamic parameter reconfiguration — live tuning without restart
    // ------------------------------------------------------------------
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params)
        -> rcl_interfaces::msg::SetParametersResult {
        for (const auto& p : params) {
          // Linear damping
          if      (p.get_name() == "damping_x") D_x_ = p.as_double();
          else if (p.get_name() == "damping_y") D_y_ = p.as_double();
          else if (p.get_name() == "damping_z") D_z_ = p.as_double();
          // Angular damping
          else if (p.get_name() == "damping_x_angular") D_x_ang_ = p.as_double();
          else if (p.get_name() == "damping_y_angular") D_y_ang_ = p.as_double();
          else if (p.get_name() == "damping_z_angular") D_z_ang_ = p.as_double();
          // Gains
          else if (p.get_name() == "position_gain") K_p_ = p.as_double();
          else if (p.get_name() == "Kp_x_angular") Kp_x_ang_ = p.as_double();
          else if (p.get_name() == "Kp_y_angular") Kp_y_ang_ = p.as_double();
          else if (p.get_name() == "Kp_z_angular") Kp_z_ang_ = p.as_double();
          // Dead zones
          else if (p.get_name() == "dead_zone_force") dead_zone_force_ = p.as_double();
          else if (p.get_name() == "dead_zone_torque") dead_zone_torque_ = p.as_double();
          // Filter
          else if (p.get_name() == "filter_alpha") filter_.set_alpha(p.as_double());
          // Safety
          else if (p.get_name() == "max_velocity") max_vel_ = p.as_double();
          else if (p.get_name() == "max_angular_velocity") max_ang_vel_ = p.as_double();
          // Gravity model
          else if (p.get_name() == "gripper_mass") gripper_mass_ = p.as_double();
          else if (p.get_name() == "cog_x") cog_x_ = p.as_double();
          else if (p.get_name() == "cog_y") cog_y_ = p.as_double();
          else if (p.get_name() == "cog_z") cog_z_ = p.as_double();
          RCLCPP_INFO(get_logger(), "Parameter updated: %s", p.get_name().c_str());
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    RCLCPP_INFO(get_logger(), "AdmittanceNode created. Call init() to connect.");
  }

  // ========================================================================
  // init(): connect to arm, capture desired pose + orientation, tare, start loop
  // ========================================================================
  bool init() {
    std::string ip = get_parameter("robot_ip").as_string();
    RCLCPP_INFO(get_logger(), "Connecting to Kinova Gen3 at %s ...", ip.c_str());

    kinova_ = std::make_shared<kinova_wrapper::KinovaInterface>();
    bool connected = kinova_->connect(ip);

    if (!connected) {
      RCLCPP_ERROR(get_logger(), "Not connected");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Connected to Kinova Gen3 successfully!");

    // Capture startup pose as the desired (home) position + orientation.
    // The arm springs back to this pose when external force/torque is removed.
    desired_pos_ = kinova_->getCurrentPose();
    RCLCPP_INFO(get_logger(),
      "Initial EE pose: x=%.4f, y=%.4f, z=%.4f, "
      "θx=%.2f°, θy=%.2f°, θz=%.2f°",
      desired_pos_.x, desired_pos_.y, desired_pos_.z,
      desired_pos_.theta_x, desired_pos_.theta_y, desired_pos_.theta_z);

    // Convert desired orientation from Kortex Euler (deg) to quaternion
    q_desired_ = euler_deg_to_quaternion(
      desired_pos_.theta_x, desired_pos_.theta_y, desired_pos_.theta_z);
    RCLCPP_INFO(get_logger(),
      "Desired quaternion: w=%.4f, x=%.4f, y=%.4f, z=%.4f",
      q_desired_.w(), q_desired_.x(), q_desired_.y(), q_desired_.z());

    // Tare at startup to capture residual gravity model error
    RCLCPP_INFO(get_logger(), "Taring sensor — keep the arm still...");
    tare_sensor();
    RCLCPP_INFO(get_logger(),
      "Tare complete. Residual bias: F=[%.3f, %.3f, %.3f] N, "
      "T=[%.3f, %.3f, %.3f] Nm",
      bias_.fx, bias_.fy, bias_.fz,
      bias_.tx, bias_.ty, bias_.tz);

    // Start background pose-caching thread (decouples 20ms Kortex call)
    start_pose_thread();

    // Start the control loop timer
    int rate = get_parameter("loop_rate_hz").as_int();

    double dt = 1.0 / rate;
    // v_max for the force loop reuses the linear velocity clamp.
    force_pi_ = std::make_unique<admittance::ForceControllerPI>(
      force_kp_, force_ki_, dt, max_vel_);
    
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      [this]() { control_loop(); });

    enabled_ = false;
    RCLCPP_INFO(get_logger(),
      "6-DOF admittance controller running at %d Hz.", rate);
    RCLCPP_INFO(get_logger(),
      "  Linear:  Dx=%.0f Dy=%.0f Dz=%.0f | Kp=%.1f",
      D_x_, D_y_, D_z_, K_p_);
    RCLCPP_INFO(get_logger(),
      "  Angular: Dx=%.1f Dy=%.1f Dz=%.1f | Kp=[%.1f, %.1f, %.1f]",
      D_x_ang_, D_y_ang_, D_z_ang_, Kp_x_ang_, Kp_y_ang_, Kp_z_ang_);
    RCLCPP_INFO(get_logger(),
      "  Dead zone: %.1f N / %.2f Nm | Max vel: %.2f m/s / %.2f rad/s",
      dead_zone_force_, dead_zone_torque_, max_vel_, max_ang_vel_);
    RCLCPP_INFO(get_logger(),
      "  Payload mass: %.3f kg | CoG: [%.3f, %.3f, %.3f] m",
      gripper_mass_, cog_x_, cog_y_, cog_z_);
    RCLCPP_INFO(get_logger(), "Services: ~/tare  ~/enable  ~/disable");
    RCLCPP_INFO(get_logger(), "Topics:   ~/wrench_corrected  (subscribes: /wrench_raw)");

    return true;
  }

  // ========================================================================
  // cleanup(): graceful shutdown — stop motion, disconnect
  // ========================================================================
  void cleanup() {
    if (timer_) timer_->cancel();
    stop_pose_thread();
    RCLCPP_INFO(get_logger(), "Shutting down...");
    if (kinova_) {
      kinova_->stopMotion();
      kinova_->disconnect();
    }
    RCLCPP_INFO(get_logger(), "Disconnected from Kinova Gen3.");
  }

private:

  // ========================================================================
  // euler_deg_to_quaternion(): Kortex ZYX Euler (degrees) → Eigen quaternion
  //
  // Kortex convention: ZYX intrinsic Euler angles in degrees.
  // Eigen AngleAxisd composition: Rz · Ry · Rx (applied right to left).
  // ========================================================================
  Eigen::Quaterniond euler_deg_to_quaternion(double tx_deg, double ty_deg,
                                              double tz_deg) {
    double rx = tx_deg * DEG2RAD;
    double ry = ty_deg * DEG2RAD;
    double rz = tz_deg * DEG2RAD;

    Eigen::Quaterniond q =
        Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());

    return q.normalized();
  }

  // ========================================================================
  // load_params(): cache ROS parameters into member variables
  // Avoids parameter server lookups in the 100Hz control loop.
  // ========================================================================
  void load_params() {
    // Linear damping
    D_x_ = get_parameter("damping_x").as_double();
    D_y_ = get_parameter("damping_y").as_double();
    D_z_ = get_parameter("damping_z").as_double();

    // Angular damping
    D_x_ang_ = get_parameter("damping_x_angular").as_double();
    D_y_ang_ = get_parameter("damping_y_angular").as_double();
    D_z_ang_ = get_parameter("damping_z_angular").as_double();

    // Gains
    K_p_ = get_parameter("position_gain").as_double();
    Kp_x_ang_ = get_parameter("Kp_x_angular").as_double();
    Kp_y_ang_ = get_parameter("Kp_y_angular").as_double();
    Kp_z_ang_ = get_parameter("Kp_z_angular").as_double();

    // Dead zones
    dead_zone_force_ = get_parameter("dead_zone_force").as_double();
    dead_zone_torque_ = get_parameter("dead_zone_torque").as_double();

    // Safety
    max_vel_ = get_parameter("max_velocity").as_double();
    max_ang_vel_ = get_parameter("max_angular_velocity").as_double();

    filter_.set_alpha(get_parameter("filter_alpha").as_double());

    // Gravity model
    gripper_mass_ = get_parameter("gripper_mass").as_double();
    cog_x_ = get_parameter("cog_x").as_double();
    cog_y_ = get_parameter("cog_y").as_double();
    cog_z_ = get_parameter("cog_z").as_double();

    //Force controller PI
    force_kp_ = get_parameter("force_kp").as_double();
    force_ki_ = get_parameter("force_ki").as_double();
    setpoint_timeout_ = get_parameter("setpoint_timeout").as_double();
  }

  // ========================================================================
  // read_wrench(): get latest wrench from /wrench_raw (MAE sensor)
  //
  // Primary: cached value from subscriber callback (updated at 500Hz).
  // Fallback: Kortex built-in F/T via getWrench() (lower quality, ~10Hz).
  // Fallback allows testing without the external MAE sensor connected.
  // ========================================================================
  admittance::Wrench read_wrench() {
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      if (wrench_received_) {
        return latest_wrench_;
      }
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "No wrench data on /wrench_raw — falling back to Kortex getWrench()");

    auto w_vec = kinova_->getWrench();
    admittance::Wrench w;
    if (w_vec.size() == 6) {
      w.fx = w_vec[0]; w.fy = w_vec[1]; w.fz = w_vec[2];
      w.tx = w_vec[3]; w.ty = w_vec[4]; w.tz = w_vec[5];
    }
    return w;
  }

  // ========================================================================
  // Pose caching — background thread polls getCurrentPose() at ~50Hz
  //
  // getCurrentPose() blocks for ~20ms (Kortex network round-trip).
  // Running it in the control loop caps the loop at ~14Hz. This thread
  // polls independently and caches the result behind a mutex. The control
  // loop reads the cached value in <1μs instead of blocking 20ms.
  //
  // Staleness: at 50Hz polling, the pose is at most ~20ms old. The arm
  // moves at <0.15 m/s max → worst-case position error = 3mm, well within
  // the 3mm deadzone. Acceptable for admittance control.
  // ========================================================================
  void start_pose_thread() {
    pose_thread_running_ = true;
    pose_thread_ = std::thread([this]() {
      while (pose_thread_running_ && g_running) {
        auto pose = kinova_->getCurrentPose();
        {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          cached_pose_ = pose;
          pose_received_ = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
      }
    });
  }

  void stop_pose_thread() {
    pose_thread_running_ = false;
    if (pose_thread_.joinable()) {
      pose_thread_.join();
    }
  }

  // Read cached pose — returns last value from background thread.
  // Falls back to blocking call if thread hasn't produced a value yet.
  kinova_wrapper::Pose read_cached_pose() {
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      if (pose_received_) {
        return cached_pose_;
      }
    }
    // Fallback: first cycle before thread has run — block once
    return kinova_->getCurrentPose();
  }

  // ========================================================================
  // computeGravityWrench(): model-based gravity compensation
  //
  // Computes expected gravity wrench in tool frame from current orientation.
  //
  // Method:
  //   R_tool_to_world = Rz(θz) · Ry(θy) · Rx(θx)    [ZYX intrinsic]
  //   F_tool = R^T · [0, 0, −m·g] = −m·g · [−sθy, cθy·sθx, cθy·cθx]
  //   T_tool = r_cog × F_tool
  //
  // Works at any arm configuration because R updates from Kortex every cycle.
  // Residual error (imprecise mass, CoG, friction) is captured by tare.
  //
  // Kortex Euler convention: ZYX intrinsic, angles in degrees.
  // ========================================================================
  admittance::Wrench computeGravityWrench(double theta_x_deg,
                                           double theta_y_deg,
                                           [[maybe_unused]] double theta_z_deg) {
    double rx = theta_x_deg * DEG2RAD;
    double ry = theta_y_deg * DEG2RAD;

    double cx = std::cos(rx), sx = std::sin(rx);
    double cy = std::cos(ry), sy = std::sin(ry);

    // Only the third column of R^T is needed for gravity projection:
    //   F_tool = R^T · [0, 0, −mg] = −mg · third_col(R^T) = −mg · third_row(R)
    //   R[2] = [−sy, cy·sx, cy·cx]

    double mg = gripper_mass_ * 9.81;

    double fg_x = -mg * (-sy);
    double fg_y = -mg * (cy * sx);
    double fg_z = -mg * (cy * cx);

    // Torque from lever arm: T = r_cog × F_gravity_tool
    double tg_x = cog_y_ * fg_z - cog_z_ * fg_y;
    double tg_y = cog_z_ * fg_x - cog_x_ * fg_z;
    double tg_z = cog_x_ * fg_y - cog_y_ * fg_x;

    return admittance::Wrench{fg_x, fg_y, fg_z, tg_x, tg_y, tg_z};
  }

  // ========================================================================
  // tare_sensor(): capture residual gravity model error (force AND torque)
  //
  // Samples N wrench readings, subtracts model-predicted gravity at
  // current orientation, averages the residual. This bias is subtracted
  // every control cycle.
  // ========================================================================
  void tare_sensor() {

    int samples = get_parameter("tare_samples").as_int();
    int interval_ms = get_parameter("tare_sample_interval_ms").as_int();

    auto pose = kinova_->getCurrentPose();

    admittance::Wrench sum;
    for (int i = 0; i < samples; i++) {
      admittance::Wrench raw = read_wrench();
      admittance::Wrench gravity = computeGravityWrench(
        pose.theta_x, pose.theta_y, pose.theta_z);

      sum += (raw - gravity);
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }

    bias_ = sum / static_cast<double>(samples);
    filter_.reset();

    RCLCPP_INFO(get_logger(),
      "Tare done (%d samples). Residual bias: F=[%.3f, %.3f, %.3f] N, "
      "T=[%.3f, %.3f, %.3f] Nm",
      samples, bias_.fx, bias_.fy, bias_.fz,
      bias_.tx, bias_.ty, bias_.tz);
  }

  // ========================================================================
  // control_loop(): main real-time control cycle (100Hz)
  //
  // Pipeline:
  //   1.  Read wrench (tool frame, from MAE sensor)
  //   2.  Gravity compensation (model-based, orientation-dependent)
  //   3.  EMA filter (noise smoothing — both force AND torque channels)
  //   4.  Dead zone (reject sensor drift — separate thresholds for F/T)
  //   5.  If disabled → return (watchdog auto-stops arm)
  //   6.  Build rotation matrix R (tool → base) from current Euler angles
  //   7.  Linear: position error + admittance law → linear velocity
  //   8.  Angular: quaternion orientation error + admittance law → angular velocity
  //   9.  Velocity clamp (linear + angular separately)
  //  10.  Send full 6D twist to Kortex via SendTwistCommand (base frame)
  // ========================================================================
  void control_loop() {
    if (!g_running) {
      timer_->cancel();
      return;
    }

    // [TIMING] temporary loop instrumentation — remove once loop rate is fixed
    auto _t0 = std::chrono::steady_clock::now();

    // --- 1. Read raw wrench (tool frame) ---
    admittance::Wrench raw = read_wrench();
    auto _t_wrench = std::chrono::steady_clock::now();

    // --- 2. Model-based gravity compensation ---
    // Pose from background cache thread (~20ms old, <1μs read)
    auto current_pose = read_cached_pose();
    auto _t_pose = std::chrono::steady_clock::now();
    admittance::Wrench gravity = computeGravityWrench(
      current_pose.theta_x, current_pose.theta_y, current_pose.theta_z);

    admittance::Wrench corrected = raw - gravity - bias_;

    // --- 3. EMA filter (all 6 channels) ---
    admittance::Wrench smooth = filter_.update(corrected);

    // --- 4. Dead zone (separate thresholds for force and torque) ---
    double fx = admittance::apply_deadzone(smooth.fx, dead_zone_force_);
    double fy = admittance::apply_deadzone(smooth.fy, dead_zone_force_);
    double fz = admittance::apply_deadzone(smooth.fz, dead_zone_force_);
    double tx = admittance::apply_deadzone(smooth.tx, dead_zone_torque_);
    double ty = admittance::apply_deadzone(smooth.ty, dead_zone_torque_);
    double tz = admittance::apply_deadzone(smooth.tz, dead_zone_torque_);

    // Publish corrected wrench for debugging / visualization
    admittance::Wrench pub_wrench{fx, fy, fz, tx, ty, tz};
    publish_wrench(wrench_pub_, pub_wrench);

    // --- 5. If disabled, do nothing (watchdog auto-stops arm) ---
    if (!enabled_) {
      return;
    }

    // --- 6. Build R_tool_to_base from current Kortex Euler angles ---
    double rx_cur = current_pose.theta_x * DEG2RAD;
    double ry_cur = current_pose.theta_y * DEG2RAD;
    double rz_cur = current_pose.theta_z * DEG2RAD;

    // Kortex ZYX intrinsic Euler: R = Rz · Ry · Rx
    R_tool_to_base_ =
        Eigen::AngleAxisd(rz_cur, Eigen::Vector3d::UnitZ()).toRotationMatrix()
      * Eigen::AngleAxisd(ry_cur, Eigen::Vector3d::UnitY()).toRotationMatrix()
      * Eigen::AngleAxisd(rx_cur, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // =====================================================================
    // 7. LINEAR: position error + force admittance → linear velocity
    // =====================================================================
    double ex = desired_pos_.x - current_pose.x;
    double ey = desired_pos_.y - current_pose.y;
    double ez = desired_pos_.z - current_pose.z;
    double pos_error = std::sqrt(ex*ex + ey*ey + ez*ez);

    // Transform force: tool frame → base frame
    Eigen::Vector3d F_tool(fx, fy, fz);
    Eigen::Vector3d F_base = R_tool_to_base_ * F_tool;

    // Linear admittance law: v = F_base / D  +  Kp · (desired − current)
    double vx = F_base(0) / D_x_ + K_p_ * ex;
    double vy = F_base(1) / D_y_ + K_p_ * ey;
    double vz = F_base(2) / D_z_ + K_p_ * ez;

    // =====================================================================
    // 8. ANGULAR: quaternion orientation error + torque admittance → angular vel
    // =====================================================================

    // Current orientation as quaternion
    Eigen::Quaterniond q_current = euler_deg_to_quaternion(
      current_pose.theta_x, current_pose.theta_y, current_pose.theta_z);

    // Orientation error: rotation from current → desired
    Eigen::Quaterniond q_err = q_desired_ * q_current.inverse();

    // Short-path fix: q and -q represent the same rotation.
    // If w < 0, the angle > 180° — negate to take the shorter path.
    if (q_err.w() < 0.0) {
      q_err.coeffs() *= -1.0;
    }

    // Orientation error as angular velocity correction (base frame)
    // 2 * q_err.vec() ≈ θ_error in radians for small angles,
    // well-behaved for large angles (always points in correction direction)
    Eigen::Vector3d orient_error = 2.0 * q_err.vec();

    // Orientation regulation: ω_correction = Kp_angular · orient_error
    // Per-axis gains allow different stiffness per direction
    double wx_correction = Kp_x_ang_ * orient_error(0);
    double wy_correction = Kp_y_ang_ * orient_error(1);
    double wz_correction = Kp_z_ang_ * orient_error(2);


    // Transform torque: tool frame → base frame
    Eigen::Vector3d T_tool(tx, ty, tz);
    Eigen::Vector3d T_base = R_tool_to_base_ * T_tool;

    // Torque admittance: ω_admittance = T_base / D_angular
    double wx_admit = T_base(0) / D_x_ang_;
    double wy_admit = T_base(1) / D_y_ang_;
    double wz_admit = T_base(2) / D_z_ang_;

    // Total angular velocity = admittance + orientation regulation
    double wx = wx_admit + wx_correction;
    double wy = wy_admit + wy_correction;
    double wz = wz_admit + wz_correction;

    // =====================================================================
    // WIPE OVERRIDE
    // If the wipe_node commands active control, override the LINEAR
    // velocities (X/Y commanded, Z force-controlled or commanded).
    // Angular (orientation hold) is left completely untouched.
    // =====================================================================
    wipe_msgs::msg::WipeSetpoint sp;
    bool have_sp;
    rclcpp::Time sp_time;
    {
      std::lock_guard<std::mutex> lock(setpoint_mutex_);
      sp       = latest_setpoint_;
      have_sp  = setpoint_received_;
      sp_time  = last_setpoint_time_;
    }

    uint8_t mode = wipe_msgs::msg::WipeSetpoint::IDLE;
    if (have_sp) {
      mode = sp.mode;
      // Stale-setpoint safety: brain died / stopped publishing → fall to IDLE.
      double age = (now() - sp_time).seconds();
      if (age > setpoint_timeout_) {
        mode = wipe_msgs::msg::WipeSetpoint::IDLE;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Wipe setpoint stale (%.2fs) — reverting to admittance.", age);
      }
    }

    bool wipe_active = (mode != wipe_msgs::msg::WipeSetpoint::IDLE);

    //start the force controller pi when we are not in the idle mode
    if (wipe_active) {
      // X/Y: commanded directly (position regulation OFF on these axes).
      vx = sp.velocity_x;
      vy = sp.velocity_y;

      // Z: force control or velocity command.
      bool z_fc = sp.z_force_control;
      if (z_fc && !prev_z_force_control_) {
        force_pi_->reset();              // rising edge: bumpless transfer
      }
      prev_z_force_control_ = z_fc;

      if (z_fc) {
        vz = force_pi_->update(sp.force_desired_z, -fz);  // fz is negative during contact; -fz makes it positive to match positive F_desired
      } else {
        vz = sp.velocity_z;              // APPROACH / RETRACT
      }
    } else {
      prev_z_force_control_ = false;     // reset edge tracker when not wiping
    }


    // =====================================================================
    // Combined deadzone: no force/torque + close to home → zero twist
    // =====================================================================
    double orient_error_mag = orient_error.norm();
    bool no_external = (fx == 0.0 && fy == 0.0 && fz == 0.0 &&
                        tx == 0.0 && ty == 0.0 && tz == 0.0);
    // 0.003m = 3mm position deadzone, ~0.5° orientation deadzone
    bool at_home = (pos_error < 0.003) && (orient_error_mag < 0.015);

    if (!wipe_active && no_external && at_home) {
      kinova_->setCartesianVelocity(0, 0, 0, 0, 0, 0);
      return;
    }

    // --- 9. Safety clamp ---
    vx = std::clamp(vx, -max_vel_, max_vel_);
    vy = std::clamp(vy, -max_vel_, max_vel_);
    vz = std::clamp(vz, -max_vel_, max_vel_);

    wx = std::clamp(wx, -max_ang_vel_, max_ang_vel_);
    wy = std::clamp(wy, -max_ang_vel_, max_ang_vel_);
    wz = std::clamp(wz, -max_ang_vel_, max_ang_vel_);

    // small velocities completely zeroed out to avoid randoms small jerk
    if (std::abs(vx) < 1e-4) vx = 0.0;
    if (std::abs(vy) < 1e-4) vy = 0.0;
    if (std::abs(vz) < 1e-4) vz = 0.0;
    if (std::abs(wx) < 1e-4) wx = 0.0;
    if (std::abs(wy) < 1e-4) wy = 0.0;
    if (std::abs(wz) < 1e-4) wz = 0.0;


    // --- 10. Send full 6D twist command (base frame) ---
    // Kortex expects angular velocity in degrees/s
    auto _t_math = std::chrono::steady_clock::now();
    kinova_->setCartesianVelocity(vx, vy, vz,
                               wx * RAD2DEG, wy * RAD2DEG, wz * RAD2DEG);
    auto _t_twist = std::chrono::steady_clock::now();

    // [TIMING] per-section breakdown, throttled to 1 Hz.
    // Whichever number is ~70+ ms is the blocking call gating the loop.
    auto _ms = [](auto a, auto b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[timing] wrench=%.1f pose=%.1f math=%.1f twist=%.1f total=%.1f ms",
      _ms(_t0, _t_wrench), _ms(_t_wrench, _t_pose), _ms(_t_pose, _t_math),
      _ms(_t_math, _t_twist), _ms(_t0, _t_twist));
  }


  // ========================================================================
  // publish_wrench(): publish WrenchStamped for debugging / visualization
  // ========================================================================
  void publish_wrench(
      rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr& pub,
      const admittance::Wrench& w) {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "tool_frame";
    msg.wrench.force.x = w.fx;
    msg.wrench.force.y = w.fy;
    msg.wrench.force.z = w.fz;
    msg.wrench.torque.x = w.tx;
    msg.wrench.torque.y = w.ty;
    msg.wrench.torque.z = w.tz;
    pub->publish(msg);
  }

  // ------------------------------------------------------------------
  // Member variables
  // ------------------------------------------------------------------

  // Degree-to-radian and reverse constant
  static constexpr double DEG2RAD = M_PI / 180.0;
  static constexpr double RAD2DEG = 180.0 / M_PI;

  // Kinova arm interface — owns the Kortex API connection
  std::shared_ptr<kinova_wrapper::KinovaInterface> kinova_;

  // ROS2 communication handles
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::CallbackGroup::SharedPtr io_cb_group_;

  // Wipe integration
  rclcpp::Subscription<wipe_msgs::msg::WipeSetpoint>::SharedPtr setpoint_sub_;
  wipe_msgs::msg::WipeSetpoint latest_setpoint_;
  std::mutex setpoint_mutex_;
  bool setpoint_received_ = false;
  rclcpp::Time last_setpoint_time_;
  bool prev_z_force_control_ = false;

  std::unique_ptr<admittance::ForceControllerPI> force_pi_;
  double force_kp_, force_ki_, setpoint_timeout_;

  // Thread-safe wrench cache (updated by subscriber at 500Hz)
  admittance::Wrench latest_wrench_;
  std::mutex wrench_mutex_;
  bool wrench_received_ = false;

  // Thread-safe pose cache (updated by background thread at ~25Hz)
  // Decouples the ~20ms Kortex getCurrentPose() network round-trip from
  // the control loop. The loop reads a cached copy in <1μs instead of
  // blocking. At 25Hz polling, pose is at most ~40ms old; at max velocity
  // 0.15 m/s the arm moves 6mm in that window — within the 3mm deadzone
  // tolerance for admittance control.
  kinova_wrapper::Pose cached_pose_;
  std::mutex pose_mutex_;
  bool pose_received_ = false;
  std::thread pose_thread_;
  std::atomic<bool> pose_thread_running_{false};

  // Controller state
  admittance::Wrench bias_;                   // residual tare bias (F + T)
  admittance::WrenchFilter filter_{0.1};      // EMA noise filter (all 6 channels)
  kinova_wrapper::Pose desired_pos_;          // spring-back target pose

  // Desired orientation as quaternion (captured at startup)
  Eigen::Quaterniond q_desired_;

  // Frame transformation (Eigen, updated every control cycle)
  Eigen::Matrix3d R_tool_to_base_;            // FK rotation: tool → base

  // Cached parameters — linear
  double D_x_, D_y_, D_z_;                    // linear damping [N·s/m]
  double K_p_;                                // position regulation gain [1/s]

  // Cached parameters — angular
  double D_x_ang_, D_y_ang_, D_z_ang_;        // angular damping [Nm·s/rad]
  double Kp_x_ang_, Kp_y_ang_, Kp_z_ang_;    // orientation regulation gain [1/s]

  // Dead zones
  double dead_zone_force_;                    // force noise threshold [N]
  double dead_zone_torque_;                   // torque noise threshold [Nm]

  // Safety limits
  double max_disp_x_, max_disp_y_, max_disp_z_;  // workspace limits [m]
  double max_vel_;                            // linear velocity clamp [m/s]
  double max_ang_vel_;                        // angular velocity clamp [rad/s]

  bool enabled_ = false;                      // controller active flag

  // Gravity compensation model
  double gripper_mass_;                       // payload mass [kg]
  double cog_x_, cog_y_, cog_z_;             // center of gravity, tool frame [m]
};


// =============================================================================
// main
// =============================================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::signal(SIGINT, signal_handler);
  auto node = std::make_shared<AdmittanceNode>();
  if (!node->init()) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize. Exiting.");
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && g_running) {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  node->cleanup();
  rclcpp::shutdown();
  return 0;
}