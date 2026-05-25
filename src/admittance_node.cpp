// =============================================================================
// Admittance Controller Node — Kinova Gen3 + MAE SensuReal F/T Sensor
// =============================================================================
//
// Velocity-based admittance controller that maps external forces to compliant
// end-effector motion. Designed for force-controlled contact tasks such as
// surface wiping, polishing, and guided manipulation.
//
// Architecture:
//   MAE SensuReal F/T sensor (UDP, 1kHz)
//     → mae_sensor_node (Python lifecycle driver, publishes /wrench_raw at 500Hz)
//       → this node (subscribes, compensates gravity, filters, controls)
//         → Kortex SendTwistCommand (streaming Cartesian velocity at 100Hz)
//
// Control law (velocity-based admittance):
//
//   v_base = R_tool_to_base · (F_tool / D) + Kp · (x_desired − x_current)
//            ├─ compliance term ─┘              └─ position regulation ─┤
//
//   where:
//     F_tool    Gravity-compensated, filtered, dead-zoned force (tool frame)
//     D         Damping gain [N·s/m] — controls compliance (higher = stiffer)
//     Kp        Position regulation gain [1/s] — drives spring-back to x_desired
//     R         FK rotation matrix (tool → base), updated every cycle via Eigen
//     x_desired Captured at startup — the "home" position for spring-back
//
//   At steady state with constant force F:
//     Equilibrium displacement = F / (D × Kp)
//     e.g. 5N with D=150, Kp=2 → 16.7mm displacement
//
// Gravity compensation (model-based, orientation-independent):
//   F_contact = F_measured − R^T · [0, 0, −m·g] − bias_residual
//   R updates every cycle from Kortex Euler angles → works at any configuration.
//   Tare captures residual model error (imprecise mass, CoG, friction).
//
// Frame conventions:
//   - Sensor frame: MAE SensuReal native axes
//   - Tool frame:   Sensor readings rotated by Rz(90°) in mae_sensor_node.py
//   - Base frame:   Kortex world frame, used for SendTwistCommand
//   - R_tool_to_base built from Kortex ZYX intrinsic Euler angles using Eigen
//
// End-effector payload:
//   Currently configured for a lightweight test handle (~175g) mounted
//   directly on the F/T sensor flange for controller validation. No gripper
//   attached. Sequence: robot → flange → F/T sensor → test handle.
//   When Robotiq 2F-85 is installed, update gravity model parameters:
//     gripper_mass: 0.93 kg, cog_z: 0.058 m
//
// Safety:
//   - Velocity clamped to max_velocity (default 0.15 m/s)
//   - Dead zone (0.3N) rejects sensor noise and thermal drift
//   - Position deadzone (3mm) prevents oscillation near desired pose
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
//
// Parameters (all dynamically reconfigurable via `ros2 param set`):
//   damping_x/y/z        Damping D [N·s/m]       (higher = stiffer)
//   position_gain        Spring-back Kp [1/s]    (higher = faster return)
//   dead_zone_force      Noise threshold [N]
//   max_velocity         Safety clamp [m/s]
//   filter_alpha         EMA coefficient [0–1]   (lower = smoother)
//   gripper_mass         Payload mass [kg]
//   cog_x/y/z            Center of gravity, tool frame [m]
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
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "admittance_controller/types.hpp"
#include "kinova_wrapper/KinovaInterface.hpp"

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
    //   ros2 param set /admittance_node stiffness_x 300.0
    // ------------------------------------------------------------------
    declare_parameter("robot_ip", "192.168.1.10");

    declare_parameter("loop_rate_hz", 100);

    // Damping gains D [N·s/m]: force required to produce 1 m/s velocity
    declare_parameter("damping_x", 150.0);
    declare_parameter("damping_y", 150.0);
    declare_parameter("damping_z", 150.0);

    // Position regulation gain Kp [1/s]: controls spring-back speed.
    // Equilibrium displacement = F / (D × Kp)
    declare_parameter("position_gain", 2.0);

    // Dead zone: forces below this are zeroed (MAE drift envelope: ~0.1–0.3N)
    declare_parameter("dead_zone_force", 0.3);
    declare_parameter("dead_zone_torque", 0.5);

    // Safety limits
    declare_parameter("max_displacement_x", 0.05);
    declare_parameter("max_displacement_y", 0.05);
    declare_parameter("max_displacement_z", 0.05);
    declare_parameter("max_velocity", 0.15);

    // EMA filter coefficient: 0.1 = heavy smoothing, 1.0 = no filtering
    declare_parameter("filter_alpha", 0.1);

    // Tare configuration
    declare_parameter("tare_samples", 100);
    declare_parameter("tare_sample_interval_ms", 10);

    // Gravity model — end-effector payload
    // Current: test handle (~175g) on F/T sensor flange, no gripper
    // With Robotiq 2F-85: mass=0.93, cog_z=0.058
    declare_parameter("gripper_mass", 0.175);
    declare_parameter("cog_x", 0.0);
    declare_parameter("cog_y", 0.0);
    declare_parameter("cog_z", 0.0);

    load_params();

    // ------------------------------------------------------------------
    // Publisher: corrected wrench for debugging and visualization
    // Published AFTER gravity comp + EMA filter + dead zone
    // ------------------------------------------------------------------
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/wrench_corrected", 10);

    // ------------------------------------------------------------------
    // Subscriber: raw wrench from MAE sensor driver (tool frame, 500Hz)
    //
    // Caches the latest reading behind a mutex. The 100Hz control loop
    // reads the most recent value each cycle — intermediate readings
    // are overwritten (5:1 oversampling ensures fresh data every cycle).
    // ------------------------------------------------------------------
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench_raw", rclcpp::SensorDataQoS(),
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
      });

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
          if (p.get_name() == "damping_x") D_x_ = p.as_double();
          else if (p.get_name() == "damping_y") D_y_ = p.as_double();
          else if (p.get_name() == "damping_z") D_z_ = p.as_double();
          else if (p.get_name() == "dead_zone_force") dead_zone_force_ = p.as_double();
          else if (p.get_name() == "filter_alpha") filter_.set_alpha(p.as_double());
          else if (p.get_name() == "max_displacement_x") max_disp_x_ = p.as_double();
          else if (p.get_name() == "max_displacement_y") max_disp_y_ = p.as_double();
          else if (p.get_name() == "max_displacement_z") max_disp_z_ = p.as_double();
          else if (p.get_name() == "gripper_mass") gripper_mass_ = p.as_double();
          else if (p.get_name() == "cog_x") cog_x_ = p.as_double();
          else if (p.get_name() == "cog_y") cog_y_ = p.as_double();
          else if (p.get_name() == "cog_z") cog_z_ = p.as_double();
          else if (p.get_name() == "position_gain") K_p_ = p.as_double();
          RCLCPP_INFO(get_logger(), "Parameter updated: %s", p.get_name().c_str());
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    RCLCPP_INFO(get_logger(), "AdmittanceNode created. Call init() to connect.");
  }

  // ========================================================================
  // init(): connect to arm, capture desired pose, tare sensor, start loop
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

    // Capture startup pose as the desired (home) position.
    // The arm springs back to this pose when external force is removed.
    desired_pos_ = kinova_->getCurrentPose();
    RCLCPP_INFO(get_logger(),
      "Initial EE position: x=%.4f, y=%.4f, z=%.4f",
      desired_pos_.x, desired_pos_.y, desired_pos_.z);

    // Tare at startup to capture residual gravity model error
    RCLCPP_INFO(get_logger(), "Taring sensor — keep the arm still...");
    tare_sensor();
    RCLCPP_INFO(get_logger(),
      "Tare complete. Residual bias: fx=%.3f, fy=%.3f, fz=%.3f",
      bias_.fx, bias_.fy, bias_.fz);

    // Start the control loop timer
    int rate = get_parameter("loop_rate_hz").as_int();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      [this]() { control_loop(); });

    enabled_ = false;
    RCLCPP_INFO(get_logger(),
      "Admittance controller running at %d Hz. Push the EE!", rate);
    RCLCPP_INFO(get_logger(),
      "  Damping: Dx=%.0f  Dy=%.0f  Dz=%.0f  |  Kp=%.1f", D_x_, D_y_, D_z_, K_p_);
    RCLCPP_INFO(get_logger(),
      "  Dead zone: %.1f N  |  Max velocity: %.2f m/s",
      dead_zone_force_, max_vel_);
    RCLCPP_INFO(get_logger(),
      "  Payload mass: %.3f kg  |  CoG: [%.3f, %.3f, %.3f] m",
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
    RCLCPP_INFO(get_logger(), "Shutting down...");
    if (kinova_) {
      kinova_->stopMotion();
      kinova_->disconnect();
    }
    RCLCPP_INFO(get_logger(), "Disconnected from Kinova Gen3.");
  }

private:
  // ========================================================================
  // load_params(): cache ROS parameters into member variables
  // Avoids parameter server lookups in the 100Hz control loop.
  // ========================================================================
  void load_params() {
    D_x_ = get_parameter("damping_x").as_double();
    D_y_ = get_parameter("damping_y").as_double();
    D_z_ = get_parameter("damping_z").as_double();
    K_p_ = get_parameter("position_gain").as_double();
    dead_zone_force_ = get_parameter("dead_zone_force").as_double();

    max_disp_x_ = get_parameter("max_displacement_x").as_double();
    max_disp_y_ = get_parameter("max_displacement_y").as_double();
    max_disp_z_ = get_parameter("max_displacement_z").as_double();
    max_vel_ = get_parameter("max_velocity").as_double();

    filter_.set_alpha(get_parameter("filter_alpha").as_double());

    gripper_mass_ = get_parameter("gripper_mass").as_double();
    cog_x_ = get_parameter("cog_x").as_double();
    cog_y_ = get_parameter("cog_y").as_double();
    cog_z_ = get_parameter("cog_z").as_double();
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
                                           double theta_z_deg) {
    constexpr double deg2rad = M_PI / 180.0;
    double rx = theta_x_deg * deg2rad;
    double ry = theta_y_deg * deg2rad;
    double rz = theta_z_deg * deg2rad;

    double cx = std::cos(rx), sx = std::sin(rx);
    double cy = std::cos(ry), sy = std::sin(ry);
    double cz = std::cos(rz), sz = std::sin(rz);

    // Only the third row of R is needed for gravity projection:
    //   R[2] = [−sy, cy·sx, cy·cx]
    //   F_tool = R^T · [0, 0, −mg] = −mg · R[2]

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
  // tare_sensor(): capture residual gravity model error
  //
  // Samples N wrench readings, subtracts model-predicted gravity at
  // current orientation, averages the residual. This bias is subtracted
  // every control cycle.
  //
  // Note: single-threaded executor — all N samples read the same cached
  // wrench (subscriber blocked during sleep). Still valid as it captures
  // the steady-state offset at the current pose.
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
      "Tare done (%d samples). Residual bias: [%.3f, %.3f, %.3f] N",
      samples, bias_.fx, bias_.fy, bias_.fz);
  }

  // ========================================================================
  // control_loop(): main real-time control cycle (100Hz)
  //
  // Pipeline:
  //   1. Read wrench (tool frame, from MAE sensor)
  //   2. Gravity compensation (model-based, orientation-dependent)
  //   3. EMA filter (noise smoothing)
  //   4. Dead zone (reject sensor drift)
  //   5. If disabled → return (watchdog auto-stops arm)
  //   6. Position error + deadzone check (base frame)
  //   7. Frame transformation: F_tool → F_base via Eigen rotation matrix
  //   8. Admittance law: v = F_base/D + Kp·(desired − current)
  //   9. Velocity clamp (safety)
  //  10. Send to Kortex via SendTwistCommand (base frame)
  // ========================================================================
  void control_loop() {
    if (!g_running) {
      timer_->cancel();
      return;
    }

    // --- 1. Read raw wrench (tool frame) ---
    admittance::Wrench raw = read_wrench();

    // --- 2. Model-based gravity compensation ---
    auto current_pose = kinova_->getCurrentPose();
    admittance::Wrench gravity = computeGravityWrench(
      current_pose.theta_x, current_pose.theta_y, current_pose.theta_z);

    admittance::Wrench corrected = raw - gravity - bias_;

    // --- 3. EMA filter ---
    admittance::Wrench smooth = filter_.update(corrected);

    // --- 4. Dead zone ---
    double fx = admittance::apply_deadzone(smooth.fx, dead_zone_force_);
    double fy = admittance::apply_deadzone(smooth.fy, dead_zone_force_);
    double fz = admittance::apply_deadzone(smooth.fz, dead_zone_force_);

    // Publish corrected wrench for debugging / visualization
    admittance::Wrench pub_wrench{fx, fy, fz, 0, 0, 0};
    publish_wrench(wrench_pub_, pub_wrench);

    // --- 5. If disabled, do nothing (watchdog auto-stops arm) ---
    if (!enabled_) {
      return;
    }

    // --- 6. Position error (base frame) ---
    double ex = desired_pos_.x - current_pose.x;
    double ey = desired_pos_.y - current_pose.y;
    double ez = desired_pos_.z - current_pose.z;
    double pos_error = std::sqrt(ex*ex + ey*ey + ez*ez);

    // Position deadzone: no force + close to home → zero velocity
    if (fx == 0.0 && fy == 0.0 && fz == 0.0 && pos_error < 0.003) {
        kinova_->setCartesianVelocity(0, 0, 0, 0, 0, 0);
        return;
    }

    // --- 7. Frame transformation: tool → base ---
    // Build R_tool_to_base from Kortex ZYX Euler angles using Eigen.
    // Updates every cycle → compliance direction correct at any configuration.
    constexpr double deg2rad = M_PI / 180.0;
    double rx = current_pose.theta_x * deg2rad;
    double ry = current_pose.theta_y * deg2rad;
    double rz = current_pose.theta_z * deg2rad;

    Eigen::Matrix3d Rx = Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Kortex ZYX intrinsic Euler: R = Rz · Ry · Rx
    R_tool_to_base_ = Rz * Ry * Rx;

    // Transform force: tool frame → base frame
    F_tool<<fx, fy, fz;
    F_base = R_tool_to_base_ * F_tool;

    // --- 8. Admittance law ---
    // v = F_base / D  +  Kp · (desired − current)
    //     ↑ compliance     ↑ position regulation (spring-back)
    double vx = F_base(0) / D_x_ + K_p_ * ex;
    double vy = F_base(1) / D_y_ + K_p_ * ey;
    double vz = F_base(2) / D_z_ + K_p_ * ez;
    

    // --- 9. Safety clamp ---
    vx = std::clamp(vx, -max_vel_, max_vel_);
    vy = std::clamp(vy, -max_vel_, max_vel_);
    vz = std::clamp(vz, -max_vel_, max_vel_);

    

    // --- 10. Send velocity command (base frame) ---
    kinova_->setCartesianVelocity(vx,vy,vz,0,0,0);
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

  // Thread-safe wrench cache (updated by subscriber at 500Hz)
  admittance::Wrench latest_wrench_;
  std::mutex wrench_mutex_;
  bool wrench_received_ = false;

  // Controller state
  admittance::Wrench bias_;                   // residual tare bias
  admittance::WrenchFilter filter_{0.1};      // EMA noise filter
  kinova_wrapper::Pose desired_pos_;          // spring-back target pose

  // Frame transformation (Eigen, updated every control cycle)
  Eigen::Matrix3d R_tool_to_base_;            // FK rotation: tool → base
  Eigen::Vector3d F_tool;                     // force vector in tool frame
  Eigen::Vector3d F_base;                     // force vector in base frame
  
  // Cached parameters (avoid ROS param lookups in 100Hz loop)
  double D_x_, D_y_, D_z_;                    // damping gains [N·s/m]
  double K_p_;                                // position regulation gain [1/s]
  double dead_zone_force_;                    // noise threshold [N]
  double max_disp_x_, max_disp_y_, max_disp_z_;  // workspace limits [m]
  double max_vel_;                            // velocity clamp [m/s]
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

  // Single-threaded executor: spin_some() drains all ready callbacks
  // (timer, subscriber, service) sequentially each iteration.
  while (rclcpp::ok() && g_running) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(1ms);
  }

  node->cleanup();
  rclcpp::shutdown();
  return 0;
}
