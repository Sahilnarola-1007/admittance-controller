// =============================================================================
// Admittance Controller Node — Kinova Gen3
// =============================================================================
// Day 22: Force/Torque Sensing Fundamentals
//
// This node uses KinovaInterface wrapper to access the Kortex API.
// The ros2_kortex driver doesn't expose wrench data, so we go direct.
//
// What it does:
//   1. On startup: reads current EE position → stores as x_desired
//   2. Optionally tares the sensor (for residual model error)
//   3. Every loop cycle:
//      - Reads wrench from KinovaInterface
//      - Computes gravity wrench from current orientation (model-based)
//      - Subtracts gravity + residual bias
//      - Filters (exponential moving average)
//      - Applies dead zone
//      - Computes displacement: dx = F / K
//      - Clamps to safety limits
//      - Sends x_desired + dx as new Cartesian command
//
// Gravity compensation: model-based (works at any arm configuration).
//   F_contact = F_measured - R_world_to_tool * [0, 0, -m*g] - bias_residual
//
// The result: push the EE → it yields. Let go → it springs back.
//
// IMPORTANT: This node takes exclusive control of the arm. Do NOT run
// the ros2_kortex driver simultaneously — they'll fight over the API.
// =============================================================================

#include <chrono>
#include <memory>
#include <atomic>
#include <csignal>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "admittance_controller/types.hpp"
#include "kinova_wrapper/KinovaInterface.hpp"

using namespace std::chrono_literals;

// Global flag for clean shutdown
static std::atomic<bool> g_running{true};
void signal_handler(int) { g_running = false; }

// =============================================================================
// AdmittanceNode class
// =============================================================================
class AdmittanceNode : public rclcpp::Node {
public:
  AdmittanceNode() : Node("admittance_node") {
    // ------------------------------------------------------------------
    // Declare all parameters (loaded from YAML or set via command line)
    // ------------------------------------------------------------------
    declare_parameter("robot_ip", "192.168.1.10");

    declare_parameter("loop_rate_hz", 100);

    declare_parameter("stiffness_x", 200.0);
    declare_parameter("stiffness_y", 200.0);
    declare_parameter("stiffness_z", 200.0);

    declare_parameter("dead_zone_force", 1.5);
    declare_parameter("dead_zone_torque", 0.5);

    declare_parameter("max_displacement_x", 0.05);
    declare_parameter("max_displacement_y", 0.05);
    declare_parameter("max_displacement_z", 0.05);
    declare_parameter("max_velocity", 0.1);

    declare_parameter("filter_alpha", 0.1);

    declare_parameter("tare_samples", 100);
    declare_parameter("tare_sample_interval_ms", 10);

    // Gravity compensation model parameters
    // Robotiq 2F-85: mass ≈ 0.93 kg, CoG ≈ 58mm along tool z-axis
    declare_parameter("gripper_mass", 0.93);
    declare_parameter("cog_x", 0.0);      // meters, in tool frame
    declare_parameter("cog_y", 0.0);
    declare_parameter("cog_z", 0.058);

    load_params();

    // ------------------------------------------------------------------
    // Publishers: corrected wrench for debugging/visualization
    // ------------------------------------------------------------------
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/wrench_corrected", 10);

    raw_wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/wrench_raw", 10);

    // ------------------------------------------------------------------
    // Services: tare and enable/disable
    // ------------------------------------------------------------------
    tare_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/tare",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        RCLCPP_INFO(get_logger(), "Tare requested — hold the arm still...");
        tare_sensor();
        response->success = true;
        response->message = "Sensor tared successfully.";
      });

    enable_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/enable",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        enabled_ = true;
        response->success = true;
        response->message = "Admittance control enabled.";
        RCLCPP_INFO(get_logger(), "Admittance control ENABLED");
      });

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
    // Dynamic parameter reconfiguration callback for live tuning
    // ------------------------------------------------------------------
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params)
        -> rcl_interfaces::msg::SetParametersResult {
        for (const auto& p : params) {
          if (p.get_name() == "stiffness_x") K_x_ = p.as_double();
          else if (p.get_name() == "stiffness_y") K_y_ = p.as_double();
          else if (p.get_name() == "stiffness_z") K_z_ = p.as_double();
          else if (p.get_name() == "dead_zone_force") dead_zone_force_ = p.as_double();
          else if (p.get_name() == "filter_alpha") filter_.set_alpha(p.as_double());
          else if (p.get_name() == "max_displacement_x") max_disp_x_ = p.as_double();
          else if (p.get_name() == "max_displacement_y") max_disp_y_ = p.as_double();
          else if (p.get_name() == "max_displacement_z") max_disp_z_ = p.as_double();
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
  // init(): connect via KinovaInterface, read initial pose, tare, start loop
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

    // --- Read current EE position → this becomes x_desired ---
    desired_pos_ = kinova_->getCurrentPose();
    RCLCPP_INFO(get_logger(),
      "Initial EE position: x=%.4f, y=%.4f, z=%.4f",
      desired_pos_.x, desired_pos_.y, desired_pos_.z);

    // --- Tare the sensor on intitial position (captures residual model error) ---
    RCLCPP_INFO(get_logger(), "Taring sensor — keep the arm still...");
    tare_sensor();
    RCLCPP_INFO(get_logger(),
      "Tare complete. Residual bias: fx=%.3f, fy=%.3f, fz=%.3f",
      bias_.fx, bias_.fy, bias_.fz);

    // --- Start the control loop timer ---
    int rate = get_parameter("loop_rate_hz").as_int();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      [this]() { control_loop(); });

    enabled_ = true;
    RCLCPP_INFO(get_logger(),
      "Admittance controller running at %d Hz. Push the EE!", rate);
    RCLCPP_INFO(get_logger(),
      "  Stiffness: Kx=%.0f  Ky=%.0f  Kz=%.0f", K_x_, K_y_, K_z_);
    RCLCPP_INFO(get_logger(),
      "  Dead zone: %.1f N  |  Max displacement: %.0f mm",
      dead_zone_force_, max_disp_x_ * 1000);
    RCLCPP_INFO(get_logger(),
      "  Gripper mass: %.3f kg  |  CoG: [%.3f, %.3f, %.3f] m",
      gripper_mass_, cog_x_, cog_y_, cog_z_);
    RCLCPP_INFO(get_logger(), "Services: ~/tare  ~/enable  ~/disable");
    RCLCPP_INFO(get_logger(), "Topics:   ~/wrench_corrected  ~/wrench_raw");

    return true;
  }

  // ========================================================================
  // cleanup(): called on shutdown
  // ========================================================================
  void cleanup() {
    if (timer_) timer_->cancel();
    RCLCPP_INFO(get_logger(), "Shutting down...");
    if (kinova_) {
      kinova_->disconnect();
    }
    RCLCPP_INFO(get_logger(), "Disconnected from Kinova Gen3.");
  }

private:
  // ========================================================================
  // load_params(): read all parameters into member variables
  // ========================================================================
  void load_params() {
    K_x_ = get_parameter("stiffness_x").as_double();
    K_y_ = get_parameter("stiffness_y").as_double();
    K_z_ = get_parameter("stiffness_z").as_double();

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
  // read_wrench(): get current wrench from KinovaInterface
  // ========================================================================
  admittance::Wrench read_wrench() {
    auto w_vec = kinova_->getWrench();

    admittance::Wrench w;
    if (w_vec.size() == 6) {
      w.fx = w_vec[0];
      w.fy = w_vec[1];
      w.fz = w_vec[2];
      w.tx = w_vec[3];
      w.ty = w_vec[4];
      w.tz = w_vec[5];
    }
    return w;
  }

  // ========================================================================
  // computeGravityWrench(): model-based gravity compensation
  // ========================================================================
  // Given the current tool orientation (Euler angles from Kortex),
  // compute the expected gravity wrench in the tool frame.
  //
  // This replaces fixed taring — works at ANY arm configuration.
  //
  // Steps:
  //   1. Build rotation matrix R_tool_to_world from Euler angles (ZYX order)
  //   2. Transpose it → R_world_to_tool
  //   3. Transform gravity vector [0, 0, -m*g] from world to tool frame
  //   4. Compute torque from lever arm: T = r_cog × F_gravity_tool
  //
  // Kortex Euler convention: ZYX intrinsic (Rz * Ry * Rx), angles in degrees.
  // ========================================================================
  admittance::Wrench computeGravityWrench(double theta_x_deg,
                                           double theta_y_deg,
                                           double theta_z_deg) {
    // Convert degrees to radians (Kortex gives orientation in degrees)
    constexpr double deg2rad = M_PI / 180.0;
    double rx = theta_x_deg * deg2rad;
    double ry = theta_y_deg * deg2rad;
    double rz = theta_z_deg * deg2rad;

    // Precompute sin/cos
    double cx = std::cos(rx), sx = std::sin(rx);
    double cy = std::cos(ry), sy = std::sin(ry);
    double cz = std::cos(rz), sz = std::sin(rz);

    // Build R_tool_to_world (ZYX intrinsic = Rz * Ry * Rx)
    //
    // R = | cz*cy   cz*sy*sx - sz*cx   cz*sy*cx + sz*sx |
    //     | sz*cy   sz*sy*sx + cz*cx   sz*sy*cx - cz*sx |
    //     | -sy     cy*sx              cy*cx             |
    //
    // We only need R_transpose (= R_world_to_tool) applied to [0, 0, -m*g]
    // which means we only need column 2 of R (the third column dotted with
    // each row of R_transpose = each column of R).
    //
    // Actually, R^T * [0, 0, F] = F * [R[2][0], R[2][1], R[2][2]]
    // (third row of R, since R^T columns = R rows)
    //
    // Gravity in world: [0, 0, -m*g]
    // F_tool = R^T * [0, 0, -m*g] = -m*g * [R[2][0], R[2][1], R[2][2]]
    //        = -m*g * [-sy, cy*sx, cy*cx]

    double mg = gripper_mass_ * 9.81;

    // Gravity force in tool frame
    double fg_x = -mg * (-sy);          // = mg * sy
    double fg_y = -mg * (cy * sx);
    double fg_z = -mg * (cy * cx);

    // Torque from lever arm: T = r_cog × F_gravity_tool
    // r_cog = [cog_x_, cog_y_, cog_z_] (constant, measured once)
    //
    // Cross product:
    //   Tx = cog_y * fg_z - cog_z * fg_y
    //   Ty = cog_z * fg_x - cog_x * fg_z
    //   Tz = cog_x * fg_y - cog_y * fg_x
    double tg_x = cog_y_ * fg_z - cog_z_ * fg_y;
    double tg_y = cog_z_ * fg_x - cog_x_ * fg_z;
    double tg_z = cog_x_ * fg_y - cog_y_ * fg_x;

    return admittance::Wrench{fg_x, fg_y, fg_z, tg_x, tg_y, tg_z};
  }

  // ========================================================================
  // tare_sensor(): sample N wrench readings, subtract model gravity,
  // average the residual → bias_. This captures model inaccuracies
  // (imprecise mass, CoG offsets, joint friction, etc.)
  // ========================================================================
  void tare_sensor() {
    int samples = get_parameter("tare_samples").as_int();
    int interval_ms = get_parameter("tare_sample_interval_ms").as_int();

    // Get current orientation for gravity model during tare
    auto pose = kinova_->getCurrentPose();

    admittance::Wrench sum;
    for (int i = 0; i < samples; i++) {
      admittance::Wrench raw = read_wrench();
      admittance::Wrench gravity = computeGravityWrench(
        pose.theta_x, pose.theta_y, pose.theta_z);

      // Residual = measured - model_gravity (should be near zero if model is good)
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
  // control_loop(): THE MAIN LOOP — runs every cycle
  // ========================================================================
  void control_loop() {
    if (!g_running) {
      timer_->cancel();
      return;
    }

    // --- 1. Read raw wrench ---
    admittance::Wrench raw = read_wrench();
    publish_wrench(raw_wrench_pub_, raw);

    // --- 2. Model-based gravity compensation ---
    // Get current orientation (updates every cycle → works at any config)
    auto current_pose = kinova_->getCurrentPose();
    admittance::Wrench gravity = computeGravityWrench(
      current_pose.theta_x, current_pose.theta_y, current_pose.theta_z);

    // Subtract model gravity AND residual bias
    admittance::Wrench corrected = raw - gravity - bias_;

    // --- 3. Filter ---
    admittance::Wrench smooth = filter_.update(corrected);

    // --- 4. Apply dead zone ---
    double fx = admittance::apply_deadzone(smooth.fx, dead_zone_force_);
    double fy = admittance::apply_deadzone(smooth.fy, dead_zone_force_);
    double fz = admittance::apply_deadzone(smooth.fz, dead_zone_force_);

    // Publish corrected wrench (after gravity comp + filter + deadzone)
    admittance::Wrench pub_wrench{fx, fy, fz, 0, 0, 0};
    publish_wrench(wrench_pub_, pub_wrench);

    // --- 5. If disabled, just hold position ---
    if (!enabled_) {
      send_cartesian_pose(desired_pos_);
      return;
    }

    // --- 6. Admittance: F / K = displacement ---
    double dx = fx / K_x_;
    double dy = fy / K_y_;
    double dz = fz / K_z_;

    // --- 7. Clamp displacement to safety limits ---
    dx = std::clamp(dx, -max_disp_x_, max_disp_x_);
    dy = std::clamp(dy, -max_disp_y_, max_disp_y_);
    dz = std::clamp(dz, -max_disp_z_, max_disp_z_);

    // --- 8. Rate-limit velocity (smooth transitions) ---
    int rate = get_parameter("loop_rate_hz").as_int();
    double dt = 1.0 / static_cast<double>(rate);
    auto limit_rate = [&](double target, double prev, double max_v) {
      double delta = target - prev;
      double max_delta = max_v * dt;
      return prev + std::clamp(delta, -max_delta, max_delta);
    };

    current_offset_.x = limit_rate(dx, current_offset_.x, max_vel_);
    current_offset_.y = limit_rate(dy, current_offset_.y, max_vel_);
    current_offset_.z = limit_rate(dz, current_offset_.z, max_vel_);

    // --- 9. Compute commanded position ---
    kinova_wrapper::Pose cmd = desired_pos_;
    cmd.x += current_offset_.x;
    cmd.y += current_offset_.y;
    cmd.z += current_offset_.z;
    // Orientation stays as desired_pos_ (unchanged)

    // --- 10. Send to Kinova ---
    send_cartesian_pose(cmd);
  }

  // ========================================================================
  // send_cartesian_pose(): send a Cartesian position via KinovaInterface
  // ========================================================================
  void send_cartesian_pose(const kinova_wrapper::Pose& pose) {
    // NOTE: moveToCartesianPose() is blocking — this is fine for high-level
    // servoing but adds latency. For tighter control, switch to
    // setCartesianVelocity() in a future version.
    if (!kinova_->moveToCartesianPose(pose)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "Cartesian pose command failed");
    }
  }

  // ========================================================================
  // publish_wrench(): publish a WrenchStamped message
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

  // KinovaInterface — owns the connection to the robot
  std::shared_ptr<kinova_wrapper::KinovaInterface> kinova_;

  // ROS2 pub/sub/srv
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr raw_wrench_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // Admittance state
  admittance::Wrench bias_;                   // residual bias (model error)
  admittance::WrenchFilter filter_{0.1};
  kinova_wrapper::Pose desired_pos_;          // where you WANT the EE
  admittance::CartesianPos current_offset_;   // current displacement from desired

  // Parameters (cached for fast access in control loop)
  double K_x_, K_y_, K_z_;
  double dead_zone_force_;
  double max_disp_x_, max_disp_y_, max_disp_z_;
  double max_vel_;
  bool enabled_ = false;

  // Gravity compensation model constants
  double gripper_mass_;
  double cog_x_, cog_y_, cog_z_;   // center of gravity in tool frame (meters)
};


// =============================================================================
// main()
// =============================================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::signal(SIGINT, signal_handler);

  auto node = std::make_shared<AdmittanceNode>();

  // Initialize Kortex connection
  if (!node->init()) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize. Exiting.");
    return 1;
  }

  // Spin until shutdown
  while (rclcpp::ok() && g_running) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(1ms);
  }

  node->cleanup();
  rclcpp::shutdown();
  return 0;
}
