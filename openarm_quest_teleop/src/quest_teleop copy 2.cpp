/*******************************************************************************
 * Title     : quest_teleop.cpp
 * Project   : openarm_quest_teleop
 * Purpose   : Quest VR Teleoperation using MoveIt Servo C++ API
 *             Step-by-step implementation based on demo_quest_sim.cpp
 *******************************************************************************/

#include <chrono>
#include <cmath>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

// MoveIt Servo includes (Step 5)
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <moveit/utils/logger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <map>

// Socket includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

// JSON parsing
#include <nlohmann/json.hpp>

#include <thread>
#include <mutex>
#include <atomic>

using json = nlohmann::json;
using namespace moveit_servo;

namespace {

std::vector<std::string> filterNodeNameRemaps(const std::vector<std::string>& args) {
  std::vector<std::string> filtered;
  filtered.reserve(args.size());
  for (size_t i = 0; i < args.size(); ++i) {
    const std::string& arg = args[i];
    if ((arg == "-r" || arg == "--remap") && (i + 1) < args.size()) {
      const std::string& rule = args[i + 1];
      if (rule.rfind("__node:=", 0) == 0 || rule.rfind("__name:=", 0) == 0) {
        ++i;
        continue;
      }
    }
    if (arg.rfind("__node:=", 0) == 0 || arg.rfind("__name:=", 0) == 0) {
      continue;
    }
    filtered.push_back(arg);
  }
  return filtered;
}

}  // namespace

//==============================================================================
// Step 1: Quest Data Structure & Socket Server
//==============================================================================

struct ControllerData {
  Eigen::Vector3d position{0, 0, 0};
  Eigen::Quaterniond rotation{1, 0, 0, 0};  // w, x, y, z
  double trigger{0.0};
  bool enabled{false};
};

struct QuestData {
  ControllerData left;
  ControllerData right;
  double timestamp{0.0};
};

//==============================================================================
// Step 2: Coordinate Transformation (Quest -> Robot)
//==============================================================================

// Transform Quest controller data to robot frame
// Quest coordinate: Y-up, Robot coordinate: Z-up
// Mapping: Quest (x, y, z) -> Robot (y, x, z)
ControllerData transformToRobotFrame(const ControllerData& quest_data) {
  ControllerData robot_data;
  robot_data.enabled = quest_data.enabled;
  robot_data.trigger = quest_data.trigger;

  // Position: swap X and Y
  robot_data.position.x() = quest_data.position.y();
  robot_data.position.y() = quest_data.position.x();
  robot_data.position.z() = quest_data.position.z();

  // Quaternion: swap X and Y components
  // Quest quat (x, y, z, w) -> Robot quat (y, x, z, w)
  double qx = quest_data.rotation.x();
  double qy = quest_data.rotation.y();
  double qz = quest_data.rotation.z();
  double qw = quest_data.rotation.w();
  robot_data.rotation = Eigen::Quaterniond(qw, qy, qx, qz);  // w, x, y, z order

  return robot_data;
}

// Transform both controllers
QuestData transformQuestData(const QuestData& quest_raw) {
  QuestData robot_data;
  robot_data.timestamp = quest_raw.timestamp;
  robot_data.left = transformToRobotFrame(quest_raw.left);
  robot_data.right = transformToRobotFrame(quest_raw.right);
  return robot_data;
}

QuestData simulateQuestData(double t) {
  QuestData data;
  data.timestamp = t;

  const double radius = 0.001;  // 5 cm
  const double freq = 0.5;     // Hz

  data.left.enabled = true;
  data.left.position.x() = radius * std::sin(2 * M_PI * freq * t);
  data.left.position.y() = radius * std::cos(2 * M_PI * freq * t);
  data.left.position.z() = 0.02 * std::sin(2 * M_PI * freq * 2 * t);
  const double angle = 0.3 * std::sin(2 * M_PI * freq * 0.5 * t);
  data.left.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
  data.left.trigger = 0.0;

  data.right.enabled = false;
  return data;
}

//==============================================================================
// Step 3: Calibration Logic
//==============================================================================

// Collects samples for 1 second, then outputs averaged position/rotation
// to be used as the initial "prev" frame for velocity calculation
class Calibrator {
public:
  Calibrator(double duration_sec = 1.0) : duration_(duration_sec) {
    reset();
  }

  void reset() {
    calibrated_ = false;
    start_time_ = -1.0;
    position_samples_.clear();
    rotation_samples_.clear();
  }

  // Call every frame while button is pressed
  // Returns true when calibration completes
  bool update(const ControllerData& data, double current_time) {
    if (calibrated_) return true;

    // First sample - record start time
    if (start_time_ < 0.0) {
      start_time_ = current_time;
    }

    // Collect samples
    position_samples_.push_back(data.position);
    rotation_samples_.push_back(data.rotation);

    // Check if 1 second elapsed
    if (current_time - start_time_ >= duration_) {
      // Calculate averages
      avg_position_ = Eigen::Vector3d::Zero();
      for (const auto& pos : position_samples_) {
        avg_position_ += pos;
      }
      avg_position_ /= static_cast<double>(position_samples_.size());

      // Average quaternion (simple mean + normalize)
      Eigen::Vector4d quat_sum = Eigen::Vector4d::Zero();
      for (const auto& rot : rotation_samples_) {
        quat_sum += Eigen::Vector4d(rot.w(), rot.x(), rot.y(), rot.z());
      }
      quat_sum.normalize();
      avg_rotation_ = Eigen::Quaterniond(quat_sum(0), quat_sum(1), quat_sum(2), quat_sum(3));

      calibrated_ = true;
      return true;
    }

    return false;
  }

  bool isCalibrated() const { return calibrated_; }
  size_t getSampleCount() const { return position_samples_.size(); }

  // Get the averaged values to use as initial "prev" frame
  Eigen::Vector3d getAvgPosition() const { return avg_position_; }
  Eigen::Quaterniond getAvgRotation() const { return avg_rotation_; }

private:
  double duration_;
  double start_time_;
  bool calibrated_;
  std::vector<Eigen::Vector3d> position_samples_;
  std::vector<Eigen::Quaterniond> rotation_samples_;
  Eigen::Vector3d avg_position_;
  Eigen::Quaterniond avg_rotation_;
};

//==============================================================================
// Step 4: Position/Rotation -> Velocity Conversion
//==============================================================================

// Calculate velocity from position/rotation difference
// Same logic as demo_quest_sim.cpp and quest_servo_both.py
void calculateVelocity(
    const Eigen::Vector3d& prev_pos, const Eigen::Quaterniond& prev_rot, double prev_time,
    const Eigen::Vector3d& curr_pos, const Eigen::Quaterniond& curr_rot, double curr_time,
    Eigen::Vector3d& linear_vel, Eigen::Vector3d& angular_vel)
{
  double dt = curr_time - prev_time;

  // Invalid dt check
  if (dt <= 0.0 || dt > 1.0) {
    linear_vel.setZero();
    angular_vel.setZero();
    return;
  }

  // Linear velocity: (current_pos - prev_pos) / dt
  linear_vel = (curr_pos - prev_pos) / dt;

  // Angular velocity from quaternion difference
  // delta_rot = curr_rot * prev_rot.inverse()
  Eigen::Quaterniond delta_rot = curr_rot * prev_rot.inverse();

  // Convert to rotation vector (axis * angle)
  Eigen::AngleAxisd angle_axis(delta_rot);
  angular_vel = angle_axis.axis() * angle_axis.angle() / dt;
}

//==============================================================================
// Step 5: Servo API Integration (Separate Node per Arm)
//==============================================================================

class ServoController {
public:
  // Constructor uses main node directly like demo_twist.cpp
  ServoController(rclcpp::Node::SharedPtr node,
                  const std::string& param_namespace,
                  const std::string& frame_id)
    : node_(node), frame_id_(frame_id), initialized_(false)
  {
    RCLCPP_INFO(node_->get_logger(), "[ServoController] Step 1: Getting parameters for %s...", param_namespace.c_str());

    // Get servo parameters (NO try-catch to see real errors)
    servo_param_listener_ = std::make_shared<const servo::ParamListener>(node_, param_namespace);
    servo_params_ = servo_param_listener_->get_params();

    RCLCPP_INFO(node_->get_logger(), "[ServoController] Step 2: Creating trajectory publisher...");

    // Create trajectory publisher
    trajectory_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(node_->get_logger(), "[ServoController] Step 3: Creating planning scene monitor...");

    // Create planning scene monitor
    planning_scene_monitor_ = createPlanningSceneMonitor(node_, servo_params_);

    RCLCPP_INFO(node_->get_logger(), "[ServoController] Step 4: Creating Servo object...");

    // Create Servo (this is where collision monitor starts)
    servo_ = std::make_unique<Servo>(node_, servo_param_listener_, planning_scene_monitor_);

    RCLCPP_INFO(node_->get_logger(), "[ServoController] Step 5: Servo created! Waiting 3 seconds...");

    // Wait 3 seconds like demo_quest_sim.cpp does
    std::this_thread::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(node_->get_logger(), "[ServoController] Step 6: Getting robot state...");

    // Now safe to get robot state
    robot_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
    joint_model_group_ = robot_state_->getJointModelGroup(servo_params_.move_group_name);

    RCLCPP_INFO(node_->get_logger(), "[ServoController] Step 7: Setting command type...");

    // Set command type to TWIST
    servo_->setCommandType(CommandType::TWIST);

    RCLCPP_INFO(node_->get_logger(), "[ServoController] Step 8: Initializing sliding window...");

    // Initialize sliding window
    KinematicState current_state = servo_->getCurrentRobotState(true);
    updateSlidingWindow(current_state, joint_cmd_rolling_window_,
                        servo_params_.max_expected_latency, node_->now());

    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "[ServoController] DONE! Initialized for %s", servo_params_.move_group_name.c_str());
  }

  bool isInitialized() const { return initialized_; }

  // Send velocity command to servo
  bool sendVelocity(const Eigen::Vector3d& linear_vel, const Eigen::Vector3d& angular_vel) {
    if (!initialized_) return false;

    // Create TwistCommand
    TwistCommand twist;
    twist.frame_id = frame_id_;
    twist.velocities.setZero();
    twist.velocities[0] = linear_vel.x();   // vx
    twist.velocities[1] = linear_vel.y();   // vy
    twist.velocities[2] = linear_vel.z();   // vz
    twist.velocities[3] = angular_vel.x();  // wx
    twist.velocities[4] = angular_vel.y();  // wy
    twist.velocities[5] = angular_vel.z();  // wz

    // Get next joint state from Servo
    KinematicState joint_state = servo_->getNextJointState(robot_state_, twist);
    const StatusCode status = servo_->getStatus();

    if (status != StatusCode::INVALID) {
      // Update sliding window and publish trajectory
      updateSlidingWindow(joint_state, joint_cmd_rolling_window_,
                          servo_params_.max_expected_latency, node_->now());

      if (const auto msg = composeTrajectoryMessage(servo_params_, joint_cmd_rolling_window_)) {
        trajectory_pub_->publish(msg.value());
      }

      // Update robot state for next iteration
      if (!joint_cmd_rolling_window_.empty()) {
        robot_state_->setJointGroupPositions(joint_model_group_,
                                              joint_cmd_rolling_window_.back().positions);
        robot_state_->setJointGroupVelocities(joint_model_group_,
                                               joint_cmd_rolling_window_.back().velocities);
      }
      return true;
    }

    return false;
  }

  // Send zero velocity (stop)
  void stop() {
    sendVelocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }

  // Re-sync internal state to current robot state (e.g., after homing)
  void resyncToCurrentState() {
    if (!initialized_) {
      return;
    }

    KinematicState current_state = servo_->getCurrentRobotState(true);
    updateSlidingWindow(current_state, joint_cmd_rolling_window_,
                        servo_params_.max_expected_latency, node_->now());

    if (!joint_cmd_rolling_window_.empty()) {
      robot_state_->setJointGroupPositions(joint_model_group_,
                                           joint_cmd_rolling_window_.back().positions);
      robot_state_->setJointGroupVelocities(joint_model_group_,
                                            joint_cmd_rolling_window_.back().velocities);
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string frame_id_;
  bool initialized_;

  std::shared_ptr<const servo::ParamListener> servo_param_listener_;
  servo::Params servo_params_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::unique_ptr<Servo> servo_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  std::deque<KinematicState> joint_cmd_rolling_window_;
};

//==============================================================================
// Socket Server
//==============================================================================

class QuestSocketServer {
public:
  QuestSocketServer(const std::string& host = "0.0.0.0", int port = 5454)
    : host_(host), port_(port), running_(false), connected_(false), server_fd_(-1), client_fd_(-1)
  {
  }

  ~QuestSocketServer() {
    stop();
  }

  bool start() {
    if (running_) return true;

    // Create socket
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      std::cerr << "[SOCKET] Failed to create socket" << std::endl;
      return false;
    }

    // Set socket options
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Bind
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port_);

    if (bind(server_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      std::cerr << "[SOCKET] Failed to bind to port " << port_ << std::endl;
      close(server_fd_);
      server_fd_ = -1;
      return false;
    }

    // Listen
    if (listen(server_fd_, 1) < 0) {
      std::cerr << "[SOCKET] Failed to listen" << std::endl;
      close(server_fd_);
      server_fd_ = -1;
      return false;
    }

    running_ = true;
    server_thread_ = std::thread(&QuestSocketServer::serverLoop, this);

    std::cout << "[SOCKET] Listening on " << host_ << ":" << port_ << std::endl;
    return true;
  }

  void stop() {
    running_ = false;
    connected_ = false;

    if (client_fd_ >= 0) {
      close(client_fd_);
      client_fd_ = -1;
    }
    if (server_fd_ >= 0) {
      close(server_fd_);
      server_fd_ = -1;
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

  bool isConnected() const {
    return connected_;
  }

  QuestData getLatestData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_data_;
  }

private:
  void serverLoop() {
    // Set server socket to non-blocking for clean shutdown
    fcntl(server_fd_, F_SETFL, O_NONBLOCK);

    while (running_) {
      // Accept connection (non-blocking)
      struct sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);
      client_fd_ = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);

      if (client_fd_ >= 0) {
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        std::cout << "[SOCKET] Connected: " << client_ip << std::endl;
        connected_ = true;

        // Handle client connection
        handleClient();

        connected_ = false;
        close(client_fd_);
        client_fd_ = -1;
        std::cout << "[SOCKET] Disconnected" << std::endl;
      } else {
        // No connection yet, sleep briefly
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  void handleClient() {
    std::string buffer;
    char recv_buf[1024];

    while (running_ && connected_) {
      ssize_t bytes = recv(client_fd_, recv_buf, sizeof(recv_buf) - 1, 0);

      if (bytes <= 0) {
        if (bytes == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
          break;  // Connection closed or error
        }
        continue;
      }

      recv_buf[bytes] = '\0';
      buffer += recv_buf;

      // Process complete JSON lines
      size_t pos;
      while ((pos = buffer.find('\n')) != std::string::npos) {
        std::string line = buffer.substr(0, pos);
        buffer.erase(0, pos + 1);

        if (!line.empty()) {
          parseQuestData(line);
        }
      }
    }
  }

  void parseQuestData(const std::string& json_str) {
    try {
      json j = json::parse(json_str);

      QuestData data;
      data.timestamp = j.value("timestamp", 0.0);

      // Parse left controller
      if (j.contains("left")) {
        auto& left = j["left"];
        data.left.enabled = left.value("enabled", false);
        data.left.trigger = left.value("trigger", 0.0);

        if (left.contains("position")) {
          data.left.position.x() = left["position"].value("x", 0.0);
          data.left.position.y() = left["position"].value("y", 0.0);
          data.left.position.z() = left["position"].value("z", 0.0);
        }
        if (left.contains("rotation")) {
          double x = left["rotation"].value("x", 0.0);
          double y = left["rotation"].value("y", 0.0);
          double z = left["rotation"].value("z", 0.0);
          double w = left["rotation"].value("w", 1.0);
          data.left.rotation = Eigen::Quaterniond(w, x, y, z);
        }
      }

      // Parse right controller
      if (j.contains("right")) {
        auto& right = j["right"];
        data.right.enabled = right.value("enabled", false);
        data.right.trigger = right.value("trigger", 0.0);

        if (right.contains("position")) {
          data.right.position.x() = right["position"].value("x", 0.0);
          data.right.position.y() = right["position"].value("y", 0.0);
          data.right.position.z() = right["position"].value("z", 0.0);
        }
        if (right.contains("rotation")) {
          double x = right["rotation"].value("x", 0.0);
          double y = right["rotation"].value("y", 0.0);
          double z = right["rotation"].value("z", 0.0);
          double w = right["rotation"].value("w", 1.0);
          data.right.rotation = Eigen::Quaterniond(w, x, y, z);
        }
      }

      // Update latest data
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_ = data;
      }

    } catch (const json::parse_error& e) {
      // Ignore parse errors (incomplete JSON, etc.)
    }
  }

  std::string host_;
  int port_;
  std::atomic<bool> running_;
  std::atomic<bool> connected_;
  int server_fd_;
  int client_fd_;
  std::thread server_thread_;
  std::mutex data_mutex_;
  QuestData latest_data_;
};

//==============================================================================
// Homing: Move joint4 to 1.58 rad (4 seconds, tolerance 0.03)
//==============================================================================

class HomingController {
public:
  HomingController(rclcpp::Node::SharedPtr node)
    : node_(node), homing_complete_(false)
  {
    // Publishers for left and right arm trajectories
    traj_pub_left_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/left_joint_trajectory_controller/joint_trajectory", 10);
    traj_pub_right_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/right_joint_trajectory_controller/joint_trajectory", 10);

    // Subscriber for joint states
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&HomingController::jointStateCallback, this, std::placeholders::_1));
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_positions_[msg->name[i]] = msg->position[i];
    }
  }

  double getJointPosition(const std::string& joint_name) {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    auto it = joint_positions_.find(joint_name);
    return (it != joint_positions_.end()) ? it->second : 0.0;
  }

  // Send homing trajectory for one arm
  void sendHomingTrajectory(const std::string& arm) {
    std::string prefix = (arm == "left") ? "openarm_left_" : "openarm_right_";
    std::vector<std::string> joint_names;
    std::vector<double> positions;

    for (int i = 1; i <= 7; ++i) {
      std::string joint_name = prefix + "joint" + std::to_string(i);
      joint_names.push_back(joint_name);
      double current = getJointPosition(joint_name);
      positions.push_back((i == 4) ? 1.58 : current);  // Only joint4 changes
    }

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = node_->now();
    traj.joint_names = joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities = std::vector<double>(7, 0.0);
    point.time_from_start.sec = 4;
    point.time_from_start.nanosec = 0;
    traj.points.push_back(point);

    if (arm == "left") {
      traj_pub_left_->publish(traj);
    } else {
      traj_pub_right_->publish(traj);
    }

    RCLCPP_INFO(node_->get_logger(), "[HOMING] %s arm: joint4 -> 1.58 (4s)", arm.c_str());
  }

  // Check if joint4 reached target
  bool isHomingComplete(const std::string& arm) {
    std::string joint_name = (arm == "left") ? "openarm_left_joint4" : "openarm_right_joint4";
    double current = getJointPosition(joint_name);
    return std::abs(current - 1.58) < 0.03;  // Tolerance ±0.03 rad
  }

  // Execute full homing sequence: left then right
  bool executeHoming() {
    if (homing_complete_) return true;

    // Wait for joint states
    RCLCPP_INFO(node_->get_logger(), "[HOMING] Waiting for joint states...");
    rclcpp::Rate wait_rate(10);
    int wait_count = 0;
    while (rclcpp::ok() && wait_count++ < 30) {  // 3 second timeout
      rclcpp::spin_some(node_);
      if (getJointPosition("openarm_left_joint4") != 0.0 &&
          getJointPosition("openarm_right_joint4") != 0.0) {
        break;
      }
      wait_rate.sleep();
    }

    // Home left arm
    RCLCPP_INFO(node_->get_logger(), "[HOMING] Starting left arm...");
    sendHomingTrajectory("left");

    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !isHomingComplete("left")) {
      rclcpp::spin_some(node_);
      rate.sleep();
    }
    RCLCPP_INFO(node_->get_logger(), "[HOMING] Left arm complete!");

    // Home right arm
    RCLCPP_INFO(node_->get_logger(), "[HOMING] Starting right arm...");
    sendHomingTrajectory("right");

    while (rclcpp::ok() && !isHomingComplete("right")) {
      rclcpp::spin_some(node_);
      rate.sleep();
    }
    RCLCPP_INFO(node_->get_logger(), "[HOMING] Right arm complete!");

    homing_complete_ = true;
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_left_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_right_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  std::mutex joint_mutex_;
  std::map<std::string, double> joint_positions_;
  bool homing_complete_;
};

//==============================================================================
// Step 7: Gripper Control
//==============================================================================

class GripperController {
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  GripperController(rclcpp::Node::SharedPtr node)
    : node_(node),
      gripper_min_(0.0),      // Fully closed
      gripper_max_(0.044),    // Fully open (44mm)
      prev_trigger_left_(-1.0),
      prev_trigger_right_(-1.0)
  {
    // Create action clients for grippers
    gripper_client_left_ = rclcpp_action::create_client<GripperCommand>(
        node_, "/left_gripper_controller/gripper_cmd");
    gripper_client_right_ = rclcpp_action::create_client<GripperCommand>(
        node_, "/right_gripper_controller/gripper_cmd");

    RCLCPP_INFO(node_->get_logger(), "[GripperController] Initialized");
  }

  // Send gripper command based on trigger value (0~1)
  // Inverted: trigger 0 (release) = open, trigger 1 (press) = close
  void sendGripperCommand(const std::string& arm, double trigger_value) {
    // Invert trigger
    double inverted = 1.0 - trigger_value;

    // Map to gripper position
    double position = gripper_min_ + (inverted * (gripper_max_ - gripper_min_));

    // Create goal
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = 100.0;

    // Send goal (non-blocking)
    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();

    if (arm == "left" && gripper_client_left_->action_server_is_ready()) {
      gripper_client_left_->async_send_goal(goal_msg, send_goal_options);
    } else if (arm == "right" && gripper_client_right_->action_server_is_ready()) {
      gripper_client_right_->async_send_goal(goal_msg, send_goal_options);
    }
  }

  // Update grippers based on Quest trigger values
  void update(double left_trigger, double right_trigger) {
    // Only send if trigger changed significantly
    if (std::abs(left_trigger - prev_trigger_left_) > 0.01) {
      sendGripperCommand("left", left_trigger);
      prev_trigger_left_ = left_trigger;
    }

    if (std::abs(right_trigger - prev_trigger_right_) > 0.01) {
      sendGripperCommand("right", right_trigger);
      prev_trigger_right_ = right_trigger;
    }
  }

  // Open both grippers (for homing)
  void openBoth() {
    sendGripperCommand("left", 0.0);   // trigger 0 = open
    sendGripperCommand("right", 0.0);
    prev_trigger_left_ = 0.0;
    prev_trigger_right_ = 0.0;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_left_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_right_;

  double gripper_min_;
  double gripper_max_;
  double prev_trigger_left_;
  double prev_trigger_right_;
};

//==============================================================================
// Step 6: Bimanual Teleop Controller (Separate Nodes per Arm)
//==============================================================================

class BimanualTeleop {
public:
  BimanualTeleop(rclcpp::Node::SharedPtr node)
    : node_(node),
      calibrator_left_(1.0),   // 1 second calibration
      calibrator_right_(1.0),
      prev_timestamp_left_(0.0),
      prev_timestamp_right_(0.0),
      left_calibrated_(false),
      right_calibrated_(false)
  {
    RCLCPP_INFO(node_->get_logger(), "[BimanualTeleop] Creating LEFT servo only (test mode)...");

    // Create only LEFT ServoController using main node (like demo_twist.cpp)
    servo_left_ = std::make_unique<ServoController>(
        node_, "moveit_servo_left", "openarm_left_link0");

    RCLCPP_INFO(node_->get_logger(), "[BimanualTeleop] Left servo created!");

    // Skip right servo for now - test single servo first
    // servo_right_ = std::make_unique<ServoController>(
    //     node_, "moveit_servo_right", "openarm_right_link0");
  }

  bool isReady() const {
    // Test mode: only check left servo
    return servo_left_ && servo_left_->isInitialized();
  }

  // Process Quest data and send to servos
  void update(const QuestData& robot_data) {
    // Process left arm
    if (robot_data.left.enabled) {
      processArm("left", robot_data.left, robot_data.timestamp,
                 calibrator_left_, left_calibrated_,
                 prev_pos_left_, prev_rot_left_, prev_timestamp_left_,
                 *servo_left_);
    }

    // Process right arm
    if (robot_data.right.enabled) {
      processArm("right", robot_data.right, robot_data.timestamp,
                 calibrator_right_, right_calibrated_,
                 prev_pos_right_, prev_rot_right_, prev_timestamp_right_,
                 *servo_right_);
    }
  }

  bool isLeftCalibrated() const { return left_calibrated_; }
  bool isRightCalibrated() const { return right_calibrated_; }

  void resyncServoState() {
    if (servo_left_) {
      servo_left_->resyncToCurrentState();
    }
    if (servo_right_) {
      servo_right_->resyncToCurrentState();
    }
  }

private:
  void processArm(const std::string& arm_name,
                  const ControllerData& data,
                  double timestamp,
                  Calibrator& calibrator,
                  bool& calibrated,
                  Eigen::Vector3d& prev_pos,
                  Eigen::Quaterniond& prev_rot,
                  double& prev_time,
                  ServoController& servo)
  {
    if (!calibrated) {
      // Calibration phase: collect samples for 1 second
      if (calibrator.update(data, timestamp)) {
        // Calibration complete - set initial prev values
        prev_pos = calibrator.getAvgPosition();
        prev_rot = calibrator.getAvgRotation();
        prev_time = timestamp;
        calibrated = true;
        RCLCPP_INFO(node_->get_logger(), "[%s] Calibration complete! Samples: %zu",
                    arm_name.c_str(), calibrator.getSampleCount());
      }
      return;
    }

    // Teleoperation phase: calculate velocity and send to servo
    Eigen::Vector3d linear_vel, angular_vel;
    calculateVelocity(prev_pos, prev_rot, prev_time,
                      data.position, data.rotation, timestamp,
                      linear_vel, angular_vel);

    // Send to servo
    servo.sendVelocity(linear_vel, angular_vel);

    // Update prev values
    prev_pos = data.position;
    prev_rot = data.rotation;
    prev_time = timestamp;
  }

  rclcpp::Node::SharedPtr node_;

  // Servo controllers (using main node like demo_twist.cpp)
  std::unique_ptr<ServoController> servo_left_;
  std::unique_ptr<ServoController> servo_right_;

  // Calibrators
  Calibrator calibrator_left_;
  Calibrator calibrator_right_;
  bool left_calibrated_;
  bool right_calibrated_;

  // Previous frame data - Left
  Eigen::Vector3d prev_pos_left_;
  Eigen::Quaterniond prev_rot_left_;
  double prev_timestamp_left_;

  // Previous frame data - Right
  Eigen::Vector3d prev_pos_right_;
  Eigen::Quaterniond prev_rot_right_;
  double prev_timestamp_right_;
};

//==============================================================================
// Main Node (Multi-threaded executor for separate servo nodes)
//==============================================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create main node with options to receive parameters from launch file
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("quest_teleop", node_options);
  moveit::setNodeLoggerName(node->get_name());

  RCLCPP_INFO(node->get_logger(), "=== Quest VR Teleop for OpenArm ===");
  RCLCPP_INFO(node->get_logger(), "Step 1: Socket Server (TCP port 5454)");
  RCLCPP_INFO(node->get_logger(), "Step 2: Coordinate Transformation");
  RCLCPP_INFO(node->get_logger(), "Step 3: Calibration (1 sec per arm)");
  RCLCPP_INFO(node->get_logger(), "Step 4: Velocity Conversion");
  RCLCPP_INFO(node->get_logger(), "Step 5: Servo API Integration (separate nodes)");
  RCLCPP_INFO(node->get_logger(), "Step 6: Bimanual Support");
  RCLCPP_INFO(node->get_logger(), "Step 7: Gripper Control");
  RCLCPP_INFO(node->get_logger(), "Step 8: Safety Features");

  // Wait for system to be ready
  RCLCPP_INFO(node->get_logger(), "Waiting 3 seconds for system ready...");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Create bimanual teleop controller using main node (like demo_twist.cpp)
  BimanualTeleop teleop(node);
  if (!teleop.isReady()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize servo controllers!");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Using simulated Quest input...");

  // Execute homing after servo is ready
  HomingController homing(node);
  homing.executeHoming();
  teleop.resyncServoState();

  // Create gripper controller and open grippers after homing
  GripperController gripper(node);
  gripper.openBoth();
  RCLCPP_INFO(node->get_logger(), "[GRIPPER] Opening both grippers...");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Main loop
  rclcpp::WallRate rate(100.0);  // 100Hz
  int log_counter = 0;
  const auto start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    // Spin to process callbacks (like demo_twist.cpp)
    rclcpp::spin_some(node);

    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = now - start_time;

    // Get raw Quest data (simulated) without socket input
    QuestData robot_data = simulateQuestData(elapsed.count());

    // Process teleop
    teleop.update(robot_data);

    // Step 7: Update grippers based on trigger values
    if (robot_data.left.enabled || robot_data.right.enabled) {
      gripper.update(
          robot_data.left.enabled ? robot_data.left.trigger : 0.0,
          robot_data.right.enabled ? robot_data.right.trigger : 0.0);
    }

    // Log every second
    if (++log_counter % 100 == 0) {
      RCLCPP_INFO(node->get_logger(),
        "L: cal=%d en=%d trig=%.2f | R: cal=%d en=%d trig=%.2f",
        teleop.isLeftCalibrated(), robot_data.left.enabled, robot_data.left.trigger,
        teleop.isRightCalibrated(), robot_data.right.enabled, robot_data.right.trigger);
    }

    rate.sleep();
  }

  // Cleanup
  RCLCPP_INFO(node->get_logger(), "Quest teleop finished.");
  rclcpp::shutdown();
  return 0;
}
