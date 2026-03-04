/*******************************************************************************
 * Title     : quest_teleop_bimanual.cpp
 * Project   : openarm_quest_teleop
 * Purpose   : Quest VR Teleoperation for Bimanual Arms using MoveIt Servo C++ API
 *             PlanningSceneMonitor is shared between both arms
 *******************************************************************************/

#include <chrono>
#include <cmath>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

// MoveIt Servo includes
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <moveit/utils/logger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
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
#include <optional>

using json = nlohmann::json;
using namespace moveit_servo;

//==============================================================================
// Quest Data Structures
//==============================================================================

struct ControllerData {
  Eigen::Vector3d position{0, 0, 0};
  Eigen::Vector3d euler{0, 0, 0};  // x, y, z in degrees (rotation around each axis)
  double trigger{0.0};
  bool enabled{false};
};

struct HeadData {
  Eigen::Vector3d euler{0, 0, 0};  // x=pitch, y=yaw, z=roll in degrees
  bool valid{false};
};

struct QuestData {
  ControllerData left;
  ControllerData right;
  HeadData head;
  double timestamp{0.0};
  bool x_button{false};
  // Joystick data for special commands
  double agv_x{0.0};   // Right joystick X
  double agv_y{0.0};   // Right joystick Y
  double lift{0.0};    // Left joystick Y
};

//==============================================================================
// Coordinate Transformation (Quest -> Robot EEF)
//==============================================================================

// Quest: front=+Z, up=+Y, right=+X
// Robot: front=+Z, up=+X, right=+Y
ControllerData transformToRobotFrame(const ControllerData& quest_data) {
  ControllerData robot_data;
  robot_data.enabled = quest_data.enabled;
  robot_data.trigger = quest_data.trigger;

  // Position transformation (swap X <-> Y)
  robot_data.position.x() = quest_data.position.y();  // Quest Y (up) -> Robot X (up)
  robot_data.position.y() = quest_data.position.x();  // Quest X (right) -> Robot Y (right)
  robot_data.position.z() = quest_data.position.z();  // Quest Z (front) -> Robot Z (front)

  // Euler transformation (same axis swap as position, signs inverted)
  robot_data.euler.x() = -quest_data.euler.y();  // Quest Y rotation -> Robot X rotation (inverted)
  robot_data.euler.y() = -quest_data.euler.x();  // Quest X rotation -> Robot Y rotation (inverted)
  robot_data.euler.z() = -quest_data.euler.z();  // Quest Z rotation -> Robot Z rotation (inverted)

  return robot_data;
}

QuestData transformQuestData(const QuestData& quest_raw) {
  QuestData robot_data;
  robot_data.timestamp = quest_raw.timestamp;
  robot_data.left = transformToRobotFrame(quest_raw.left);
  robot_data.right = transformToRobotFrame(quest_raw.right);
  return robot_data;
}

//==============================================================================
// Calibration Logic
//==============================================================================

class Calibrator {
public:
  Calibrator(double duration_sec = 1.0) : duration_(duration_sec) {
    reset();
  }

  void reset() {
    calibrated_ = false;
    start_time_ = -1.0;
    position_samples_.clear();
    euler_samples_.clear();
  }

  bool update(const ControllerData& data, double current_time) {
    if (calibrated_) return true;

    if (start_time_ < 0.0) {
      start_time_ = current_time;
    }

    position_samples_.push_back(data.position);
    euler_samples_.push_back(data.euler);

    if (current_time - start_time_ >= duration_) {
      avg_position_ = Eigen::Vector3d::Zero();
      for (const auto& pos : position_samples_) {
        avg_position_ += pos;
      }
      avg_position_ /= static_cast<double>(position_samples_.size());

      avg_euler_ = Eigen::Vector3d::Zero();
      for (const auto& euler : euler_samples_) {
        avg_euler_ += euler;
      }
      avg_euler_ /= static_cast<double>(euler_samples_.size());

      calibrated_ = true;
      return true;
    }

    return false;
  }

  bool isCalibrated() const { return calibrated_; }
  size_t getSampleCount() const { return position_samples_.size(); }
  Eigen::Vector3d getAvgPosition() const { return avg_position_; }
  Eigen::Vector3d getAvgEuler() const { return avg_euler_; }

private:
  double duration_;
  double start_time_;
  bool calibrated_;
  std::vector<Eigen::Vector3d> position_samples_;
  std::vector<Eigen::Vector3d> euler_samples_;
  Eigen::Vector3d avg_position_;
  Eigen::Vector3d avg_euler_;
};

//==============================================================================
// Velocity Conversion
//==============================================================================

constexpr double DEG_TO_RAD = M_PI / 180.0;

// Handle angle wrapping (e.g., 359 -> 1 should be +2, not -358)
double wrapAngleDelta(double delta) {
  while (delta > 180.0) delta -= 360.0;
  while (delta < -180.0) delta += 360.0;
  return delta;
}

void calculateVelocity(
    const Eigen::Vector3d& prev_pos, const Eigen::Vector3d& prev_euler, double prev_time,
    const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& curr_euler, double curr_time,
    Eigen::Vector3d& linear_vel, Eigen::Vector3d& angular_vel)
{
  double dt = curr_time - prev_time;

  if (dt <= 0.0 || dt > 1.0) {
    linear_vel.setZero();
    angular_vel.setZero();
    return;
  }

  // Linear velocity (m/s)
  linear_vel = (curr_pos - prev_pos) / dt;

  // Angular velocity from euler angles (deg -> rad/s)
  double dx = wrapAngleDelta(curr_euler.x() - prev_euler.x());
  double dy = wrapAngleDelta(curr_euler.y() - prev_euler.y());
  double dz = wrapAngleDelta(curr_euler.z() - prev_euler.z());

  angular_vel.x() = dx * DEG_TO_RAD / dt;
  angular_vel.y() = dy * DEG_TO_RAD / dt;
  angular_vel.z() = dz * DEG_TO_RAD / dt;
}

//==============================================================================
// ServoController (accepts external PlanningSceneMonitor)
//==============================================================================

class ServoController {
public:
  // Constructor that accepts shared PlanningSceneMonitor
  ServoController(rclcpp::Node::SharedPtr node,
                  const std::string& param_namespace,
                  const std::string& frame_id,
                  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
    : node_(node), frame_id_(frame_id), initialized_(false),
      planning_scene_monitor_(planning_scene_monitor)
  {
    RCLCPP_INFO(node_->get_logger(), "[ServoController] Initializing %s with shared PlanningSceneMonitor...", param_namespace.c_str());

    // Get servo parameters
    servo_param_listener_ = std::make_shared<const servo::ParamListener>(node_, param_namespace);
    servo_params_ = servo_param_listener_->get_params();

    // Create trajectory publisher
    trajectory_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());

    // Create Servo with shared PlanningSceneMonitor
    servo_ = std::make_unique<Servo>(node_, servo_param_listener_, planning_scene_monitor_);

    RCLCPP_INFO(node_->get_logger(), "[ServoController] Servo created, waiting 3 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Get robot state
    robot_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
    joint_model_group_ = robot_state_->getJointModelGroup(servo_params_.move_group_name);

    // Set command type to POSE
    servo_->setCommandType(CommandType::POSE);

    // Get IK solver base frame and tip frame for POSE commands
    auto ik_base = getIKSolverBaseFrame(robot_state_, servo_params_.move_group_name);
    auto ik_tip = getIKSolverTipFrame(robot_state_, servo_params_.move_group_name);
    if (!ik_base || !ik_tip) {
      RCLCPP_ERROR(node_->get_logger(), "[ServoController] No IK solver found for %s!", servo_params_.move_group_name.c_str());
      return;
    }
    planning_frame_ = ik_base.value();
    tip_frame_ = ik_tip.value();
    RCLCPP_INFO(node_->get_logger(), "[ServoController] IK frames: base=%s, tip=%s",
                planning_frame_.c_str(), tip_frame_.c_str());

    // Log planning frame global transform for debugging axis mapping
    {
      const Eigen::Isometry3d& pf_tf = robot_state_->getGlobalLinkTransform(planning_frame_);
      Eigen::Vector3d pf_pos = pf_tf.translation();
      Eigen::Matrix3d pf_rot = pf_tf.rotation();
      RCLCPP_INFO(node_->get_logger(),
          "[ServoController] %s global transform: pos=(%.4f, %.4f, %.4f)",
          planning_frame_.c_str(), pf_pos.x(), pf_pos.y(), pf_pos.z());
      RCLCPP_INFO(node_->get_logger(),
          "[ServoController] %s rotation row0=(%.4f, %.4f, %.4f) row1=(%.4f, %.4f, %.4f) row2=(%.4f, %.4f, %.4f)",
          planning_frame_.c_str(),
          pf_rot(0,0), pf_rot(0,1), pf_rot(0,2),
          pf_rot(1,0), pf_rot(1,1), pf_rot(1,2),
          pf_rot(2,0), pf_rot(2,1), pf_rot(2,2));

      // Also log tip frame
      const Eigen::Isometry3d& tip_tf = robot_state_->getGlobalLinkTransform(tip_frame_);
      Eigen::Vector3d tip_pos = tip_tf.translation();
      RCLCPP_INFO(node_->get_logger(),
          "[ServoController] %s global pos=(%.4f, %.4f, %.4f)",
          tip_frame_.c_str(), tip_pos.x(), tip_pos.y(), tip_pos.z());

      // Log initial EE pose in planning frame
      Eigen::Isometry3d ee_pose = pf_tf.inverse() * tip_tf;
      Eigen::Vector3d ee_pos = ee_pose.translation();
      RCLCPP_INFO(node_->get_logger(),
          "[ServoController] initial EE in planning frame: pos=(%.4f, %.4f, %.4f)",
          ee_pos.x(), ee_pos.y(), ee_pos.z());
    }

    // Initialize sliding window
    KinematicState current_state = servo_->getCurrentRobotState(true);
    updateSlidingWindow(current_state, joint_cmd_rolling_window_,
                        servo_params_.max_expected_latency, node_->now());

    // Cache world-to-planning-frame rotation (link0 is fixed joint, constant)
    world_to_planning_rot_ = robot_state_->getGlobalLinkTransform(planning_frame_).rotation().transpose();

    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "[ServoController] DONE! Initialized for %s (POSE mode)", servo_params_.move_group_name.c_str());
  }

  bool isInitialized() const { return initialized_; }

  // Transform a vector from world frame to planning (link0) frame
  Eigen::Vector3d worldToPlanning(const Eigen::Vector3d& v_world) const {
    return world_to_planning_rot_ * v_world;
  }

  // Get the world-to-planning rotation matrix
  const Eigen::Matrix3d& getWorldToPlanningRot() const {
    return world_to_planning_rot_;
  }

  bool sendPose(const Eigen::Isometry3d& target_pose) {
    if (!initialized_) return false;

    PoseCommand pose_cmd;
    pose_cmd.frame_id = planning_frame_;
    pose_cmd.pose = target_pose;

    KinematicState joint_state = servo_->getNextJointState(robot_state_, pose_cmd);
    const StatusCode status = servo_->getStatus();

    if (status == StatusCode::INVALID) {
      static int fail_count = 0;
      if (++fail_count % 50 == 1) {
        RCLCPP_WARN(node_->get_logger(), "[ServoController] sendPose INVALID (count=%d)", fail_count);
      }
    }

    if (status != StatusCode::INVALID) {
      updateSlidingWindow(joint_state, joint_cmd_rolling_window_,
                          servo_params_.max_expected_latency, node_->now());

      if (const auto msg = composeTrajectoryMessage(servo_params_, joint_cmd_rolling_window_)) {
        trajectory_pub_->publish(msg.value());
      }

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

  Eigen::Isometry3d getCurrentEEPose() {
    const Eigen::Isometry3d& base_transform = robot_state_->getGlobalLinkTransform(planning_frame_);
    const Eigen::Isometry3d& tip_transform = robot_state_->getGlobalLinkTransform(tip_frame_);
    return base_transform.inverse() * tip_transform;
  }

  Eigen::Isometry3d getGlobalEEPose() {
    return robot_state_->getGlobalLinkTransform(tip_frame_);
  }

  std::optional<Eigen::VectorXd> getLastCommandedJointPositions() const {
    if (!joint_cmd_rolling_window_.empty())
      return joint_cmd_rolling_window_.back().positions;
    return std::nullopt;
  }

  void stop() {
    if (!initialized_) return;
    // In POSE mode, send current pose to hold position
    sendPose(getCurrentEEPose());
  }

  void resyncToCurrentState() {
    if (!initialized_) return;

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

  // IK solver frames for POSE mode
  std::string planning_frame_;
  std::string tip_frame_;
  Eigen::Matrix3d world_to_planning_rot_;  // cached rotation: world -> link0

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

    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      std::cerr << "[SOCKET] Failed to create socket" << std::endl;
      return false;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

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

  bool isConnected() const { return connected_; }

  QuestData getLatestData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_data_;
  }

private:
  void serverLoop() {
    fcntl(server_fd_, F_SETFL, O_NONBLOCK);

    while (running_) {
      struct sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);
      client_fd_ = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);

      if (client_fd_ >= 0) {
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        std::cout << "[SOCKET] Connected: " << client_ip << std::endl;
        connected_ = true;

        handleClient();

        connected_ = false;
        close(client_fd_);
        client_fd_ = -1;
        std::cout << "[SOCKET] Disconnected" << std::endl;
      } else {
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
          break;
        }
        continue;
      }

      recv_buf[bytes] = '\0';
      buffer += recv_buf;

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
      data.x_button = j.value("x_button", false);

      if (j.contains("left")) {
        auto& left = j["left"];
        data.left.enabled = left.value("enabled", false);
        data.left.trigger = left.value("trigger", 0.0);

        if (left.contains("position")) {
          data.left.position.x() = left["position"].value("x", 0.0);
          data.left.position.y() = left["position"].value("y", 0.0);
          data.left.position.z() = left["position"].value("z", 0.0);
        }
        if (left.contains("euler")) {
          data.left.euler.x() = left["euler"].value("x", 0.0);
          data.left.euler.y() = left["euler"].value("y", 0.0);
          data.left.euler.z() = left["euler"].value("z", 0.0);
        }
      }

      if (j.contains("right")) {
        auto& right = j["right"];
        data.right.enabled = right.value("enabled", false);
        data.right.trigger = right.value("trigger", 0.0);

        if (right.contains("position")) {
          data.right.position.x() = right["position"].value("x", 0.0);
          data.right.position.y() = right["position"].value("y", 0.0);
          data.right.position.z() = right["position"].value("z", 0.0);
        }
        if (right.contains("euler")) {
          data.right.euler.x() = right["euler"].value("x", 0.0);
          data.right.euler.y() = right["euler"].value("y", 0.0);
          data.right.euler.z() = right["euler"].value("z", 0.0);
        }
      }

      // Parse head (headset) data
      if (j.contains("head")) {
        auto& head = j["head"];
        if (head.contains("euler")) {
          data.head.euler.x() = head["euler"].value("x", 0.0);
          data.head.euler.y() = head["euler"].value("y", 0.0);
          data.head.euler.z() = head["euler"].value("z", 0.0);
          data.head.valid = true;
        }
      }

      // Parse joystick data for homing trigger
      if (j.contains("agv")) {
        data.agv_x = j["agv"].value("x", 0.0);
        data.agv_y = j["agv"].value("y", 0.0);
      }
      if (j.contains("lift")) {
        data.lift = j["lift"].value("value", 0.0);
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_ = data;
      }

    } catch (const json::parse_error& e) {
      // Ignore parse errors
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
// Bimanual Homing Controller
//==============================================================================

class HomingController {
public:
  HomingController(rclcpp::Node::SharedPtr node)
    : node_(node), homing_complete_(false)
  {
    // Publishers for both arms
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

  void sendHomingTrajectory(const std::string& arm) {
    std::string prefix = (arm == "left") ? "openarm_left_" : "openarm_right_";
    std::vector<std::string> joint_names;
    std::vector<double> positions;

    for (int i = 1; i <= 7; ++i) {
      std::string joint_name = prefix + "joint" + std::to_string(i);
      joint_names.push_back(joint_name);
      double current = getJointPosition(joint_name);
      positions.push_back((i == 4) ? 1.58 : current);
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

  bool isHomingComplete(const std::string& arm) {
    std::string joint_name = (arm == "left") ? "openarm_left_joint4" : "openarm_right_joint4";
    double current = getJointPosition(joint_name);
    return std::abs(current - 1.58) < 0.04;
  }

  // Smooth homing for both arms simultaneously (called during teleop)
  // All joints go to 0, except joint4 goes to 1.58
  void smoothHomingBoth(double duration_sec = 3.0) {
    RCLCPP_INFO(node_->get_logger(), "[HOMING] Smooth homing triggered (%.1fs)...", duration_sec);

    // Send homing trajectory for left arm
    {
      std::string prefix = "openarm_left_";
      std::vector<std::string> joint_names;
      std::vector<double> positions;
      for (int i = 1; i <= 7; ++i) {
        joint_names.push_back(prefix + "joint" + std::to_string(i));
        positions.push_back((i == 4) ? 1.58 : 0.0);  // joint4=1.58, others=0
      }

      trajectory_msgs::msg::JointTrajectory traj;
      traj.header.stamp = node_->now();
      traj.joint_names = joint_names;

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = positions;
      point.velocities = std::vector<double>(7, 0.0);
      point.time_from_start.sec = static_cast<int>(duration_sec);
      point.time_from_start.nanosec = static_cast<int>((duration_sec - static_cast<int>(duration_sec)) * 1e9);
      traj.points.push_back(point);

      traj_pub_left_->publish(traj);
    }

    // Send homing trajectory for right arm
    {
      std::string prefix = "openarm_right_";
      std::vector<std::string> joint_names;
      std::vector<double> positions;
      for (int i = 1; i <= 7; ++i) {
        joint_names.push_back(prefix + "joint" + std::to_string(i));
        positions.push_back((i == 4) ? 1.58 : 0.0);  // joint4=1.58, others=0
      }

      trajectory_msgs::msg::JointTrajectory traj;
      traj.header.stamp = node_->now();
      traj.joint_names = joint_names;

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = positions;
      point.velocities = std::vector<double>(7, 0.0);
      point.time_from_start.sec = static_cast<int>(duration_sec);
      point.time_from_start.nanosec = static_cast<int>((duration_sec - static_cast<int>(duration_sec)) * 1e9);
      traj.points.push_back(point);

      traj_pub_right_->publish(traj);
    }

    RCLCPP_INFO(node_->get_logger(), "[HOMING] Both arms returning to home position (all joints->0, j4->1.58)");
  }

  bool executeHoming() {
    if (homing_complete_) return true;

    // Wait for joint states
    RCLCPP_INFO(node_->get_logger(), "[HOMING] Waiting for joint states...");
    rclcpp::Rate wait_rate(10);
    int wait_count = 0;
    while (rclcpp::ok() && wait_count++ < 30) {
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
// Bimanual Gripper Controller
//==============================================================================

class GripperController {
public:
  using GripperCommand = control_msgs::action::GripperCommand;

  GripperController(rclcpp::Node::SharedPtr node)
    : node_(node),
      gripper_min_(0.0),
      gripper_max_(0.0264),
      prev_trigger_left_(-1.0),
      prev_trigger_right_(-1.0)
  {
    gripper_client_left_ = rclcpp_action::create_client<GripperCommand>(
        node_, "/left_gripper_controller/gripper_cmd");
    gripper_client_right_ = rclcpp_action::create_client<GripperCommand>(
        node_, "/right_gripper_controller/gripper_cmd");

    RCLCPP_INFO(node_->get_logger(), "[GripperController] Initialized (bimanual)");
  }

  void sendGripperCommand(const std::string& arm, double trigger_value) {
    double inverted = 1.0 - trigger_value;
    double position = gripper_min_ + (inverted * (gripper_max_ - gripper_min_));

    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = 100.0;

    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();

    if (arm == "left" && gripper_client_left_->action_server_is_ready()) {
      gripper_client_left_->async_send_goal(goal_msg, send_goal_options);
    } else if (arm == "right" && gripper_client_right_->action_server_is_ready()) {
      gripper_client_right_->async_send_goal(goal_msg, send_goal_options);
    }
  }

  void update(double left_trigger, double right_trigger) {
    if (std::abs(left_trigger - prev_trigger_left_) > 0.01) {
      sendGripperCommand("left", left_trigger);
      prev_trigger_left_ = left_trigger;
    }

    if (std::abs(right_trigger - prev_trigger_right_) > 0.01) {
      sendGripperCommand("right", right_trigger);
      prev_trigger_right_ = right_trigger;
    }
  }

  void openBoth() {
    sendGripperCommand("left", 0.0);
    sendGripperCommand("right", 0.0);
    prev_trigger_left_ = 0.0;
    prev_trigger_right_ = 0.0;
  }

  // Smoothly open both grippers over specified duration
  void openBothSmooth(double duration_sec = 2.0) {
    RCLCPP_INFO(node_->get_logger(), "[GripperController] Opening grippers smoothly over %.1fs...", duration_sec);

    const double dt = 0.05;  // 20Hz
    const int steps = static_cast<int>(duration_sec / dt);

    for (int step = 1; step <= steps; ++step) {
      double t = static_cast<double>(step) / steps;
      // Cosine interpolation (ease-in-out): trigger 1.0 (closed) -> 0.0 (open)
      double alpha = (1.0 - std::cos(t * M_PI)) / 2.0;
      double trigger = 1.0 - alpha;  // 1.0 -> 0.0

      sendGripperCommand("left", trigger);
      sendGripperCommand("right", trigger);

      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    prev_trigger_left_ = 0.0;
    prev_trigger_right_ = 0.0;
    RCLCPP_INFO(node_->get_logger(), "[GripperController] Grippers opened");
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
// Bimanual Teleop Controller (Shared PlanningSceneMonitor)
//==============================================================================

class BimanualTeleop {
public:
  BimanualTeleop(rclcpp::Node::SharedPtr node)
    : node_(node),
      calibrator_left_(1.0),
      calibrator_right_(1.0),
      left_calibrated_(false),
      right_calibrated_(false),
      prev_timestamp_left_(0.0),
      prev_timestamp_right_(0.0)
  {
    RCLCPP_INFO(node_->get_logger(), "[BimanualTeleop] Creating shared PlanningSceneMonitor...");

    // Get left servo params for creating shared PlanningSceneMonitor
    auto left_param_listener = std::make_shared<const servo::ParamListener>(node_, "moveit_servo_left");
    servo::Params left_servo_params = left_param_listener->get_params();

    // Create ONE shared PlanningSceneMonitor
    planning_scene_monitor_ = createPlanningSceneMonitor(node_, left_servo_params);

    RCLCPP_INFO(node_->get_logger(), "[BimanualTeleop] Creating LEFT servo...");
    servo_left_ = std::make_unique<ServoController>(
        node_, "moveit_servo_left", "openarm_left_hand_tcp", planning_scene_monitor_);

    RCLCPP_INFO(node_->get_logger(), "[BimanualTeleop] Creating RIGHT servo...");
    servo_right_ = std::make_unique<ServoController>(
        node_, "moveit_servo_right", "openarm_right_hand_tcp", planning_scene_monitor_);

    RCLCPP_INFO(node_->get_logger(), "[BimanualTeleop] Both servos created with shared PlanningSceneMonitor!");
  }

  bool isReady() const {
    return servo_left_ && servo_left_->isInitialized() &&
           servo_right_ && servo_right_->isInitialized();
  }

  void update(const QuestData& robot_data, double head_yaw_deg = 0.0) {
    auto t0 = std::chrono::steady_clock::now();

    bool do_left = robot_data.left.enabled;
    bool do_right = robot_data.right.enabled;

    // Detect controller re-enable: resync robot state and reset target pose (prevent teleport)
    bool resync_needed = false;
    bool reenable_left = do_left && !prev_enabled_left_ && left_calibrated_;
    bool reenable_right = do_right && !prev_enabled_right_ && right_calibrated_;
    if (reenable_left) resync_needed = true;
    if (reenable_right) resync_needed = true;

    // [DEBUG] Log re-enable detection
    if (reenable_left || reenable_right) {
      RCLCPP_WARN(node_->get_logger(),
          "[RE-ENABLE] L=%s R=%s | prev_en L=%d R=%d | calib L=%d R=%d | quest_ts=%.3f",
          reenable_left ? "YES" : "no", reenable_right ? "YES" : "no",
          prev_enabled_left_, prev_enabled_right_,
          left_calibrated_, right_calibrated_,
          robot_data.timestamp);
      debug_frames_left_ = reenable_left ? 10 : debug_frames_left_;
      debug_frames_right_ = reenable_right ? 10 : debug_frames_right_;
    }

    if (resync_needed) {
      // Update robot_state_ from actual joint states before getting EE pose
      if (reenable_left) servo_left_->resyncToCurrentState();
      if (reenable_right) servo_right_->resyncToCurrentState();
    }

    if (reenable_left) {
      Eigen::Isometry3d ee_before = servo_left_->getCurrentEEPose();
      prev_pos_left_ = robot_data.left.position;
      prev_euler_left_ = robot_data.left.euler;
      prev_timestamp_left_ = robot_data.timestamp;
      target_pose_left_ = servo_left_->getCurrentEEPose();
      Eigen::Vector3d tp = target_pose_left_.translation();
      RCLCPP_WARN(node_->get_logger(),
          "[RE-ENABLE left] quest_pos=(%.4f,%.4f,%.4f) prev_pos=(%.4f,%.4f,%.4f) target_pose=(%.4f,%.4f,%.4f)",
          robot_data.left.position.x(), robot_data.left.position.y(), robot_data.left.position.z(),
          prev_pos_left_.x(), prev_pos_left_.y(), prev_pos_left_.z(),
          tp.x(), tp.y(), tp.z());
    }
    if (reenable_right) {
      prev_pos_right_ = robot_data.right.position;
      prev_euler_right_ = robot_data.right.euler;
      prev_timestamp_right_ = robot_data.timestamp;
      target_pose_right_ = servo_right_->getCurrentEEPose();
      Eigen::Vector3d tp = target_pose_right_.translation();
      RCLCPP_WARN(node_->get_logger(),
          "[RE-ENABLE right] quest_pos=(%.4f,%.4f,%.4f) prev_pos=(%.4f,%.4f,%.4f) target_pose=(%.4f,%.4f,%.4f)",
          robot_data.right.position.x(), robot_data.right.position.y(), robot_data.right.position.z(),
          prev_pos_right_.x(), prev_pos_right_.y(), prev_pos_right_.z(),
          tp.x(), tp.y(), tp.z());
    }
    prev_enabled_left_ = do_left;
    prev_enabled_right_ = do_right;

    // Process both arms in parallel
    if (do_left && do_right) {
      std::thread left_thread([&]() {
        processArm("left", robot_data.left, robot_data.timestamp,
                   calibrator_left_, left_calibrated_,
                   prev_pos_left_, prev_euler_left_, prev_timestamp_left_,
                   *servo_left_, target_pose_left_, pose_initialized_left_,
                   head_yaw_deg);
      });
      processArm("right", robot_data.right, robot_data.timestamp,
                 calibrator_right_, right_calibrated_,
                 prev_pos_right_, prev_euler_right_, prev_timestamp_right_,
                 *servo_right_, target_pose_right_, pose_initialized_right_,
                 head_yaw_deg);
      left_thread.join();
    } else if (do_left) {
      processArm("left", robot_data.left, robot_data.timestamp,
                 calibrator_left_, left_calibrated_,
                 prev_pos_left_, prev_euler_left_, prev_timestamp_left_,
                 *servo_left_, target_pose_left_, pose_initialized_left_,
                 head_yaw_deg);
    } else if (do_right) {
      processArm("right", robot_data.right, robot_data.timestamp,
                 calibrator_right_, right_calibrated_,
                 prev_pos_right_, prev_euler_right_, prev_timestamp_right_,
                 *servo_right_, target_pose_right_, pose_initialized_right_,
                 head_yaw_deg);
    }

    auto t1 = std::chrono::steady_clock::now();

    // Timing log (disabled for debug)
    // static int timing_counter = 0;
    // static double total_sum = 0;
    // static int timing_samples = 0;
    // double total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    // total_sum += total_ms;
    // timing_samples++;
    // if (++timing_counter % 200 == 0) {
    //   RCLCPP_INFO(node_->get_logger(),
    //       "[TIMING] total=%.2f ms (avg over %d) | parallel=%s",
    //       total_sum / timing_samples, timing_samples,
    //       (do_left && do_right) ? "yes" : "no");
    //   total_sum = 0; timing_samples = 0;
    // }
  }

  bool isLeftCalibrated() const { return left_calibrated_; }
  bool isRightCalibrated() const { return right_calibrated_; }

  Eigen::Matrix<double, 6, 1> getLastLeftTwist() const {
    std::lock_guard<std::mutex> lock(twist_mutex_);
    return last_left_twist_;
  }

  Eigen::Matrix<double, 6, 1> getLastRightTwist() const {
    std::lock_guard<std::mutex> lock(twist_mutex_);
    return last_right_twist_;
  }

  // EEF pose getters (world frame absolute pose: xyz + quaternion)
  Eigen::Isometry3d getLastLeftEEFPose() const {
    std::lock_guard<std::mutex> lock(eef_mutex_);
    return last_left_eef_pose_;
  }
  Eigen::Isometry3d getLastRightEEFPose() const {
    std::lock_guard<std::mutex> lock(eef_mutex_);
    return last_right_eef_pose_;
  }

  // EEF delta getters (world frame delta: position xyz + rotation quaternion)
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> getLastLeftEEFDelta() const {
    std::lock_guard<std::mutex> lock(eef_mutex_);
    return {last_left_eef_delta_pos_, last_left_eef_delta_rot_};
  }
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> getLastRightEEFDelta() const {
    std::lock_guard<std::mutex> lock(eef_mutex_);
    return {last_right_eef_delta_pos_, last_right_eef_delta_rot_};
  }

  // Target joint positions getters (Servo IK commanded values)
  std::optional<Eigen::VectorXd> getLastLeftTargetJoints() const {
    std::lock_guard<std::mutex> lock(eef_mutex_);
    return last_left_target_joints_;
  }
  std::optional<Eigen::VectorXd> getLastRightTargetJoints() const {
    std::lock_guard<std::mutex> lock(eef_mutex_);
    return last_right_target_joints_;
  }

  void resyncServoState() {
    if (servo_left_) {
      servo_left_->resyncToCurrentState();
    }
    if (servo_right_) {
      servo_right_->resyncToCurrentState();
    }
    // Reset pose tracking so it re-initializes from current EE pose on next calibration
    pose_initialized_left_ = false;
    pose_initialized_right_ = false;
    left_calibrated_ = false;
    right_calibrated_ = false;
    calibrator_left_.reset();
    calibrator_right_.reset();
  }

private:
  void processArm(const std::string& arm_name,
                  const ControllerData& data,
                  double timestamp,
                  Calibrator& calibrator,
                  bool& calibrated,
                  Eigen::Vector3d& prev_pos,
                  Eigen::Vector3d& prev_euler,
                  double& prev_time,
                  ServoController& servo,
                  Eigen::Isometry3d& target_pose,
                  bool& pose_initialized,
                  double head_yaw_deg = 0.0)
  {
    if (!calibrated) {
      prev_pos = data.position;
      prev_euler = data.euler;
      prev_time = timestamp;
      calibrated = true;
      head_yaw_offset_ = head_yaw_deg;
      target_pose = servo.getCurrentEEPose();
      pose_initialized = true;
      RCLCPP_INFO(node_->get_logger(), "[%s] Calibration complete! head_yaw_offset: %.1f (POSE mode)",
                  arm_name.c_str(), head_yaw_offset_);
      return;
    }

    // Compute position and euler deltas
    Eigen::Vector3d pos_delta = data.position - prev_pos;
    Eigen::Vector3d euler_delta;
    euler_delta.x() = wrapAngleDelta(data.euler.x() - prev_euler.x());
    euler_delta.y() = wrapAngleDelta(data.euler.y() - prev_euler.y());
    euler_delta.z() = wrapAngleDelta(data.euler.z() - prev_euler.z());

    // [DEBUG] Log first 10 frames after re-enable
    int& dbg_frames = (arm_name == "left") ? debug_frames_left_ : debug_frames_right_;
    if (dbg_frames > 0) {
      double dt = timestamp - prev_time;
      RCLCPP_WARN(node_->get_logger(),
          "[DEBUG %s F%d] dt=%.4f | data.pos=(%.4f,%.4f,%.4f) prev_pos=(%.4f,%.4f,%.4f) pos_delta=(%.5f,%.5f,%.5f) euler_delta=(%.3f,%.3f,%.3f)",
          arm_name.c_str(), 11 - dbg_frames, dt,
          data.position.x(), data.position.y(), data.position.z(),
          prev_pos.x(), prev_pos.y(), prev_pos.z(),
          pos_delta.x(), pos_delta.y(), pos_delta.z(),
          euler_delta.x(), euler_delta.y(), euler_delta.z());
    }

    // Compensate position delta for head yaw rotation (horizontal Y-Z plane in robot frame)
    // Robot frame: X=up, Y=right, Z=front. Head yaw rotates around X (vertical).
    double rel_yaw_deg = fmod(head_yaw_deg - head_yaw_offset_ + 180.0, 360.0) - 180.0;
    double rel_yaw = rel_yaw_deg * DEG_TO_RAD;
    double cos_yaw = std::cos(rel_yaw);
    double sin_yaw = std::sin(rel_yaw);
    Eigen::Vector3d corrected_eef;
    corrected_eef.x() = pos_delta.x();  // up (EEF X) unchanged by yaw
    corrected_eef.y() = cos_yaw * pos_delta.y() - sin_yaw * pos_delta.z();
    corrected_eef.z() = sin_yaw * pos_delta.y() + cos_yaw * pos_delta.z();

    // Convert EEF convention (X=up, Y=right, Z=front) to world frame (X=forward, Y=left, Z=up)
    Eigen::Vector3d corrected_world;
    corrected_world.x() = corrected_eef.z();   // forward = front
    corrected_world.y() = -corrected_eef.y();   // left = -right
    corrected_world.z() = corrected_eef.x();    // up = up

    // Transform delta from world frame to planning (link0) frame
    Eigen::Vector3d delta_in_planning = servo.worldToPlanning(corrected_world);

    // Apply position delta to target pose (now in link0 frame)
    Eigen::Vector3d target_before = target_pose.translation();  // [DEBUG]
    target_pose.translation() += delta_in_planning;

    // Apply rotation delta in EEF-relative (body-frame) style:
    // euler_delta is in EEF convention (X=up, Y=right, Z=front)
    // Apply as local rotation on current target orientation (like TWIST mode did)
    double rx = euler_delta.x() * DEG_TO_RAD;
    double ry = euler_delta.y() * DEG_TO_RAD;
    double rz = euler_delta.z() * DEG_TO_RAD;
    Eigen::AngleAxisd rot_x(rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(rz, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d local_rot = (rot_x * rot_y * rot_z).toRotationMatrix();
    target_pose.linear() = target_pose.rotation() * local_rot;

    bool pose_ok = servo.sendPose(target_pose);

    // [DEBUG] Log target_pose and sendPose result for first 10 frames after re-enable
    if (dbg_frames > 0) {
      Eigen::Vector3d target_t = target_pose.translation();
      Eigen::Isometry3d actual_ee = servo.getGlobalEEPose();
      Eigen::Vector3d actual_t = actual_ee.translation();
      RCLCPP_WARN(node_->get_logger(),
          "[DEBUG %s F%d] link0_delta=(%.5f,%.5f,%.5f) target_before=(%.4f,%.4f,%.4f) target_after=(%.4f,%.4f,%.4f) actual_ee=(%.4f,%.4f,%.4f) sendPose=%s",
          arm_name.c_str(), 11 - dbg_frames,
          delta_in_planning.x(), delta_in_planning.y(), delta_in_planning.z(),
          target_before.x(), target_before.y(), target_before.z(),
          target_t.x(), target_t.y(), target_t.z(),
          actual_t.x(), actual_t.y(), actual_t.z(),
          pose_ok ? "OK" : "FAIL");
      dbg_frames--;
    }

    // Store twist (delta-based) for external publishing
    {
      double dt = timestamp - prev_time;
      Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
      if (dt > 0.0 && dt < 1.0) {
        twist(0) = corrected_world.x() / dt;
        twist(1) = corrected_world.y() / dt;
        twist(2) = corrected_world.z() / dt;
        twist(3) = rx / dt;
        twist(4) = ry / dt;
        twist(5) = rz / dt;
      }
      std::lock_guard<std::mutex> lock(twist_mutex_);
      if (arm_name == "left") {
        last_left_twist_ = twist;
      } else {
        last_right_twist_ = twist;
      }
    }

    // Store EEF pose, delta, and target joint positions for data collection
    {
      // World frame absolute EE pose
      Eigen::Isometry3d global_ee = servo.getGlobalEEPose();

      // World frame delta: position is corrected_world, rotation as world-frame quaternion
      // Convert local rotation (EEF-relative) to world frame rotation delta
      // The local rotation was applied as: target_pose.linear() = target_pose.rotation() * local_rot
      // In world frame, this corresponds to: world_rot_delta = R_world * local_rot * R_world^T (similarity transform)
      // But for data collection, we store the rotation delta as a quaternion directly
      Eigen::Matrix3d world_rot_delta = global_ee.rotation() * local_rot * global_ee.rotation().transpose();
      // Normalize to handle numerical drift
      Eigen::Quaterniond delta_quat(world_rot_delta);
      delta_quat.normalize();

      // Target joint positions from Servo IK
      auto target_joints = servo.getLastCommandedJointPositions();

      std::lock_guard<std::mutex> lock(eef_mutex_);
      if (arm_name == "left") {
        last_left_eef_pose_ = global_ee;
        last_left_eef_delta_pos_ = corrected_world;
        last_left_eef_delta_rot_ = delta_quat;
        last_left_target_joints_ = target_joints;
      } else {
        last_right_eef_pose_ = global_ee;
        last_right_eef_delta_pos_ = corrected_world;
        last_right_eef_delta_rot_ = delta_quat;
        last_right_target_joints_ = target_joints;
      }
    }

    prev_pos = data.position;
    prev_euler = data.euler;
    prev_time = timestamp;
  }

  rclcpp::Node::SharedPtr node_;

  // Shared PlanningSceneMonitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Servo controllers
  std::unique_ptr<ServoController> servo_left_;
  std::unique_ptr<ServoController> servo_right_;

  // Calibrators
  Calibrator calibrator_left_;
  Calibrator calibrator_right_;
  bool left_calibrated_;
  bool right_calibrated_;

  // Previous frame data - Left
  Eigen::Vector3d prev_pos_left_;
  Eigen::Vector3d prev_euler_left_;
  double prev_timestamp_left_;

  // Previous frame data - Right
  Eigen::Vector3d prev_pos_right_;
  Eigen::Vector3d prev_euler_right_;
  double prev_timestamp_right_;

  // Previous enabled state for detecting re-enable (prevent teleport)
  bool prev_enabled_left_ = false;
  bool prev_enabled_right_ = false;

  // [DEBUG] Frame counter for re-enable diagnostics
  int debug_frames_left_ = 0;
  int debug_frames_right_ = 0;

  // Head yaw offset for body rotation compensation
  double head_yaw_offset_ = 0.0;

  // Target poses for POSE mode (accumulated from Quest deltas)
  Eigen::Isometry3d target_pose_left_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_pose_right_ = Eigen::Isometry3d::Identity();
  bool pose_initialized_left_ = false;
  bool pose_initialized_right_ = false;

  // Last computed twist for publishing
  mutable std::mutex twist_mutex_;
  Eigen::Matrix<double, 6, 1> last_left_twist_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> last_right_twist_ = Eigen::Matrix<double, 6, 1>::Zero();

  // EEF pose/delta and target joints for data collection
  mutable std::mutex eef_mutex_;
  Eigen::Isometry3d last_left_eef_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_right_eef_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d last_left_eef_delta_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_right_eef_delta_pos_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond last_left_eef_delta_rot_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond last_right_eef_delta_rot_ = Eigen::Quaterniond::Identity();
  std::optional<Eigen::VectorXd> last_left_target_joints_;
  std::optional<Eigen::VectorXd> last_right_target_joints_;
};

//==============================================================================
// Main
//==============================================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("quest_teleop_bimanual", node_options);
  moveit::setNodeLoggerName(node->get_name());

  RCLCPP_INFO(node->get_logger(), "=== Quest VR Teleop for OpenArm (BIMANUAL) ===");
  RCLCPP_INFO(node->get_logger(), "PlanningSceneMonitor: SHARED between both arms");

  // Wait for system to be ready
  RCLCPP_INFO(node->get_logger(), "Waiting 3 seconds for system ready...");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Create bimanual teleop controller
  BimanualTeleop teleop(node);
  if (!teleop.isReady()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize servo controllers!");
    return 1;
  }

  // Start socket server
  QuestSocketServer socket_server("0.0.0.0", 5454);
  if (!socket_server.start()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to start socket server!");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Waiting for Quest VR connection on port 5454...");

  // Set trajectory controller targets to match return_to_zero positions
  HomingController homing(node);
  homing.smoothHomingBoth(0.5);  // Already at position, just sets targets
  teleop.resyncServoState();

  // Create gripper controller and smoothly open grippers
  GripperController gripper(node);
  gripper.openBothSmooth(2.0);  // 2초 동안 부드럽게 열기

  // Publishers for data collection (LeRobot)
  auto left_eef_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/left_eef_pose", 10);
  auto right_eef_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/right_eef_pose", 10);
  auto left_eef_delta_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/left_eef_delta", 10);
  auto right_eef_delta_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/right_eef_delta", 10);
  auto left_target_joints_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/left_target_joint_positions", 10);
  auto right_target_joints_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/right_target_joint_positions", 10);
  auto left_gripper_pub = node->create_publisher<std_msgs::msg::Float64>(
      "/left_gripper_trigger", 10);
  auto right_gripper_pub = node->create_publisher<std_msgs::msg::Float64>(
      "/right_gripper_trigger", 10);
  auto x_button_pub = node->create_publisher<std_msgs::msg::Bool>(
      "/quest_x_button", 10);
  auto rerecord_pub = node->create_publisher<std_msgs::msg::Bool>(
      "/quest_rerecord", 10);
  auto head_euler_pub = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/quest_head_euler", 10);

  // Main loop
  rclcpp::WallRate rate(100.0);
  int log_counter = 0;
  bool homing_triggered = false;  // Prevent repeated triggers
  bool is_homing_active = false;
  auto homing_start_time = std::chrono::steady_clock::now();
  double prev_quest_timestamp = 0.0;
  int quest_new_count = 0;
  int quest_stale_count = 0;
  auto rate_monitor_start = std::chrono::steady_clock::now();
  auto last_new_quest_time = std::chrono::steady_clock::now();
  bool quest_stale_disabled = false;  // true when stale timeout forced enabled=false

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    QuestData quest_raw = socket_server.getLatestData();
    QuestData robot_data = transformQuestData(quest_raw);

    // Check for new Quest data
    bool is_new_quest_data = (quest_raw.timestamp != prev_quest_timestamp);
    if (is_new_quest_data) {
      prev_quest_timestamp = quest_raw.timestamp;
      last_new_quest_time = std::chrono::steady_clock::now();
      if (quest_stale_disabled) {
        RCLCPP_WARN(node->get_logger(), "[STALE] Quest data resumed, L_en=%d R_en=%d",
            robot_data.left.enabled ? 1 : 0, robot_data.right.enabled ? 1 : 0);
        quest_stale_disabled = false;
      }
      quest_new_count++;
    } else {
      quest_stale_count++;
    }

    // RATE monitor (disabled for debug)
    // auto rate_now = std::chrono::steady_clock::now();
    // double rate_elapsed = std::chrono::duration<double>(rate_now - rate_monitor_start).count();
    // if (rate_elapsed >= 3.0) {
    //   int total = quest_new_count + quest_stale_count;
    //   double quest_hz = quest_new_count / rate_elapsed;
    //   double loop_hz = total / rate_elapsed;
    //   double stale_pct = (total > 0) ? (100.0 * quest_stale_count / total) : 0.0;
    //   RCLCPP_INFO(node->get_logger(),
    //       "[RATE] Quest=%.1f Hz | Loop=%.1f Hz | Stale=%.1f%% | L_en=%d R_en=%d",
    //       quest_hz, loop_hz, stale_pct,
    //       robot_data.left.enabled ? 1 : 0, robot_data.right.enabled ? 1 : 0);
    //   quest_new_count = 0;
    //   quest_stale_count = 0;
    //   rate_monitor_start = rate_now;
    // }

    // Check for homing trigger: right joystick right (agv_x > 0.8) && left joystick down (lift < -0.8)
    bool homing_condition = (quest_raw.agv_x > 0.8) && (quest_raw.lift < -0.8);

    if (homing_condition && !homing_triggered) {
      homing_triggered = true;
      RCLCPP_INFO(node->get_logger(), "[HOMING] Joystick trigger detected! Starting smooth homing...");

      // Execute smooth homing (2 seconds), grippers stay in current position
      homing.smoothHomingBoth(2.0);

      // Set homing flag - main loop will publish home positions until homing completes
      is_homing_active = true;
      homing_start_time = std::chrono::steady_clock::now();

      RCLCPP_INFO(node->get_logger(), "[HOMING] Homing started, publishing home positions for 2s...");
    }

    // Reset trigger when joysticks return to neutral
    if (quest_raw.agv_x < 0.3 && quest_raw.lift > -0.3) {
      homing_triggered = false;
    }

    // Check if homing duration has elapsed
    if (is_homing_active) {
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - homing_start_time).count();
      if (elapsed_ms >= 2000) {
        is_homing_active = false;
        teleop.resyncServoState();
        RCLCPP_INFO(node->get_logger(), "[HOMING] Homing complete, resuming teleop");
      }
    }

    // Episode control: left joystick up + right joystick left/right
    // Left stick up (lift > 0.8) + Right stick right (agv_x > 0.8) = next episode
    // Left stick up (lift > 0.8) + Right stick left (agv_x < -0.8) = rerecord episode
    static bool episode_next_triggered = false;
    static bool episode_rerecord_triggered = false;

    if (quest_raw.lift > 0.8 && quest_raw.agv_x > 0.8 && !episode_next_triggered) {
      episode_next_triggered = true;
      RCLCPP_INFO(node->get_logger(), "[EPISODE] Next episode triggered (lift up + agv right)");
      std_msgs::msg::Bool msg;
      msg.data = true;
      x_button_pub->publish(msg);
    }
    if (quest_raw.lift > 0.8 && quest_raw.agv_x < -0.8 && !episode_rerecord_triggered) {
      episode_rerecord_triggered = true;
      RCLCPP_INFO(node->get_logger(), "[EPISODE] Rerecord triggered (lift up + agv left)");
      std_msgs::msg::Bool msg;
      msg.data = true;
      rerecord_pub->publish(msg);
    }
    // Reset episode triggers when BOTH joysticks return to neutral (AND, not OR)
    if (quest_raw.lift < 0.3 && (quest_raw.agv_x > -0.3 && quest_raw.agv_x < 0.3)) {
      episode_next_triggered = false;
      episode_rerecord_triggered = false;
    }

    // Stale timeout: if no new Quest data for 50ms, force both controllers disabled
    // This handles the case where both triggers are released and Quest stops sending data,
    // so the last-released controller's enabled=false is never transmitted.
    auto stale_elapsed = std::chrono::steady_clock::now() - last_new_quest_time;
    if (stale_elapsed > std::chrono::milliseconds(50) && !quest_stale_disabled) {
      if (robot_data.left.enabled || robot_data.right.enabled) {
        RCLCPP_WARN(node->get_logger(),
            "[STALE] No Quest data for >50ms, forcing both enabled=false (was L=%d R=%d)",
            robot_data.left.enabled ? 1 : 0, robot_data.right.enabled ? 1 : 0);
        robot_data.left.enabled = false;
        robot_data.right.enabled = false;
        // Run one update with forced disabled so prev_enabled flags are cleared
        // Skip during homing to avoid cancelling the homing trajectory
        if (!is_homing_active) {
          teleop.update(robot_data, quest_raw.head.euler.y());
        }
        quest_stale_disabled = true;
      }
    }

    // Only update when new Quest data arrives (skip stale data to prevent stuttering)
    // Also skip during homing to avoid Servo overriding the homing trajectory
    if (is_new_quest_data && !is_homing_active) {
      teleop.update(robot_data, quest_raw.head.euler.y());
    }

    // Update grippers
    if (robot_data.left.enabled || robot_data.right.enabled) {
      gripper.update(
          robot_data.left.enabled ? robot_data.left.trigger : 0.0,
          robot_data.right.enabled ? robot_data.right.trigger : 0.0);
    }

    // Publish EEF pose, delta, and target joint positions for data collection
    auto now = node->now();
    {
      // EEF absolute pose (world frame)
      auto left_pose = teleop.getLastLeftEEFPose();
      auto right_pose = teleop.getLastRightEEFPose();

      auto publish_pose = [&now](auto& pub, const Eigen::Isometry3d& pose, const std::string& frame) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now;
        msg.header.frame_id = frame;
        msg.pose.position.x = pose.translation().x();
        msg.pose.position.y = pose.translation().y();
        msg.pose.position.z = pose.translation().z();
        Eigen::Quaterniond q(pose.rotation());
        q.normalize();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        pub->publish(msg);
      };
      publish_pose(left_eef_pose_pub, left_pose, "world");
      publish_pose(right_eef_pose_pub, right_pose, "world");

      // EEF delta (world frame position delta + rotation delta as quaternion)
      // Zero out delta when controller is disabled (not sending commands)
      auto [left_delta_pos, left_delta_rot] = teleop.getLastLeftEEFDelta();
      auto [right_delta_pos, right_delta_rot] = teleop.getLastRightEEFDelta();
      if (!robot_data.left.enabled) {
        left_delta_pos = Eigen::Vector3d::Zero();
        left_delta_rot = Eigen::Quaterniond::Identity();
      }
      if (!robot_data.right.enabled) {
        right_delta_pos = Eigen::Vector3d::Zero();
        right_delta_rot = Eigen::Quaterniond::Identity();
      }

      auto publish_delta = [&now](auto& pub, const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now;
        msg.header.frame_id = "world";
        msg.pose.position.x = pos.x();
        msg.pose.position.y = pos.y();
        msg.pose.position.z = pos.z();
        msg.pose.orientation.x = rot.x();
        msg.pose.orientation.y = rot.y();
        msg.pose.orientation.z = rot.z();
        msg.pose.orientation.w = rot.w();
        pub->publish(msg);
      };
      publish_delta(left_eef_delta_pub, left_delta_pos, left_delta_rot);
      publish_delta(right_eef_delta_pub, right_delta_pos, right_delta_rot);

      // Target joint positions (Servo IK commanded values)
      if (is_homing_active) {
        // During homing: publish home positions so they appear in the dataset
        static const std::vector<double> home_joints = {0.0, 0.0, 0.0, 1.58, 0.0, 0.0, 0.0};
        std_msgs::msg::Float64MultiArray home_msg;
        home_msg.data = home_joints;
        left_target_joints_pub->publish(home_msg);
        right_target_joints_pub->publish(home_msg);
      } else {
        auto left_joints = teleop.getLastLeftTargetJoints();
        auto right_joints = teleop.getLastRightTargetJoints();

        auto publish_joints = [](auto& pub, const std::optional<Eigen::VectorXd>& joints) {
          std_msgs::msg::Float64MultiArray msg;
          if (joints.has_value()) {
            msg.data.assign(joints->data(), joints->data() + joints->size());
          }
          pub->publish(msg);
        };
        publish_joints(left_target_joints_pub, left_joints);
        publish_joints(right_target_joints_pub, right_joints);
      }
    }

    // Publish gripper trigger values
    {
      std_msgs::msg::Float64 left_grip_msg;
      left_grip_msg.data = robot_data.left.enabled ? robot_data.left.trigger : 0.0;
      left_gripper_pub->publish(left_grip_msg);

      std_msgs::msg::Float64 right_grip_msg;
      right_grip_msg.data = robot_data.right.enabled ? robot_data.right.trigger : 0.0;
      right_gripper_pub->publish(right_grip_msg);
    }

    // Publish Quest X button state (only publish true, never overwrite with false)
    if (quest_raw.x_button) {
      std_msgs::msg::Bool x_msg;
      x_msg.data = true;
      x_button_pub->publish(x_msg);
    }

    // Publish head euler (degrees) for neck motor control
    if (quest_raw.head.valid) {
      geometry_msgs::msg::Vector3Stamped head_msg;
      head_msg.header.stamp = now;
      head_msg.header.frame_id = "quest_head";
      head_msg.vector.x = quest_raw.head.euler.x();  // pitch (up/down)
      head_msg.vector.y = quest_raw.head.euler.y();  // yaw (left/right)
      head_msg.vector.z = quest_raw.head.euler.z();  // roll
      head_euler_pub->publish(head_msg);
    }

    // Log counter (kept for other uses)
    ++log_counter;

    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Quest teleop bimanual finished.");
  rclcpp::shutdown();
  return 0;
}
