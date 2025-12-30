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

//==============================================================================
// Quest Data Structures
//==============================================================================

struct ControllerData {
  Eigen::Vector3d position{0, 0, 0};
  Eigen::Vector3d euler{0, 0, 0};  // x, y, z in degrees (rotation around each axis)
  double trigger{0.0};
  bool enabled{false};
};

struct QuestData {
  ControllerData left;
  ControllerData right;
  double timestamp{0.0};
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

    // Set command type to TWIST
    servo_->setCommandType(CommandType::TWIST);

    // Initialize sliding window
    KinematicState current_state = servo_->getCurrentRobotState(true);
    updateSlidingWindow(current_state, joint_cmd_rolling_window_,
                        servo_params_.max_expected_latency, node_->now());

    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "[ServoController] DONE! Initialized for %s", servo_params_.move_group_name.c_str());
  }

  bool isInitialized() const { return initialized_; }

  bool sendVelocity(const Eigen::Vector3d& linear_vel, const Eigen::Vector3d& angular_vel) {
    if (!initialized_) return false;

    TwistCommand twist;
    twist.frame_id = frame_id_;
    twist.velocities.setZero();
    twist.velocities[0] = linear_vel.x();
    twist.velocities[1] = linear_vel.y();
    twist.velocities[2] = linear_vel.z();
    twist.velocities[3] = angular_vel.x();
    twist.velocities[4] = angular_vel.y();
    twist.velocities[5] = angular_vel.z();

    KinematicState joint_state = servo_->getNextJointState(robot_state_, twist);
    const StatusCode status = servo_->getStatus();

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

  void stop() {
    sendVelocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
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
      gripper_max_(0.044),
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

  void update(const QuestData& robot_data) {
    // Process left arm
    if (robot_data.left.enabled) {
      processArm("left", robot_data.left, robot_data.timestamp,
                 calibrator_left_, left_calibrated_,
                 prev_pos_left_, prev_euler_left_, prev_timestamp_left_,
                 *servo_left_);
    }

    // Process right arm
    if (robot_data.right.enabled) {
      processArm("right", robot_data.right, robot_data.timestamp,
                 calibrator_right_, right_calibrated_,
                 prev_pos_right_, prev_euler_right_, prev_timestamp_right_,
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
                  Eigen::Vector3d& prev_euler,
                  double& prev_time,
                  ServoController& servo)
  {
    if (!calibrated) {
      if (calibrator.update(data, timestamp)) {
        prev_pos = calibrator.getAvgPosition();
        prev_euler = calibrator.getAvgEuler();
        prev_time = timestamp;
        calibrated = true;
        RCLCPP_INFO(node_->get_logger(), "[%s] Calibration complete! Samples: %zu",
                    arm_name.c_str(), calibrator.getSampleCount());
      }
      return;
    }

    Eigen::Vector3d linear_vel, angular_vel;
    calculateVelocity(prev_pos, prev_euler, prev_time,
                      data.position, data.euler, timestamp,
                      linear_vel, angular_vel);

    servo.sendVelocity(linear_vel, angular_vel);

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

  // Execute homing
  HomingController homing(node);
  homing.executeHoming();
  teleop.resyncServoState();

  // Create gripper controller and smoothly open grippers
  GripperController gripper(node);
  gripper.openBothSmooth(2.0);  // 2초 동안 부드럽게 열기

  // Main loop
  rclcpp::WallRate rate(100.0);
  int log_counter = 0;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    QuestData quest_raw = socket_server.getLatestData();
    QuestData robot_data = transformQuestData(quest_raw);

    teleop.update(robot_data);

    // Update grippers
    if (robot_data.left.enabled || robot_data.right.enabled) {
      gripper.update(
          robot_data.left.enabled ? robot_data.left.trigger : 0.0,
          robot_data.right.enabled ? robot_data.right.trigger : 0.0);
    }

    // Log every second
    if (++log_counter % 100 == 0) {
      RCLCPP_INFO(node->get_logger(),
        "Socket: %s | L: cal=%d en=%d | R: cal=%d en=%d",
        socket_server.isConnected() ? "connected" : "waiting",
        teleop.isLeftCalibrated(), robot_data.left.enabled,
        teleop.isRightCalibrated(), robot_data.right.enabled);
    }

    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Quest teleop bimanual finished.");
  rclcpp::shutdown();
  return 0;
}
