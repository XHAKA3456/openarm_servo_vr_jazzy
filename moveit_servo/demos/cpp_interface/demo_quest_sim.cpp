/*******************************************************************************
 * Title     : demo_quest_sim.cpp
 * Project   : moveit_servo
 * Purpose   : Simulate Quest VR input (position/rotation -> velocity conversion)
 *             Tests the same logic as quest_servo_both.py but in C++ with Servo API
 *******************************************************************************/

#include <chrono>
#include <cmath>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/utils/logger.hpp>
#include <Eigen/Geometry>

using namespace moveit_servo;

// Simulated Quest data structure
struct QuestData {
  Eigen::Vector3d position;
  Eigen::Quaterniond rotation;
  double timestamp;
};

// Simulate Quest controller movement (sin/cos pattern)
QuestData simulateQuestData(double t) {
  QuestData data;

  // Simulate smooth position movement (circular pattern in XY plane + Z oscillation)
  double radius = 0.05;  // 5cm radius
  double freq = 0.5;     // 0.5 Hz (2 second period)

  data.position.x() = radius * std::sin(2 * M_PI * freq * t);
  data.position.y() = radius * std::cos(2 * M_PI * freq * t);
  data.position.z() = 0.02 * std::sin(2 * M_PI * freq * 2 * t);  // Z oscillates faster

  // Simulate rotation (slow rotation around Z axis)
  double angle = 0.3 * std::sin(2 * M_PI * freq * 0.5 * t);  // ±0.3 rad oscillation
  data.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

  data.timestamp = t;

  return data;
}

// Calculate velocity from position/rotation difference (same as Python logic)
void calculateVelocity(const QuestData& prev, const QuestData& curr,
                       Eigen::Vector3d& linear_vel, Eigen::Vector3d& angular_vel) {
  double dt = curr.timestamp - prev.timestamp;

  if (dt <= 0.0 || dt > 1.0) {
    linear_vel.setZero();
    angular_vel.setZero();
    return;
  }

  // Linear velocity: (current_pos - prev_pos) / dt
  linear_vel = (curr.position - prev.position) / dt;

  // Angular velocity from quaternion difference
  // delta_rot = curr_rot * prev_rot.inverse()
  Eigen::Quaterniond delta_rot = curr.rotation * prev.rotation.inverse();

  // Convert to rotation vector (axis * angle)
  Eigen::AngleAxisd angle_axis(delta_rot);
  angular_vel = angle_axis.axis() * angle_axis.angle() / dt;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  // IMPORTANT: Use same node name as demo_twist for proper topic naming
  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");
  moveit::setNodeLoggerName(demo_node->get_name());

  // Get servo parameters
  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  // Publisher for trajectory
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
      demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                         rclcpp::SystemDefaultsQoS());

  // Create Servo
  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  // Wait for rviz
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Get robot state
  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);

  // Set command type to TWIST
  servo.setCommandType(CommandType::TWIST);

  // Rate control (same as demo_twist.cpp)
  rclcpp::WallRate rate(1.0 / servo_params.publish_period);

  // Timeout
  std::chrono::seconds timeout_duration(20);  // 20 seconds demo
  const auto start_time = std::chrono::steady_clock::now();

  // Sliding window for smooth trajectory
  std::deque<KinematicState> joint_cmd_rolling_window;
  KinematicState current_state = servo.getCurrentRobotState(true);
  updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());

  RCLCPP_INFO(demo_node->get_logger(), "=== Quest VR Simulation Demo Started ===");
  RCLCPP_INFO(demo_node->get_logger(), "Simulating position/rotation -> velocity conversion");
  RCLCPP_INFO(demo_node->get_logger(), "Same logic as quest_servo_both.py");

  // Initialize previous Quest data
  QuestData prev_quest = simulateQuestData(0.0);

  while (rclcpp::ok())
  {
    const auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = current_time - start_time;
    double t = elapsed.count();

    if (t >= timeout_duration.count()) {
      RCLCPP_INFO(demo_node->get_logger(), "Timeout reached");
      break;
    }

    // Simulate Quest data (like receiving from socket)
    QuestData curr_quest = simulateQuestData(t);

    // Calculate velocity from position/rotation difference
    // THIS IS THE SAME LOGIC AS quest_servo_both.py
    Eigen::Vector3d linear_vel, angular_vel;
    calculateVelocity(prev_quest, curr_quest, linear_vel, angular_vel);

    // Create TwistCommand (same pattern as demo_twist.cpp)
    TwistCommand target_twist;
    target_twist.frame_id = "panda_link0";
    target_twist.velocities.setZero();  // Initialize first!
    target_twist.velocities[0] = linear_vel.x();   // vx
    target_twist.velocities[1] = linear_vel.y();   // vy
    target_twist.velocities[2] = linear_vel.z();   // vz
    target_twist.velocities[3] = angular_vel.x();  // wx
    target_twist.velocities[4] = angular_vel.y();  // wy
    target_twist.velocities[5] = angular_vel.z();  // wz

    // Log every 1 second
    static int log_counter = 0;
    if (++log_counter % 100 == 0) {
      RCLCPP_INFO(demo_node->get_logger(),
        "t=%.1f | pos=(%.3f, %.3f, %.3f) | vel=(%.3f, %.3f, %.3f)",
        t, curr_quest.position.x(), curr_quest.position.y(), curr_quest.position.z(),
        linear_vel.x(), linear_vel.y(), linear_vel.z());
    }

    // Get next joint state from Servo (C++ API direct call!)
    KinematicState joint_state = servo.getNextJointState(robot_state, target_twist);
    const StatusCode status = servo.getStatus();

    if (status != StatusCode::INVALID)
    {
      // Update sliding window and publish trajectory
      updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
      if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
      {
        trajectory_outgoing_cmd_pub->publish(msg.value());
      }

      if (!joint_cmd_rolling_window.empty())
      {
        robot_state->setJointGroupPositions(joint_model_group, joint_cmd_rolling_window.back().positions);
        robot_state->setJointGroupVelocities(joint_model_group, joint_cmd_rolling_window.back().velocities);
      }
    }

    // Update previous Quest data
    prev_quest = curr_quest;

    // Precise rate control (same as demo_twist.cpp)
    rate.sleep();
  }

  RCLCPP_INFO(demo_node->get_logger(), "Quest VR simulation demo finished.");
  rclcpp::shutdown();
  return 0;
}
