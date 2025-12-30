/*******************************************************************************
 * 프로젝트   : moveit_servo 0.5초 간격 고속 방향 전환 데모
 * 수정 사항   : 0.5초마다 8가지의 서로 다른 복합 속도 프로파일로 교체
 *******************************************************************************/

#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/utils/logger.hpp>

using namespace moveit_servo;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");
  moveit::setNodeLoggerName(demo_node->get_name());

  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
      demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                         rclcpp::SystemDefaultsQoS());

  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);

  servo.setCommandType(CommandType::TWIST);

  rclcpp::WallRate rate(1.0 / servo_params.publish_period);
  
  // 총 실행 시간 15초
  std::chrono::seconds timeout_duration(15); 
  const auto start_time = std::chrono::steady_clock::now();

  std::deque<KinematicState> joint_cmd_rolling_window;
  KinematicState current_state = servo.getCurrentRobotState(true);
  updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());

  RCLCPP_INFO(demo_node->get_logger(), "--- MoveIt Servo 0.5s Fast-Switch Demo Started ---");

  while (rclcpp::ok())
  {
    const auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = current_time - start_time;
    double t = elapsed.count();

    if (t >= timeout_duration.count()) break;

    TwistCommand target_twist;
    target_twist.frame_id = "panda_link0";
    target_twist.velocities.setZero(); 

    // 0.5초 간격으로 세그먼트 계산 (0, 1, 2, 3...)
    // 8로 나눈 나머지를 사용하여 8가지 동작을 무한 반복
    int segment = static_cast<int>(t / 0.5) % 8;

    /*  */
    /* 인덱스 가이드: 0:LX, 1:LY, 2:LZ, 3:AX, 4:AY, 5:AZ */
    switch(segment) {
      case 0: // 대각선 상방 우측
        target_twist.velocities[0] = 0.04; target_twist.velocities[1] = 0.04; target_twist.velocities[2] = 0.04;
        break;
      case 1: // 제자리 회전 (Yaw)
        target_twist.velocities[5] = 0.6;
        break;
      case 2: // 전진 및 하강
        target_twist.velocities[0] = 0.05; target_twist.velocities[2] = -0.04;
        break;
      case 3: // 손목 꺾기 (Pitch)
        target_twist.velocities[4] = 0.4;
        break;
      case 4: // 왼쪽 후진 상방
        target_twist.velocities[0] = -0.04; target_twist.velocities[1] = -0.04; target_twist.velocities[2] = 0.03;
        break;
      case 5: // 복합 회전 (Roll + Yaw)
        target_twist.velocities[3] = 0.3; target_twist.velocities[5] = -0.3;
        break;
      case 6: // 오른쪽 수평 이동
        target_twist.velocities[1] = 0.06;
        break;
      case 7: // 급속 하강 및 복귀준비
        target_twist.velocities[2] = -0.05; target_twist.velocities[0] = -0.02;
        break;
    }

    KinematicState joint_state = servo.getNextJointState(robot_state, target_twist);
    const StatusCode status = servo.getStatus();

    if (status != StatusCode::INVALID)
    {
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
    rate.sleep();
  }

  RCLCPP_INFO(demo_node->get_logger(), "Fast-switch demo finished.");
  rclcpp::shutdown();
  return 0;
}