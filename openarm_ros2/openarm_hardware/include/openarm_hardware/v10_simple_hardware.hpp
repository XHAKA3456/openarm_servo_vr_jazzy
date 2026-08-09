// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <memory>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarm_hardware/gravity_comp.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

/**
 * @brief Simplified OpenArm V10 Hardware Interface
 *
 * This is a simplified version that uses the OpenArm CAN API directly,
 * following the pattern from full_arm.cpp example. Much simpler than
 * the original implementation.
 */
class OpenArm_v10HW : public hardware_interface::SystemInterface {
 public:
  OpenArm_v10HW();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // V10 default configuration
  static constexpr size_t ARM_DOF = 7;
  static constexpr bool ENABLE_GRIPPER = true;

  // Default motor configuration for V10
  const std::vector<openarm::damiao_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      openarm::damiao_motor::MotorType::DM8009,  // Joint 1
      openarm::damiao_motor::MotorType::DM8009,  // Joint 2
      openarm::damiao_motor::MotorType::DM4340,  // Joint 3
      openarm::damiao_motor::MotorType::DM4340,  // Joint 4
      openarm::damiao_motor::MotorType::DM4310,  // Joint 5
      openarm::damiao_motor::MotorType::DM4310,  // Joint 6
      openarm::damiao_motor::MotorType::DM4310   // Joint 7
  };

  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05, 0x06, 0x07};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14,
                                                      0x15, 0x16, 0x17};

  const openarm::damiao_motor::MotorType DEFAULT_GRIPPER_MOTOR_TYPE =
      openarm::damiao_motor::MotorType::DM4310;
  const uint32_t DEFAULT_GRIPPER_SEND_CAN_ID = 0x08;
  const uint32_t DEFAULT_GRIPPER_RECV_CAN_ID = 0x18;

  // Default gains
  // #11 중력보상 도입 후 실기 검증된 게인 (2026-08-07).
  // 중력보상 이전에는 kp만으로 중력을 버텨야 해서 {180,130,130,180,25,25,25}까지
  // 올려야 했고(그래도 처짐), 그 고강성이 정지 buzz와 저속 스틱슬립의 원인이었다.
  // 이제 tau_ff가 무게를 들어주므로 kp는 추종만 담당 → 6분의 1로 낮춰도 처지지 않고,
  // 밀면 밀리고 놓으면 돌아오는 컴플라이언트 거동을 얻는다.
  // 되돌리려면 launch 인자 arm_kp/arm_kd로 덮어쓰면 된다(재빌드 불필요).
  //   중력보상 전 값: KP {180,130,130,180,25,25,25} / KD {5,4,2,3,1.3,1.3,1.5}
  //   공장 초기값   : KP {20,20,20,20,5,5,5}        / KD {2.75,2.5,0.7,0.4,0.7,0.6,0.5}
  const std::vector<double> DEFAULT_KP = {30.0, 25.0, 25.0, 30.0,
                                          6.0,  6.0,  6.0,  0.5};
  // MIT 인코딩 상한: kd <= 5.0
  const std::vector<double> DEFAULT_KD = {2.75, 2.5, 0.7, 0.6,
                                          0.7,  0.6, 0.5, 0.1};
  // 실제 사용 게인 (기본=DEFAULT, arm_kp/arm_kd 파라미터로 덮어씀)
  std::vector<double> kp_;
  std::vector<double> kd_;

  const double GRIPPER_JOINT_0_POSITION = 0.044;
  const double GRIPPER_JOINT_1_POSITION = 0.0;
  const double GRIPPER_MOTOR_0_RADIANS = 0.0;
  const double GRIPPER_MOTOR_1_RADIANS = -1.0472;
  const double GRIPPER_DEFAULT_KP = 5.0;
  const double GRIPPER_DEFAULT_KD = 0.1;

  // Configuration
  std::string can_interface_;
  std::string arm_prefix_;
  bool hand_;
  bool can_fd_;

  // #11 중력보상 (tau_ff): KDL로 G(q) 계산해 MIT 명령의 feedforward 토크에 주입.
  // scale 0.0 = 주입 안 함(현행 동작). 모델은 로드되면 항상 계산되어 진단 로그에 나온다.
  std::string gravity_urdf_path_;
  double gravity_comp_scale_ = 0.0;
  GravityComp gravity_comp_;
  std::vector<double> gravity_tau_;
  size_t gravity_log_counter_ = 0;
  // 관절별 tau_ff 클램프 [Nm] (URDF effort 40/40/27/27/7/7/7의 ~75%)
  const std::vector<double> GRAVITY_TAU_CLAMP = {30.0, 30.0, 20.0, 20.0,
                                                 5.0,  5.0,  5.0};

  // #11-① 마찰보상: tau_fric(v) = Fc·tanh(0.1·k·v) + Fv·v + Fo (레퍼런스 tanh 모델).
  // 계수는 references/openarm_teleop config/follower.yaml의 실측 식별값(관절 1..7).
  // 같은 모델 팔이지만 개체차가 있으므로 friction_comp_scale(기본 0)로 0.3부터 단계 인가.
  // tanh 내부의 0.1 배율은 정지 근처 속도 노이즈로 보상이 널뛰는 것(=buzz)을 막는
  // 레퍼런스의 의도적 소프트닝 — 유지할 것.
  double friction_comp_scale_ = 0.0;
  const std::vector<double> FRIC_FC = {0.306, 0.306, 0.40, 0.166, 0.050, 0.093, 0.172};
  const std::vector<double> FRIC_K  = {28.417, 28.417, 29.065, 130.038, 151.771, 242.287, 7.888};
  const std::vector<double> FRIC_FV = {0.063, 0.063, 0.604, 0.813, 0.029, 0.072, 0.084};
  const std::vector<double> FRIC_FO = {0.088, 0.088, 0.008, -0.058, 0.005, 0.009, -0.059};

  // OpenArm instance
  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

  // Generated joint names for this arm instance
  std::vector<std::string> joint_names_;

  // ROS2 control state and command vectors
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  // Helper methods
  void return_to_zero();
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void generate_joint_names();

  // Gripper mapping functions
  double joint_to_motor_radians(double joint_value);
  double motor_radians_to_joint(double motor_radians);
};

}  // namespace openarm_hardware
