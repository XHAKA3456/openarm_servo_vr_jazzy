cmake_minimum_required(VERSION 3.8)
project(openarm_quest_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Quest Teleop executable
add_executable(quest_teleop src/quest_teleop.cpp)
target_include_directories(quest_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(quest_teleop PUBLIC cxx_std_17)
ament_target_dependencies(quest_teleop
  rclcpp
  rclcpp_action
  moveit_servo
  moveit_core
  moveit_ros_planning_interface
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(quest_teleop nlohmann_json::nlohmann_json)

# Quest Teleop Bimanual executable
add_executable(quest_teleop_bimanual src/quest_teleop_bimanual.cpp)
target_include_directories(quest_teleop_bimanual PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(quest_teleop_bimanual PUBLIC cxx_std_17)
ament_target_dependencies(quest_teleop_bimanual
  rclcpp
  rclcpp_action
  moveit_servo
  moveit_core
  moveit_ros_planning_interface
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(quest_teleop_bimanual nlohmann_json::nlohmann_json)

install(TARGETS quest_teleop quest_teleop_bimanual
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
