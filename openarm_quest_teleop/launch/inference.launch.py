import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def generate_launch_description():
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware (simulation) or real hardware'
    )
    left_can_interface_arg = DeclareLaunchArgument(
        'left_can_interface',
        default_value='can1',
        description='CAN interface for left arm'
    )
    right_can_interface_arg = DeclareLaunchArgument(
        'right_can_interface',
        default_value='can0',
        description='CAN interface for right arm'
    )

    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    left_can_interface = LaunchConfiguration('left_can_interface')
    right_can_interface = LaunchConfiguration('right_can_interface')

    moveit_config = (
        MoveItConfigsBuilder("openarm_bimanual")
        .robot_description(
            file_path="config/openarm_bimanual.urdf.xacro",
            mappings={
                "ros2_control": "true",
                "use_fake_hardware": use_fake_hardware,
                "left_can_interface": left_can_interface,
                "right_can_interface": right_can_interface,
                "bimanual": "true",
            }
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("openarm_quest_teleop"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    container = ComposableNodeContainer(
        name="robot_state_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
        ],
        output="log",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "300",
            "--controller-manager", "/controller_manager",
        ],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_joint_trajectory_controller",
            "--controller-manager-timeout", "300",
            "-c", "/controller_manager",
        ],
    )

    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_gripper_controller",
            "--controller-manager-timeout", "300",
            "-c", "/controller_manager",
        ],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_joint_trajectory_controller",
            "--controller-manager-timeout", "300",
            "-c", "/controller_manager",
        ],
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_gripper_controller",
            "--controller-manager-timeout", "300",
            "-c", "/controller_manager",
        ],
    )

    camera_streamer_node = Node(
        package='openarm_quest_teleop',
        executable='camera_tcp_streamer.py',
        name='camera_tcp_streamer',
        parameters=[{
            'serial_number': '348522076238',
            'port': 5656,
            'width': 640,
            'height': 480,
            'fps': 30,
            'jpeg_quality': 70,
            'use_depth': True,
            'depth_min_m': 0.4,
            'depth_max_m': 1.1,
            'stream_to_quest': False,  # 추론 시 Quest 스트리밍 불필요
        }],
        output='screen',
    )

    homing_node = Node(
        package='openarm_quest_teleop',
        executable='homing_node.py',
        name='homing_node',
        output='screen',
    )

    return LaunchDescription([
        use_fake_hardware_arg,
        left_can_interface_arg,
        right_can_interface_arg,
        ros2_control_node,
        container,
        joint_state_broadcaster_spawner,
        left_arm_controller_spawner,
        left_gripper_controller_spawner,
        right_arm_controller_spawner,
        right_gripper_controller_spawner,
        homing_node,
        camera_streamer_node,
    ])
