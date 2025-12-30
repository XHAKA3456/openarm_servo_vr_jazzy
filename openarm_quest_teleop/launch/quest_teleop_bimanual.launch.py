import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Declare launch arguments
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

    # MoveIt config for bimanual
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

    # Servo parameters for both arms
    servo_yaml_left = load_yaml("openarm_quest_teleop", "config/openarm_left_simulated_config.yaml")
    servo_params_left = {"moveit_servo_left": servo_yaml_left}

    servo_yaml_right = load_yaml("openarm_quest_teleop", "config/openarm_right_simulated_config.yaml")
    servo_params_right = {"moveit_servo_right": servo_yaml_right}

    # Acceleration filter parameters
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name_left = {"planning_group_name": "left_arm"}

    # RViz
    rviz_config_file = (
        get_package_share_directory("openarm_quest_teleop") + "/config/openarm_servo_bimanual.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # ros2_control
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Left arm controller
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_joint_trajectory_controller",
            "--controller-manager-timeout",
            "300",
            "-c",
            "/controller_manager",
        ],
    )

    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_gripper_controller",
            "--controller-manager-timeout",
            "300",
            "-c",
            "/controller_manager",
        ],
    )

    # Right arm controller
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_joint_trajectory_controller",
            "--controller-manager-timeout",
            "300",
            "-c",
            "/controller_manager",
        ],
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_gripper_controller",
            "--controller-manager-timeout",
            "300",
            "-c",
            "/controller_manager",
        ],
    )

    # Robot state publisher
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
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
        output="screen",
    )

    # Quest Teleop Bimanual Node
    quest_teleop_bimanual_node = Node(
        package="openarm_quest_teleop",
        executable="quest_teleop_bimanual",
        name="quest_teleop_bimanual",
        parameters=[
            servo_params_left,
            servo_params_right,
            acceleration_filter_update_period,
            planning_group_name_left,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
        output="screen",
    )

    # Wait for all joint states before starting quest_teleop
    required_joints = [
        "openarm_left_joint1",
        "openarm_left_joint2",
        "openarm_left_joint3",
        "openarm_left_joint4",
        "openarm_left_joint5",
        "openarm_left_joint6",
        "openarm_left_joint7",
        "openarm_right_joint1",
        "openarm_right_joint2",
        "openarm_right_joint3",
        "openarm_right_joint4",
        "openarm_right_joint5",
        "openarm_right_joint6",
        "openarm_right_joint7",
    ]
    wait_for_joint_states = ExecuteProcess(
        cmd=[
            "python3",
            "-c",
            (
                "import rclpy\n"
                "from rclpy.node import Node\n"
                "from sensor_msgs.msg import JointState\n"
                "required = set(" + repr(required_joints) + ")\n"
                "class WaitNode(Node):\n"
                "    def __init__(self):\n"
                "        super().__init__('wait_for_joint_states')\n"
                "        self.create_subscription(JointState, '/joint_states', self.cb, 10)\n"
                "    def cb(self, msg):\n"
                "        if required.issubset(set(msg.name)):\n"
                "            self.get_logger().info('Required joint states received.')\n"
                "            rclpy.shutdown()\n"
                "rclpy.init()\n"
                "node = WaitNode()\n"
                "rclpy.spin(node)\n"
            ),
        ],
        output="screen",
    )

    start_quest_teleop_after_joint_states = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_joint_states,
            on_exit=[quest_teleop_bimanual_node],
        )
    )

    delayed_wait = TimerAction(
        period=3.0,
        actions=[wait_for_joint_states]
    )

    return LaunchDescription(
        [
            use_fake_hardware_arg,
            left_can_interface_arg,
            right_can_interface_arg,
            rviz_node,
            ros2_control_node,
            container,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            left_gripper_controller_spawner,
            right_arm_controller_spawner,
            right_gripper_controller_spawner,
            delayed_wait,
            start_quest_teleop_after_joint_states,
        ]
    )
