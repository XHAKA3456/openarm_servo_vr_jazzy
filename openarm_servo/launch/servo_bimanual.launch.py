import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration
import xacro
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Declare launch arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware (simulation) or real hardware'
    )

    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    moveit_config = (
        MoveItConfigsBuilder("openarm_bimanual")
        .robot_description(
            file_path="config/openarm_bimanual.urdf.xacro",
            mappings={
                "ros2_control": "true",
                "use_fake_hardware": use_fake_hardware,
                "bimanual": "true",
            }
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # Get parameters for the Servo nodes
    servo_yaml_left = load_yaml("openarm_servo", "config/openarm_left_simulated_config.yaml")
    servo_params_left = {"moveit_servo": servo_yaml_left}

    servo_yaml_right = load_yaml("openarm_servo", "config/openarm_right_simulated_config.yaml")
    servo_params_right = {"moveit_servo": servo_yaml_right}

    # Jazzy requires these parameters to be passed separately
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name_left = {"planning_group_name": "left_arm"}
    planning_group_name_right = {"planning_group_name": "right_arm"}

    # RViz
    rviz_config_file = (
        get_package_share_directory("openarm_servo") + "/config/openarm_servo_bimanual.rviz"
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

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("openarm_servo"),
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

    # Launch as much as possible in components
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
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    # Launch TWO standalone Servo nodes with namespaces
    # Left arm servo node
    servo_node_left = Node(
        package="moveit_servo",
        executable="servo_node",
        namespace="left",
        parameters=[
            servo_params_left,
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

    # Right arm servo node
    servo_node_right = Node(
        package="moveit_servo",
        executable="servo_node",
        namespace="right",
        parameters=[
            servo_params_right,
            acceleration_filter_update_period,
            planning_group_name_right,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
        output="screen",
    )

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

    start_servo_after_joint_states = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_joint_states,
            on_exit=[
                servo_node_left,
                servo_node_right,
            ],
        )
    )

    # Delay wait_for_joint_states to ensure broadcasters are ready
    delayed_wait = TimerAction(
        period=3.0,
        actions=[wait_for_joint_states]
    )

    return LaunchDescription(
        [
            use_fake_hardware_arg,
            rviz_node,
            ros2_control_node,
            container,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            left_gripper_controller_spawner,
            right_arm_controller_spawner,
            right_gripper_controller_spawner,
            delayed_wait,
            start_servo_after_joint_states,
        ]
    )
