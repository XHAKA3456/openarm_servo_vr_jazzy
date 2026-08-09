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
from launch_ros.parameter_descriptions import ParameterValue
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
    use_dls_ik_arg = DeclareLaunchArgument(
        'use_dls_ik',
        default_value='true',
        description='true=DLS singularity-robust IK, false=TRAC-IK (original). For A/B comparison.'
    )

    # #11 중력보상 tau_ff 스케일 (0.0=off). 실기 검증 순서: 0.0(로그로 G vs 실측 비교)
    # -> 0.3 -> 0.6 -> 1.0 단계적으로.
    gravity_comp_scale_arg = DeclareLaunchArgument(
        'gravity_comp_scale',
        default_value='1.0',
        description='Gravity feedforward scale (1.0=verified default, 0.0=off)'
    )

    # #11-① 마찰보상 스케일 (no_rviz 버전과 동일)
    friction_comp_scale_arg = DeclareLaunchArgument(
        'friction_comp_scale',
        default_value='0.0',
        description='Friction feedforward scale (0.0=off, start at 0.3; overcomp => arm creeps)'
    )

    # #11 kp/kd 오버라이드 (no_rviz 버전과 동일)
    arm_kp_arg = DeclareLaunchArgument(
        'arm_kp', default_value='',
        description="7 comma-separated MIT kp gains (''=code defaults)")
    arm_kd_arg = DeclareLaunchArgument(
        'arm_kd', default_value='',
        description="7 comma-separated MIT kd gains, each <=5.0 (''=code defaults)")

    use_dls_ik = LaunchConfiguration('use_dls_ik')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    left_can_interface = LaunchConfiguration('left_can_interface')
    right_can_interface = LaunchConfiguration('right_can_interface')
    gravity_comp_scale = LaunchConfiguration('gravity_comp_scale')
    friction_comp_scale = LaunchConfiguration('friction_comp_scale')
    arm_kp = LaunchConfiguration('arm_kp')
    arm_kd = LaunchConfiguration('arm_kd')

    # #11 중력 모델용 평문 URDF 덤프 (no_rviz launch와 동일 방식)
    import xacro as _xacro
    gravity_urdf_path = "/tmp/openarm_bimanual_gravity_model.urdf"
    _gravity_doc = _xacro.process_file(
        os.path.join(
            get_package_share_directory("openarm_bimanual_moveit_config"),
            "config", "openarm_bimanual.urdf.xacro"),
        mappings={"ros2_control": "false", "bimanual": "true"},
    )
    with open(gravity_urdf_path, "w") as f:
        f.write(_gravity_doc.toxml())

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
                "gravity_urdf_path": gravity_urdf_path,
                "gravity_comp_scale": gravity_comp_scale,
                "friction_comp_scale": friction_comp_scale,
                "arm_kp": arm_kp,
                "arm_kd": arm_kd,
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
        output="log",
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
        output="log",
    )

    # Quest Teleop Bimanual Node
    quest_teleop_bimanual_node = Node(
        package="openarm_quest_teleop",
        executable="quest_teleop_bimanual",
        name="quest_teleop_bimanual",
        parameters=[
            servo_params_left,
            servo_params_right,
            # A/B toggle: override use_dls_ik from the launch arg (last-wins over the yaml value)
            {"moveit_servo_left.use_dls_ik": ParameterValue(use_dls_ik, value_type=bool),
             "moveit_servo_right.use_dls_ik": ParameterValue(use_dls_ik, value_type=bool)},
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

    # Neck Feetech Controller (Quest 헤드 트래킹 → Feetech 모터)
    stream_to_quest_arg = DeclareLaunchArgument(
        'stream_to_quest',
        default_value='false',
        description='Quest 헤드셋으로 카메라 영상 전송 여부 (false면 데이터 수집 전용)'
    )
    stream_to_quest = LaunchConfiguration('stream_to_quest')

    neck_serial_port_arg = DeclareLaunchArgument(
        'neck_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Feetech neck servos'
    )
    neck_serial_port = LaunchConfiguration('neck_serial_port')

    neck_controller_node = Node(
        package='openarm_quest_teleop',
        executable='neck_feetech_controller.py',
        name='neck_feetech_controller',
        parameters=[{
            'serial_port': neck_serial_port,
            'baudrate': 1000000,
            'yaw_motor_id': 7,
            'pitch_motor_id': 8,
            'yaw_center': 2048,
            'pitch_center': 2048,
            'deg_per_tick': 0.088,
            'yaw_max_deg': 110.0,
            'pitch_max_deg': 90.0,
            'servo_speed': 0,
            'servo_acc': 50,
            'calibration_duration': 2.0,
        }],
        output='screen',
    )

    # Camera TCP Streamer (Quest 헤드셋으로 영상 전송)
    # [방법 B] camera_tcp_streamer가 RealSense 하드웨어를 단독 소유.
    # collect_data.py는 /camera/head/color/raw, /camera/head/depth/colormap 토픽을 구독.
    #
    # [방법 A로 전환 시 — 육안 조종 + 카메라 스트리머 없이 데이터 수집]
    #   1. 아래 camera_streamer_node를 LaunchDescription에서 제거 (or 이 파일 자체를 쓰지 않음)
    #   2. collect_data.yaml head 카메라: type: "ros2_topic" → type: "intelrealsense"
    #   3. collect_data.py: image_getters 관련 코드 제거, cameras 먼저 초기화
    #   4. cameras.py: ros2_topic 분기 삭제
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
            'depth_min_m': 0.3,
            'depth_max_m': 1.3,
            'stream_to_quest': stream_to_quest,
        }],
        output='screen',
    )

    homing_node = Node(
        package='openarm_quest_teleop',
        executable='homing_node.py',
        name='homing_node',
        output='screen',
    )

    return LaunchDescription(
        [
            use_fake_hardware_arg,
            left_can_interface_arg,
            right_can_interface_arg,
            use_dls_ik_arg,
            gravity_comp_scale_arg,
            friction_comp_scale_arg,
            arm_kp_arg,
            arm_kd_arg,
            stream_to_quest_arg,
            neck_serial_port_arg,
            # rviz_node,
            ros2_control_node,
            container,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            left_gripper_controller_spawner,
            right_arm_controller_spawner,
            right_gripper_controller_spawner,
            homing_node,
            delayed_wait,
            start_quest_teleop_after_joint_states,
            neck_controller_node,
            camera_streamer_node,
        ]
    )
