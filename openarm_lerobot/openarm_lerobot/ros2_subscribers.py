"""
ROS 2 topic subscriber manager for OpenArm data collection.
Subscribes to joint_states, eef_pose, eef_delta, target_joint_positions,
gripper, and quest_x_button topics.
Caches latest values for 30Hz sampling.
"""

import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Float64MultiArray, Bool


class ROS2Subscribers(Node):
    """Subscribes to OpenArm ROS 2 topics and caches latest values."""

    # Expected joint order for bimanual OpenArm
    JOINT_ORDER = [
        "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
        "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
        "openarm_left_joint7",
        "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
        "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
        "openarm_right_joint7",
        "openarm_left_finger_joint1", "openarm_right_finger_joint1",
    ]

    def __init__(self, topic_config: dict):
        super().__init__("openarm_lerobot_collector")

        # Latest cached values
        self._joint_positions = np.zeros(16, dtype=np.float32)
        self._joint_velocities = np.zeros(16, dtype=np.float32)
        self._joint_efforts = np.zeros(16, dtype=np.float32)
        self._left_eef_pose = np.zeros(7, dtype=np.float32)
        self._right_eef_pose = np.zeros(7, dtype=np.float32)
        self._left_eef_delta = np.zeros(7, dtype=np.float32)
        self._right_eef_delta = np.zeros(7, dtype=np.float32)
        self._left_target_joints = np.zeros(7, dtype=np.float32)
        self._right_target_joints = np.zeros(7, dtype=np.float32)
        self._left_gripper = np.float32(0.0)
        self._right_gripper = np.float32(0.0)
        self._quest_x_button = False
        self._quest_rerecord = False

        # Joint name to index mapping
        self._joint_index = {name: i for i, name in enumerate(self.JOINT_ORDER)}

        # [방법 B] 이미지 토픽 캐시 — camera_tcp_streamer가 publish한 토픽을 여기서 구독.
        # add_image_topic()으로 토픽을 등록하면 자동으로 subscription이 생성됨.
        # [방법 A로 전환 시] 아래 _images/_image_locks 및 관련 메서드 전체 삭제.
        self._images: dict[str, np.ndarray | None] = {}
        self._image_locks: dict[str, threading.Lock] = {}

        # Subscribers
        self.create_subscription(
            JointState, topic_config["joint_states"],
            self._joint_states_cb, 10)

        self.create_subscription(
            PoseStamped, topic_config["left_eef_pose"],
            self._left_eef_pose_cb, 10)

        self.create_subscription(
            PoseStamped, topic_config["right_eef_pose"],
            self._right_eef_pose_cb, 10)

        self.create_subscription(
            PoseStamped, topic_config["left_eef_delta"],
            self._left_eef_delta_cb, 10)

        self.create_subscription(
            PoseStamped, topic_config["right_eef_delta"],
            self._right_eef_delta_cb, 10)

        self.create_subscription(
            Float64MultiArray, topic_config["left_target_joint_positions"],
            self._left_target_joints_cb, 10)

        self.create_subscription(
            Float64MultiArray, topic_config["right_target_joint_positions"],
            self._right_target_joints_cb, 10)

        self.create_subscription(
            Float64, topic_config["left_gripper"],
            self._left_gripper_cb, 10)

        self.create_subscription(
            Float64, topic_config["right_gripper"],
            self._right_gripper_cb, 10)

        self.create_subscription(
            Bool, topic_config["quest_x_button"],
            self._quest_x_button_cb, 10)

        self.create_subscription(
            Bool, topic_config["quest_rerecord"],
            self._quest_rerecord_cb, 10)

    def _joint_states_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self._joint_index:
                idx = self._joint_index[name]
                if i < len(msg.position):
                    self._joint_positions[idx] = msg.position[i]
                if i < len(msg.velocity):
                    self._joint_velocities[idx] = msg.velocity[i]
                if i < len(msg.effort):
                    self._joint_efforts[idx] = msg.effort[i]

    def _pose_to_array(self, msg: PoseStamped) -> np.ndarray:
        p = msg.pose.position
        o = msg.pose.orientation
        return np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w], dtype=np.float32)

    def _left_eef_pose_cb(self, msg: PoseStamped):
        self._left_eef_pose = self._pose_to_array(msg)

    def _right_eef_pose_cb(self, msg: PoseStamped):
        self._right_eef_pose = self._pose_to_array(msg)

    def _left_eef_delta_cb(self, msg: PoseStamped):
        self._left_eef_delta = self._pose_to_array(msg)

    def _right_eef_delta_cb(self, msg: PoseStamped):
        self._right_eef_delta = self._pose_to_array(msg)

    def _left_target_joints_cb(self, msg: Float64MultiArray):
        if len(msg.data) > 0:
            self._left_target_joints = np.array(msg.data, dtype=np.float32)[:7]

    def _right_target_joints_cb(self, msg: Float64MultiArray):
        if len(msg.data) > 0:
            self._right_target_joints = np.array(msg.data, dtype=np.float32)[:7]

    def _left_gripper_cb(self, msg: Float64):
        self._left_gripper = np.float32(msg.data)

    def _right_gripper_cb(self, msg: Float64):
        self._right_gripper = np.float32(msg.data)

    def _quest_x_button_cb(self, msg: Bool):
        self._quest_x_button = msg.data

    def _quest_rerecord_cb(self, msg: Bool):
        self._quest_rerecord = msg.data

    # [방법 B] 이미지 토픽 구독 — collect_data.py에서 호출
    # [방법 A로 전환 시] 아래 세 메서드 삭제
    def add_image_topic(self, topic: str):
        """ros2_topic 타입 카메라의 토픽을 구독 등록."""
        self._images[topic] = None
        self._image_locks[topic] = threading.Lock()
        self.create_subscription(
            Image, topic,
            lambda msg, t=topic: self._image_cb(t, msg),
            1
        )

    def _image_cb(self, topic: str, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        with self._image_locks[topic]:
            self._images[topic] = img.copy()

    def get_image(self, topic: str) -> np.ndarray | None:
        lock = self._image_locks.get(topic)
        if lock is None:
            return None
        with lock:
            img = self._images[topic]
            return img.copy() if img is not None else None

    def get_observation(self) -> dict:
        """Get latest observation data."""
        return {
            "state": self._joint_positions.copy(),
            "velocity": self._joint_velocities.copy(),
            "effort": self._joint_efforts.copy(),
            "left_eef_pose": self._left_eef_pose.copy(),
            "right_eef_pose": self._right_eef_pose.copy(),
        }

    def get_action(self) -> dict:
        """Get latest action data."""
        GRIPPER_MAX = 0.0264
        # trigger=0(안쥠) → open(0.0264), trigger=1(꽉쥠) → close(0.0)
        left_grip_cmd = np.float32(GRIPPER_MAX * (1.0 - self._left_gripper))
        right_grip_cmd = np.float32(GRIPPER_MAX * (1.0 - self._right_gripper))
        joint_positions = np.concatenate([
            self._left_target_joints, self._right_target_joints,
            [left_grip_cmd], [right_grip_cmd],
        ]).astype(np.float32)
        return {
            "": joint_positions,
            "joint_positions": joint_positions,
            "left_eef_delta": self._left_eef_delta.copy(),
            "right_eef_delta": self._right_eef_delta.copy(),
            "left_gripper": np.array([left_grip_cmd], dtype=np.float32),
            "right_gripper": np.array([right_grip_cmd], dtype=np.float32),
        }

    def get_quest_x_button(self) -> bool:
        """Get and reset Quest X button state (next episode)."""
        pressed = self._quest_x_button
        if pressed:
            self._quest_x_button = False
        return pressed

    def get_quest_rerecord(self) -> bool:
        """Get and reset Quest rerecord state."""
        pressed = self._quest_rerecord
        if pressed:
            self._quest_rerecord = False
        return pressed
