"""
ROS 2 topic subscriber manager for OpenArm data collection.
Subscribes to joint_states, twist, gripper, and quest_x_button topics.
Caches latest values for 30Hz sampling.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64, Bool


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
        self._left_twist = np.zeros(6, dtype=np.float32)
        self._right_twist = np.zeros(6, dtype=np.float32)
        self._left_gripper = np.float32(0.0)
        self._right_gripper = np.float32(0.0)
        self._quest_x_button = False

        # Joint name to index mapping
        self._joint_index = {name: i for i, name in enumerate(self.JOINT_ORDER)}

        # Subscribers
        self.create_subscription(
            JointState, topic_config["joint_states"],
            self._joint_states_cb, 10)

        self.create_subscription(
            TwistStamped, topic_config["left_twist"],
            self._left_twist_cb, 10)

        self.create_subscription(
            TwistStamped, topic_config["right_twist"],
            self._right_twist_cb, 10)

        self.create_subscription(
            Float64, topic_config["left_gripper"],
            self._left_gripper_cb, 10)

        self.create_subscription(
            Float64, topic_config["right_gripper"],
            self._right_gripper_cb, 10)

        self.create_subscription(
            Bool, topic_config["quest_x_button"],
            self._quest_x_button_cb, 10)

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

    def _left_twist_cb(self, msg: TwistStamped):
        t = msg.twist
        self._left_twist = np.array([
            t.linear.x, t.linear.y, t.linear.z,
            t.angular.x, t.angular.y, t.angular.z,
        ], dtype=np.float32)

    def _right_twist_cb(self, msg: TwistStamped):
        t = msg.twist
        self._right_twist = np.array([
            t.linear.x, t.linear.y, t.linear.z,
            t.angular.x, t.angular.y, t.angular.z,
        ], dtype=np.float32)

    def _left_gripper_cb(self, msg: Float64):
        self._left_gripper = np.float32(msg.data)

    def _right_gripper_cb(self, msg: Float64):
        self._right_gripper = np.float32(msg.data)

    def _quest_x_button_cb(self, msg: Bool):
        self._quest_x_button = msg.data

    def get_observation(self) -> dict:
        """Get latest observation data."""
        return {
            "state": self._joint_positions.copy(),
            "velocity": self._joint_velocities.copy(),
            "effort": self._joint_efforts.copy(),
        }

    def get_action(self) -> dict:
        """Get latest action data."""
        return {
            "left_twist": self._left_twist.copy(),
            "right_twist": self._right_twist.copy(),
            "left_gripper": np.array([self._left_gripper], dtype=np.float32),
            "right_gripper": np.array([self._right_gripper], dtype=np.float32),
        }

    def get_quest_x_button(self) -> bool:
        """Get and reset Quest X button state."""
        pressed = self._quest_x_button
        if pressed:
            self._quest_x_button = False
        return pressed
