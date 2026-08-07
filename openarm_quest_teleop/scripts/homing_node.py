#!/usr/bin/env python3
"""
Homing node: JTC가 활성화된 후 양팔과 그리퍼를 홈 포지션으로 부드럽게 이동시킴.
- 팔: 현재 위치 → 홈 (j4=1.58, others=0)
- 그리퍼: 현재 위치 → 열린 상태 (0.0264m) — 닫았다 피는 게 아니라 현재 위치에서 바로 열림
JTC on_activate()가 actual state로 초기화한 뒤 호출되므로 startup jerk 없음.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers

LEFT_JOINTS = [
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7",
]
RIGHT_JOINTS = [
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7",
]
HOME_POSITIONS = [0.0, 0.0, 0.0, 1.58, 0.0, 0.0, 0.0]
HOME_DURATION_SEC = 4
GRIPPER_OPEN = 0.0264  # 완전히 열린 위치 (m)


class HomingNode(Node):
    def __init__(self):
        super().__init__("homing_node")
        self._left_pub = self.create_publisher(
            JointTrajectory,
            "/left_joint_trajectory_controller/joint_trajectory",
            10,
        )
        self._right_pub = self.create_publisher(
            JointTrajectory,
            "/right_joint_trajectory_controller/joint_trajectory",
            10,
        )
        self._left_gripper = ActionClient(
            self, GripperCommand, "/left_gripper_controller/gripper_cmd"
        )
        self._right_gripper = ActionClient(
            self, GripperCommand, "/right_gripper_controller/gripper_cmd"
        )

    def wait_for_jtc(self, timeout_sec: float = 30.0) -> bool:
        """양쪽 JTC가 모두 active 상태일 때까지 대기."""
        self.get_logger().info("Waiting for JTC controllers...")
        cli = self.create_client(ListControllers, "/controller_manager/list_controllers")
        cli.wait_for_service(timeout_sec=10.0)

        REQUIRED = {"left_joint_trajectory_controller", "right_joint_trajectory_controller"}
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            future = cli.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.done():
                active = {c.name for c in future.result().controller if c.state == "active"}
                if REQUIRED.issubset(active):
                    self.get_logger().info("JTC controllers ready.")
                    return True
            time.sleep(0.2)

        self.get_logger().warning("Timeout waiting for JTC. Sending homing anyway.")
        return False

    def send_home(self):
        """팔 홈 트레젝토리 + 그리퍼 현재 위치에서 열기."""
        point = JointTrajectoryPoint()
        point.positions = HOME_POSITIONS
        point.velocities = [0.0] * 7
        point.time_from_start = Duration(sec=HOME_DURATION_SEC, nanosec=0)

        now = self.get_clock().now().to_msg()

        left_traj = JointTrajectory()
        left_traj.header.stamp = now
        left_traj.joint_names = LEFT_JOINTS
        left_traj.points = [point]

        right_traj = JointTrajectory()
        right_traj.header.stamp = now
        right_traj.joint_names = RIGHT_JOINTS
        right_traj.points = [point]

        # Publish several times over ~0.6s. A single publish right after the JTC subscription is
        # discovered can be dropped before the pub/sub (DDS) connection is fully established
        # (ROS2 first-message race) -> that arm silently doesn't home. Repeating makes it reliable
        # for BOTH arms regardless of which connects a touch later.
        for _ in range(6):
            stamp = self.get_clock().now().to_msg()
            left_traj.header.stamp = stamp
            right_traj.header.stamp = stamp
            self._left_pub.publish(left_traj)
            self._right_pub.publish(right_traj)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        self.get_logger().info(
            f"Arm homing sent x6 (duration={HOME_DURATION_SEC}s, j4=1.58, others=0)"
        )

        # 그리퍼: 현재 위치에서 열린 위치로 이동
        self._send_gripper(self._left_gripper, "left")
        self._send_gripper(self._right_gripper, "right")

    def _send_gripper(self, client: ActionClient, name: str):
        if client.wait_for_server(timeout_sec=3.0):
            goal = GripperCommand.Goal()
            goal.command.position = GRIPPER_OPEN  # 현재 위치 → open (controller가 smooth하게 이동)
            goal.command.max_effort = 10.0
            client.send_goal_async(goal)
            self.get_logger().info(f"{name} gripper -> open ({GRIPPER_OPEN}m)")
        else:
            self.get_logger().warning(f"{name} gripper action server not ready, skipping.")


def main():
    rclpy.init()
    node = HomingNode()
    node.wait_for_jtc()
    node.send_home()
    time.sleep(HOME_DURATION_SEC + 0.5)
    node.get_logger().info("Homing complete.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
