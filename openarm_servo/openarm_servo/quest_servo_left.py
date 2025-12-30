#!/usr/bin/env python3
"""
Quest VR Teleoperation for OpenArm Left Arm
1. Auto homing (joint1 -> joint4)
2. Calibration (5 sec average)
3. Quest controller teleoperation with coordinate transformation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.action import GripperCommand
import time
import socket
import json
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R


class QuestServoLeft(Node):
    def __init__(self, host='0.0.0.0', port=5454):
        super().__init__('quest_servo_left')

        # Publishers
        self.joint_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # Subscriber
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Gripper action client
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/left_gripper_controller/gripper_cmd'
        )

        # Joint positions
        self.current_positions = {}

        # Home positions
        self.home_joint4 = 1.58  # Only homing joint4 to 1.58

        # State machine
        # 0: waiting, 1: homing j4, 2: calibration, 3: teleoperation
        self.stage = 0
        self.joint_vel = 2.0
        self.threshold = 0.01

        # Quest data (socket thread updates this)
        self.latest_quest_data = None
        self.data_lock = threading.Lock()

        # Calibration
        self.calibration_samples = []
        self.calibration_duration = 5.0
        self.calibration_start_time = None
        self.reference_position = None  # Calibrated reference
        self.reference_rotation = None

        # Teleoperation (velocity-based)
        self.prev_position = None
        self.prev_rotation = None
        self.prev_timestamp = None
        self.linear_scale = 5.0
        self.angular_scale = 5.0

        # Gripper control
        self.gripper_min_position = 0.0    # Fully closed
        self.gripper_max_position = 0.044  # Fully open (44mm)
        self.prev_trigger = None  # Previous trigger value to detect changes

        # Socket server
        self.host = host
        self.port = port
        self.running = True
        self.socket_thread = threading.Thread(target=self.socket_server, daemon=True)
        self.socket_thread.start()

        self.get_logger().info("=== Quest Teleoperation (LEFT) ===")
        self.get_logger().info("Stage 0: Waiting 3s")
        self.get_logger().info("Stage 1: Homing J4 to 1.58")
        self.get_logger().info("Stage 2: Calibration (5s)")
        self.get_logger().info("Stage 3: Teleoperation")

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_positions[name] = position

    def socket_server(self):
        """Socket server thread - receives Quest data"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            server.bind((self.host, self.port))
            server.listen(1)
            self.get_logger().info(f"[SOCKET] Listening on {self.host}:{self.port}")

            conn, addr = server.accept()
            self.get_logger().info(f"[SOCKET] Connected: {addr}")

            buffer = ""
            while self.running and rclpy.ok():
                data = conn.recv(1024)
                if not data:
                    break

                buffer += data.decode()
                lines = buffer.split('\n')

                for line in lines[:-1]:
                    if line.strip():
                        try:
                            quest_data = json.loads(line)
                            with self.data_lock:
                                self.latest_quest_data = quest_data
                        except json.JSONDecodeError:
                            pass

                buffer = lines[-1]

        except Exception as e:
            self.get_logger().error(f"[SOCKET] Error: {e}")
        finally:
            server.close()

    def controller_to_openarm_position(self, ctrl_pos):
        """
        Transform controller position to OpenArm frame
        Controller: 앞=+z, 위=+y, 오른쪽=+x
        OpenArm:    앞=+z, 위=+x, 오른쪽=+y
        """
        return np.array([
            ctrl_pos[1],  # OpenArm.x = Controller.y (위)
            ctrl_pos[0],  # OpenArm.y = Controller.x (오른쪽)
            ctrl_pos[2],  # OpenArm.z = Controller.z (앞)
        ])

    def controller_to_openarm_rotation(self, ctrl_quat):
        """
        Transform controller rotation to OpenArm frame
        Swap x and y axes
        """
        # Convert to rotation matrix
        ctrl_rot = R.from_quat(ctrl_quat)
        mat = ctrl_rot.as_matrix()

        # Swap columns: [x, y, z] -> [y, x, z]
        mat_swapped = mat[:, [1, 0, 2]]

        # Swap rows: [x, y, z] -> [y, x, z]
        mat_swapped = mat_swapped[[1, 0, 2], :]

        # Convert back to quaternion
        return R.from_matrix(mat_swapped).as_quat()

    def send_gripper_command(self, trigger_value):
        """
        Send gripper command based on trigger value (0~1)
        INVERTED: 0 (release) = open, 1 (press) = close
        """
        # INVERTED: trigger 0 (release) = open, trigger 1 (press) = close
        inverted_trigger = 1.0 - trigger_value

        # Map inverted trigger (0~1) to gripper position (0.0~0.044)
        gripper_position = self.gripper_min_position + \
                          (inverted_trigger * (self.gripper_max_position - self.gripper_min_position))

        # Create and send goal
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = gripper_position
        goal_msg.command.max_effort = 100.0

        # Send goal (non-blocking)
        self.gripper_action_client.send_goal_async(goal_msg)

    def run(self):
        """Main loop"""
        # Stage 0: Wait 3 seconds
        start_time = time.time()
        while time.time() - start_time < 3.0:
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        # Check joint states received
        if 'openarm_left_joint4' not in self.current_positions:
            self.get_logger().error("No joint states!")
            return

        # Start homing
        self.get_logger().info(f"[HOMING] j4: {self.current_positions.get('openarm_left_joint4', 0):.3f} -> {self.home_joint4:.3f}")
        self.stage = 1

        msg_count = 0

        # Main loop
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            # Stage 1: Homing joint4
            if self.stage == 1:
                current = self.current_positions.get('openarm_left_joint4', 0)
                delta = self.home_joint4 - current

                if abs(delta) < self.threshold:
                    self.get_logger().info(f"[HOMING] j4 done ({current:.3f})")

                    # Open gripper after homing
                    self.get_logger().info("[GRIPPER] Opening gripper...")
                    self.send_gripper_command(0.0)  # 0.0 = fully open (inverted)

                    self.get_logger().info("[CALIBRATION] Waiting for Quest data...")
                    self.stage = 2
                    self.calibration_start_time = None  # Will start when Quest data received
                    self.calibration_samples = []
                    msg_count = 0
                else:
                    vel = self.joint_vel if delta > 0 else -self.joint_vel
                    msg = JointJog()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'openarm_body_link0'
                    msg.joint_names = ['openarm_left_joint4']
                    msg.velocities = [vel]
                    self.joint_pub.publish(msg)

            # Stage 2: Calibration (collect samples for 5 seconds)
            elif self.stage == 2:
                # Log waiting status every 1 second
                if self.calibration_start_time is None:
                    msg_count += 1
                    if msg_count % 100 == 0:
                        self.get_logger().info("[CALIBRATION] Waiting for Quest connection...")

                with self.data_lock:
                    if self.latest_quest_data:
                        left = self.latest_quest_data.get('left', {})
                        if left.get('enabled', False):
                            # First data received - start calibration timer
                            if self.calibration_start_time is None:
                                self.calibration_start_time = time.time()
                                msg_count = 0
                                self.get_logger().info("[CALIBRATION] Quest connected! Starting 5s calibration...")

                            pos = left.get('position', {})
                            rot = left.get('rotation', {})

                            ctrl_pos = np.array([pos['x'], pos['y'], pos['z']])
                            ctrl_quat = np.array([rot['x'], rot['y'], rot['z'], rot['w']])

                            # Transform to OpenArm frame
                            openarm_pos = self.controller_to_openarm_position(ctrl_pos)
                            openarm_quat = self.controller_to_openarm_rotation(ctrl_quat)

                            self.calibration_samples.append((openarm_pos, openarm_quat))

                # Check if calibration time elapsed
                if self.calibration_start_time is not None:
                    elapsed = time.time() - self.calibration_start_time
                else:
                    elapsed = 0

                if elapsed >= self.calibration_duration and self.calibration_start_time is not None:
                    if len(self.calibration_samples) > 0:
                        # Calculate average
                        positions = np.array([s[0] for s in self.calibration_samples])
                        quaternions = np.array([s[1] for s in self.calibration_samples])

                        self.reference_position = np.mean(positions, axis=0)

                        # Average quaternions (simple mean, not geodesic)
                        self.reference_rotation = np.mean(quaternions, axis=0)
                        self.reference_rotation /= np.linalg.norm(self.reference_rotation)

                        self.get_logger().info(f"[CALIBRATION] Done! Samples: {len(self.calibration_samples)}")
                        self.get_logger().info(f"[CALIBRATION] Ref pos: {self.reference_position}")
                        self.get_logger().info("[TELEOPERATION] Ready! Move controller.")
                        self.stage = 3
                    else:
                        self.get_logger().error("[CALIBRATION] No data received!")
                        return

            # Stage 3: Teleoperation
            elif self.stage == 3:
                self.publish_quest_twist()
                self.publish_gripper_command()

            time.sleep(0.01)

    def publish_quest_twist(self):
        """Calculate and publish twist from Quest controller (velocity-based)"""
        with self.data_lock:
            if not self.latest_quest_data:
                return
            quest_data = self.latest_quest_data.copy()

        left = quest_data.get('left', {})
        if not left.get('enabled', False):
            return

        timestamp = quest_data.get('timestamp', 0.0)
        pos = left.get('position', {})
        rot = left.get('rotation', {})

        ctrl_pos = np.array([pos['x'], pos['y'], pos['z']])
        ctrl_quat = np.array([rot['x'], rot['y'], rot['z'], rot['w']])

        # Transform to OpenArm frame
        current_pos = self.controller_to_openarm_position(ctrl_pos)
        current_quat = self.controller_to_openarm_rotation(ctrl_quat)

        # Initialize on first data
        if self.prev_position is None:
            self.prev_position = current_pos
            self.prev_rotation = current_quat
            self.prev_timestamp = timestamp
            self.get_logger().info("[TELEOPERATION] Initialized!")
            return

        # Calculate time delta
        dt = timestamp - self.prev_timestamp
        if dt <= 0 or dt > 1.0:  # Skip invalid dt
            self.prev_timestamp = timestamp
            return

        # Calculate velocity (position change over time)
        linear_vel = (current_pos - self.prev_position) / dt

        # Calculate angular velocity
        prev_rot = R.from_quat(self.prev_rotation)
        curr_rot = R.from_quat(current_quat)
        delta_rot = curr_rot * prev_rot.inv()
        angular_vel = delta_rot.as_rotvec() / dt

        # Create twist message
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'openarm_left_hand'

        twist.twist.linear.x = float(linear_vel[0] * self.linear_scale)
        twist.twist.linear.y = float(linear_vel[1] * self.linear_scale)
        twist.twist.linear.z = float(linear_vel[2] * self.linear_scale)

        twist.twist.angular.x = float(angular_vel[0] * self.angular_scale)
        twist.twist.angular.y = float(angular_vel[1] * self.angular_scale)
        twist.twist.angular.z = float(angular_vel[2] * self.angular_scale)

        self.twist_pub.publish(twist)

        # Update previous values
        self.prev_position = current_pos
        self.prev_rotation = current_quat
        self.prev_timestamp = timestamp

    def publish_gripper_command(self):
        """Read trigger value from Quest and send gripper command"""
        with self.data_lock:
            if not self.latest_quest_data:
                return
            quest_data = self.latest_quest_data.copy()

        left = quest_data.get('left', {})
        if not left.get('enabled', False):
            return

        trigger = left.get('trigger', 0.0)

        # Send command only when trigger changes significantly
        if self.prev_trigger is None or abs(trigger - self.prev_trigger) > 0.01:
            self.send_gripper_command(trigger)
            self.prev_trigger = trigger

    def destroy_node(self):
        self.running = False
        if self.socket_thread:
            self.socket_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QuestServoLeft()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
