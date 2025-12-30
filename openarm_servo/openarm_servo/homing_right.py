#!/usr/bin/env python3
"""
Simple homing control for OpenArm Right Arm
Moves joint1 and joint4 to home position sequentially
"""

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
import time


class HomingRight(Node):
    def __init__(self):
        super().__init__('homing_right')

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointJog,
            '/servo_node/delta_joint_cmds',
            10
        )

        # Subscriber for joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Current joint positions
        self.current_positions = {}

        # Home positions
        self.home_joint1 = -0.306763475322849
        self.home_joint4 = 1.8726237741283485

        # Homing state
        self.stage = 0  # 0: waiting, 1: moving joint1, 2: moving joint4, 3: complete
        self.joint_vel = 3.0
        self.threshold = 0.01

        self.get_logger().info("=== Homing Control (Right Arm) ===")
        self.get_logger().info("Starting in 3 seconds...")

    def joint_state_callback(self, msg):
        """Store current joint positions"""
        for name, position in zip(msg.name, msg.position):
            self.current_positions[name] = position

    def run(self):
        """Main loop"""
        # Wait 3 seconds while processing callbacks
        start_time = time.time()
        while time.time() - start_time < 3.0:
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        # Check joint states
        if 'openarm_right_joint1' not in self.current_positions:
            self.get_logger().error("Joint states not received!")
            return

        # Start homing
        j1_current = self.current_positions['openarm_right_joint1']
        j4_current = self.current_positions.get('openarm_right_joint4', 0.0)

        self.get_logger().info("[HOMING] Starting...")
        self.get_logger().info(f"  joint1: {j1_current:.3f} -> {self.home_joint1:.3f}")
        self.get_logger().info(f"  joint4: {j4_current:.3f} -> {self.home_joint4:.3f}")

        self.stage = 1
        msg_count = 0

        # Main loop
        while rclpy.ok() and self.stage < 3:
            rclpy.spin_once(self, timeout_sec=0.0)

            # Stage 1: Move joint1
            if self.stage == 1:
                if 'openarm_right_joint1' not in self.current_positions:
                    time.sleep(0.01)
                    continue

                current = self.current_positions['openarm_right_joint1']
                delta = self.home_joint1 - current

                if abs(delta) < self.threshold:
                    self.get_logger().info(f"[HOMING] joint1 reached! ({current:.3f})")
                    self.stage = 2
                    msg_count = 0
                    time.sleep(0.01)
                    continue

                vel = self.joint_vel if delta > 0 else -self.joint_vel
                joint_msg = JointJog()
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.header.frame_id = 'openarm_body_link0'
                joint_msg.joint_names = ['openarm_right_joint1']
                joint_msg.velocities = [vel]
                self.joint_pub.publish(joint_msg)

                msg_count += 1
                if msg_count % 100 == 0:
                    self.get_logger().info(f"[Stage1] pos={current:.3f}, vel={vel:.1f}")

            # Stage 2: Move joint4
            elif self.stage == 2:
                if 'openarm_right_joint4' not in self.current_positions:
                    time.sleep(0.01)
                    continue

                current = self.current_positions['openarm_right_joint4']
                delta = self.home_joint4 - current

                if abs(delta) < self.threshold:
                    self.get_logger().info(f"[HOMING] joint4 reached! ({current:.3f})")
                    self.get_logger().info("[HOMING] Complete!")
                    self.stage = 3
                    break

                vel = self.joint_vel if delta > 0 else -self.joint_vel
                joint_msg = JointJog()
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.header.frame_id = 'openarm_body_link0'
                joint_msg.joint_names = ['openarm_right_joint4']
                joint_msg.velocities = [vel]
                self.joint_pub.publish(joint_msg)

                msg_count += 1
                if msg_count % 100 == 0:
                    self.get_logger().info(f"[Stage2] pos={current:.3f}, vel={vel:.1f}")

            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = HomingRight()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
