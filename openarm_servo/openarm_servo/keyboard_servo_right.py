#!/usr/bin/env python3
"""
Keyboard input for MoveIt Servo - OpenArm Right Arm version
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from control_msgs.action import GripperCommand
import sys
import termios
import tty


class KeyboardServo(Node):
    def __init__(self):
        super().__init__('keyboard_servo_right')

        # Publishers
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        self.joint_pub = self.create_publisher(
            JointJog,
            '/servo_node/delta_joint_cmds',
            10
        )

        # Gripper action client
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/right_gripper_controller/gripper_cmd'
        )

        # Subscribe to joint states to track gripper position
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Always use hand frame
        self.frame_to_publish = 'openarm_right_hand'
        self.joint_vel_cmd = 1.0

        # Gripper control
        self.gripper_max_position = 0.044  # Fully open (44mm)
        self.gripper_min_position = 0.0    # Fully closed
        self.gripper_current_position = 0.022  # Start at middle
        self.gripper_step = 0.002  # 2mm per keypress

        self.get_logger().info("Keyboard Servo Control for OpenArm RIGHT ARM")
        self.get_logger().info("----------------------------------")
        self.get_logger().info("TRANSLATION (Linear):")
        self.get_logger().info("  Arrow UP/DOWN: X axis (red)")
        self.get_logger().info("  Arrow LEFT/RIGHT: Y axis (green)")
        self.get_logger().info("  ./;: Z axis (blue)")
        self.get_logger().info("ROTATION (Angular):")
        self.get_logger().info("  I/K: Roll around X axis (red)")
        self.get_logger().info("  U/O: Pitch around Y axis (green)")
        self.get_logger().info("  J/L: Yaw around Z axis (blue)")
        self.get_logger().info("----------------------------------")
        self.get_logger().info("Frame: openarm_right_hand (fixed)")
        self.get_logger().info("1-7: Joint jog")
        self.get_logger().info("R: Reverse joint jog direction")
        self.get_logger().info("----------------------------------")
        self.get_logger().info("GRIPPER CONTROL:")
        self.get_logger().info("  G: Open gripper (+2mm per press)")
        self.get_logger().info("  H: Close gripper (-2mm per press)")
        self.get_logger().info("----------------------------------")
        self.get_logger().info("Q: Quit")

    def joint_state_callback(self, msg):
        """Update current gripper position from joint states"""
        try:
            idx = msg.name.index('openarm_right_finger_joint1')
            self.gripper_current_position = msg.position[idx]
        except (ValueError, IndexError):
            pass  # Joint not found in this message

    def get_key(self):
        """Read a single keypress from stdin"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            if ch == '\x1b':  # Arrow keys start with escape
                ch += sys.stdin.read(2)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def send_gripper_command(self, delta):
        """Send gripper command via action client with incremental position"""
        # Calculate new position
        new_position = self.gripper_current_position + delta

        # Clamp to valid range
        new_position = max(self.gripper_min_position, min(self.gripper_max_position, new_position))

        # Send command
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = new_position
        goal_msg.command.max_effort = 100.0  # Max effort

        self.get_logger().info(f"[Gripper CMD] {self.gripper_current_position:.4f}m -> {new_position:.4f}m (delta={delta:+.4f}m)")

        # Update internal position (will be updated by joint_state_callback)
        self.gripper_current_position = new_position

        # Send goal (non-blocking)
        self.gripper_action_client.send_goal_async(goal_msg)

    def run(self):
        """Main loop to read keyboard and publish commands"""
        while rclpy.ok():
            # Spin once to process callbacks (for action client)
            rclpy.spin_once(self, timeout_sec=0.0)

            key = self.get_key()

            twist_msg = TwistStamped()
            joint_msg = JointJog()
            publish_twist = False
            publish_joint = False

            # Process key
            if key == '\x1b[A':  # UP arrow
                twist_msg.twist.linear.x = 1.0
                publish_twist = True
            elif key == '\x1b[B':  # DOWN arrow
                twist_msg.twist.linear.x = -1.0
                publish_twist = True
            elif key == '\x1b[C':  # RIGHT arrow
                twist_msg.twist.linear.y = 1.0
                publish_twist = True
            elif key == '\x1b[D':  # LEFT arrow
                twist_msg.twist.linear.y = -1.0
                publish_twist = True
            elif key == ';':
                twist_msg.twist.linear.z = 1.0
                publish_twist = True
            elif key == '.':
                twist_msg.twist.linear.z = -1.0
                publish_twist = True
            # Rotation commands
            elif key == 'i' or key == 'I':
                twist_msg.twist.angular.x = 1.0
                publish_twist = True
            elif key == 'k' or key == 'K':
                twist_msg.twist.angular.x = -1.0
                publish_twist = True
            elif key == 'u' or key == 'U':
                twist_msg.twist.angular.y = 1.0
                publish_twist = True
            elif key == 'o' or key == 'O':
                twist_msg.twist.angular.y = -1.0
                publish_twist = True
            elif key == 'j' or key == 'J':
                twist_msg.twist.angular.z = 1.0
                publish_twist = True
            elif key == 'l' or key == 'L':
                twist_msg.twist.angular.z = -1.0
                publish_twist = True
            elif key == 'r' or key == 'R':
                self.joint_vel_cmd *= -1
                self.get_logger().info(f"→ Joint velocity direction reversed: {self.joint_vel_cmd}")
            elif key in ['1', '2', '3', '4', '5', '6', '7']:
                joint_num = int(key)
                joint_msg.joint_names = [f'openarm_right_joint{joint_num}']
                joint_msg.velocities = [self.joint_vel_cmd]
                publish_joint = True
            elif key == 'g' or key == 'G':
                # Open gripper (increase position)
                self.send_gripper_command(self.gripper_step)
            elif key == 'h' or key == 'H':
                # Close gripper (decrease position)
                self.send_gripper_command(-self.gripper_step)
            elif key == 'q' or key == 'Q':
                self.get_logger().info("Exiting keyboard control...")
                break

            # Publish Twist
            if publish_twist:
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.header.frame_id = self.frame_to_publish
                self.twist_pub.publish(twist_msg)
                self.get_logger().info(
                    f"[Twist CMD] key={repr(key)} | frame={self.frame_to_publish} | "
                    f"linear=({twist_msg.twist.linear.x:.3f}, {twist_msg.twist.linear.y:.3f}, {twist_msg.twist.linear.z:.3f}) | "
                    f"angular=({twist_msg.twist.angular.x:.3f}, {twist_msg.twist.angular.y:.3f}, {twist_msg.twist.angular.z:.3f})"
                )

            # Publish Joint
            elif publish_joint:
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.header.frame_id = 'openarm_body_link0'
                self.joint_pub.publish(joint_msg)
                self.get_logger().info(
                    f"[Joint CMD] key={repr(key)} | joint={joint_msg.joint_names[0]} | vel={joint_msg.velocities[0]:.3f}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServo()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
