#!/usr/bin/env python3
"""
Servo Message Monitor - 주는 쪽과 받는 쪽 메시지를 동시에 모니터링
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import TransformListener, Buffer
import math


class ServoMonitor(Node):
    def __init__(self):
        super().__init__('servo_monitor')

        # TF2 설정 (EE pose 추적용)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.twist_sub = self.create_subscription(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            self.twist_callback,
            10
        )

        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/left_joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        # Timer for periodic EE pose printing
        self.create_timer(0.5, self.print_ee_pose)

        self.last_twist = None
        self.last_traj = None

        self.get_logger().info("=" * 80)
        self.get_logger().info("Servo Monitor Started")
        self.get_logger().info("=" * 80)

    def twist_callback(self, msg: TwistStamped):
        """주는 쪽: Twist 명령 수신"""
        self.last_twist = msg

        lin = msg.twist.linear
        ang = msg.twist.angular

        self.get_logger().info(
            f"\n[📤 TWIST CMD] frame={msg.header.frame_id}\n"
            f"  Linear:  X={lin.x:+.3f} Y={lin.y:+.3f} Z={lin.z:+.3f}\n"
            f"  Angular: X={ang.x:+.3f} Y={ang.y:+.3f} Z={ang.z:+.3f}"
        )

    def trajectory_callback(self, msg: JointTrajectory):
        """받는 쪽: Servo가 생성한 Joint Trajectory"""
        self.last_traj = msg

        if len(msg.points) > 0:
            point = msg.points[0]
            positions = point.positions

            joint_str = "\n".join([
                f"    {name}: {pos:.4f} rad ({math.degrees(pos):.2f}°)"
                for name, pos in zip(msg.joint_names, positions)
            ])

            self.get_logger().info(
                f"\n[📥 JOINT TRAJ] {len(msg.joint_names)} joints\n{joint_str}"
            )

    def print_ee_pose(self):
        """EE의 실제 위치 출력 (TF 기반)"""
        try:
            # openarm_body_link0 → openarm_left_hand 변환 조회
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'openarm_body_link0',
                'openarm_left_hand',
                rclpy.time.Time()
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            # Quaternion to Euler (간단한 Roll-Pitch-Yaw)
            # 정확한 변환은 tf_transformations 사용하지만, 여기선 간단히 표시
            roll = math.atan2(2*(r.w*r.x + r.y*r.z), 1-2*(r.x**2 + r.y**2))
            pitch = math.asin(2*(r.w*r.y - r.z*r.x))
            yaw = math.atan2(2*(r.w*r.z + r.x*r.y), 1-2*(r.y**2 + r.z**2))

            self.get_logger().info(
                f"\n[📍 EE POSE] openarm_left_hand\n"
                f"  Position: X={t.x:+.4f} Y={t.y:+.4f} Z={t.z:+.4f} (m)\n"
                f"  Rotation: R={math.degrees(roll):+.2f}° "
                f"P={math.degrees(pitch):+.2f}° Y={math.degrees(yaw):+.2f}°\n"
                f"  Quaternion: x={r.x:.3f} y={r.y:.3f} z={r.z:.3f} w={r.w:.3f}"
            )

        except Exception as e:
            # TF가 아직 준비 안 됐거나 에러 발생
            pass  # 조용히 무시 (너무 많이 출력되면 로그가 지저분해짐)


def main(args=None):
    rclpy.init(args=args)
    monitor = ServoMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
