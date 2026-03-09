#!/usr/bin/env python3
"""
Neck Feetech Controller
- Subscribes to /quest_head_euler (Vector3Stamped, degrees)
- Maps headset yaw (left/right) → Feetech motor ID 7
- Maps headset pitch (up/down) → Feetech motor ID 8
- Uses scservo_sdk for STS/SCS servo communication
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

import scservo_sdk as scs
import time
import math


# STS servo control table addresses
ADDR_STS_TORQUE_ENABLE = 40
ADDR_STS_GOAL_POSITION = 42
ADDR_STS_PRESENT_POSITION = 56
ADDR_STS_GOAL_SPEED = 46
ADDR_STS_GOAL_ACC = 41


class NeckFeetechController(Node):
    def __init__(self):
        super().__init__('neck_feetech_controller')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('yaw_motor_id', 7)
        self.declare_parameter('pitch_motor_id', 8)
        # Center position of servo (STS3215: 0~4095, center=2048)
        self.declare_parameter('yaw_center', 2048)
        self.declare_parameter('pitch_center', 2048)
        # Degrees per servo tick (STS3215: 360/4096 ≈ 0.088 deg/tick)
        self.declare_parameter('deg_per_tick', 0.088)
        # Max rotation range in degrees from center
        self.declare_parameter('yaw_max_deg', 90.0)
        self.declare_parameter('pitch_max_deg', 45.0)
        # Servo speed (0=max, otherwise steps/sec)
        self.declare_parameter('servo_speed', 0)
        # Servo acceleration
        self.declare_parameter('servo_acc', 50)
        # Calibration duration in seconds
        self.declare_parameter('calibration_duration', 2.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.yaw_id = self.get_parameter('yaw_motor_id').value
        self.pitch_id = self.get_parameter('pitch_motor_id').value
        self.yaw_center = self.get_parameter('yaw_center').value
        self.pitch_center = self.get_parameter('pitch_center').value
        self.deg_per_tick = self.get_parameter('deg_per_tick').value
        self.yaw_max_deg = self.get_parameter('yaw_max_deg').value
        self.pitch_max_deg = self.get_parameter('pitch_max_deg').value
        self.servo_speed = self.get_parameter('servo_speed').value
        self.servo_acc = self.get_parameter('servo_acc').value
        self.calibration_duration = self.get_parameter('calibration_duration').value

        # Rate limiting (avoid flooding serial bus)
        self._last_cmd_time = 0.0
        self._cmd_interval = 1.0 / 30.0  # 30Hz max

        # Calibration state
        self.calibrated = False
        self.cal_start_time = None
        self.cal_yaw_samples = []
        self.cal_pitch_samples = []
        self.cal_yaw_offset = 0.0
        self.cal_pitch_offset = 0.0

        # Delta-based tracking state (avoids 0°/360° wraparound flip)
        self._prev_head_yaw = None
        self._prev_head_pitch = None
        self._accumulated_yaw = 0.0
        self._accumulated_pitch = 0.0

        # Initialize serial port
        self.port_handler = scs.PortHandler(self.serial_port)
        self.packet_handler = scs.PacketHandler(0)  # Protocol version 0 (SCS/STS)

        if not self.port_handler.openPort():
            self.get_logger().error(f'Failed to open port {self.serial_port}')
            return

        if not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().error(f'Failed to set baudrate {self.baudrate}')
            return

        self.get_logger().info(f'Serial port {self.serial_port} opened at {self.baudrate} baud')

        # Move servos to center (2048) to stay in safe range
        self.yaw_center = 2048
        self.pitch_center = 2048
        self._write_position(self.yaw_id, self.yaw_center)
        self._write_position(self.pitch_id, self.pitch_center)
        time.sleep(1.0)
        self.get_logger().info('Servos moved to center position: 2048')

        # Enable torque and set speed/acc for both motors
        self._setup_motor(self.yaw_id, 'yaw')
        self._setup_motor(self.pitch_id, 'pitch')

        # Subscribe to head euler
        self.sub = self.create_subscription(
            Vector3Stamped,
            '/quest_head_euler',
            self.head_euler_callback,
            10
        )

        self.get_logger().info(
            f'Neck controller ready: yaw=ID{self.yaw_id}, pitch=ID{self.pitch_id}. '
            f'Waiting for head tracking data to calibrate...'
        )

    def _setup_motor(self, motor_id, name):
        """Enable torque and set speed/acceleration for a motor."""
        # Set acceleration
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_STS_GOAL_ACC, self.servo_acc)
        if result != scs.COMM_SUCCESS:
            self.get_logger().warn(f'Motor {name}(ID{motor_id}) acc write failed: {self.packet_handler.getTxRxResult(result)}')

        # Set speed
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, ADDR_STS_GOAL_SPEED, self.servo_speed)
        if result != scs.COMM_SUCCESS:
            self.get_logger().warn(f'Motor {name}(ID{motor_id}) speed write failed: {self.packet_handler.getTxRxResult(result)}')

        # Enable torque
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_STS_TORQUE_ENABLE, 1)
        if result != scs.COMM_SUCCESS:
            self.get_logger().warn(f'Motor {name}(ID{motor_id}) torque enable failed: {self.packet_handler.getTxRxResult(result)}')
        else:
            self.get_logger().info(f'Motor {name}(ID{motor_id}) enabled')

    def _read_position(self, motor_id):
        """Read current position from motor."""
        position, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, ADDR_STS_PRESENT_POSITION)
        if result != scs.COMM_SUCCESS:
            self.get_logger().warn(f'Position read failed for ID{motor_id}, using 2048 as fallback')
            return 2048
        return position

    def _write_position(self, motor_id, position):
        """Write goal position to motor."""
        position = max(0, min(4095, int(position)))
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, ADDR_STS_GOAL_POSITION, position)
        if result != scs.COMM_SUCCESS:
            self.get_logger().warn(f'Position write failed for ID{motor_id}: {self.packet_handler.getTxRxResult(result)}')

    def head_euler_callback(self, msg):
        """Process head euler angles and send to motors."""
        head_pitch = msg.vector.x  # Quest euler X = pitch (up/down)
        head_yaw = msg.vector.y    # Quest euler Y = yaw (left/right)

        # Calibration phase: collect samples for offset
        if not self.calibrated:
            now = time.time()
            if self.cal_start_time is None:
                self.cal_start_time = now
                self.get_logger().info('Calibrating head position...')

            self.cal_yaw_samples.append(head_yaw)
            self.cal_pitch_samples.append(head_pitch)

            if now - self.cal_start_time >= self.calibration_duration:
                self.cal_yaw_offset = sum(self.cal_yaw_samples) / len(self.cal_yaw_samples)
                self.cal_pitch_offset = sum(self.cal_pitch_samples) / len(self.cal_pitch_samples)
                self.calibrated = True
                self.get_logger().info(
                    f'Calibration complete! Offset: yaw={self.cal_yaw_offset:.1f}, '
                    f'pitch={self.cal_pitch_offset:.1f} (samples={len(self.cal_yaw_samples)})'
                )
            return

        # Rate limit serial commands
        now = time.time()
        if now - self._last_cmd_time < self._cmd_interval:
            return
        self._last_cmd_time = now

        # Delta-based tracking to avoid 0°/360° boundary flip
        # The old modulo formula causes a full servo flip when head_yaw crosses
        # (cal_yaw_offset ± 180°), which happens when user turns past yaw_max_deg
        # by more than (180° - yaw_max_deg). With yaw_max_deg=110°, only 70° past
        # the limit triggers the flip. Fix: accumulate deltas instead.
        if self._prev_head_yaw is None:
            self._prev_head_yaw = head_yaw
            self._prev_head_pitch = head_pitch
            self._accumulated_yaw = 0.0
            self._accumulated_pitch = 0.0
        else:
            dyaw = head_yaw - self._prev_head_yaw
            if dyaw > 180.0:
                dyaw -= 360.0
            elif dyaw < -180.0:
                dyaw += 360.0

            dpitch = head_pitch - self._prev_head_pitch
            if dpitch > 180.0:
                dpitch -= 360.0
            elif dpitch < -180.0:
                dpitch += 360.0

            # Reject glitch spikes: >90° per frame at 30Hz = 2700°/s (physically impossible)
            if abs(dyaw) < 90.0:
                self._accumulated_yaw += dyaw
            if abs(dpitch) < 90.0:
                self._accumulated_pitch += dpitch

            self._prev_head_yaw = head_yaw
            self._prev_head_pitch = head_pitch

        rel_yaw = self._accumulated_yaw
        rel_pitch = self._accumulated_pitch

        # Clamp to max range
        rel_yaw = max(-self.yaw_max_deg, min(self.yaw_max_deg, rel_yaw))
        rel_pitch = max(-self.pitch_max_deg, min(self.pitch_max_deg, rel_pitch))

        # Convert degrees to servo ticks
        yaw_ticks = int(rel_yaw / self.deg_per_tick)
        pitch_ticks = int(rel_pitch / self.deg_per_tick)

        # Calculate goal positions
        yaw_goal = self.yaw_center + yaw_ticks
        pitch_goal = self.pitch_center + pitch_ticks

        # Send to motors
        self._write_position(self.yaw_id, yaw_goal)
        self._write_position(self.pitch_id, pitch_goal)

    def destroy_node(self):
        """Cleanup: move to center and disable torque."""
        self.get_logger().info('Shutting down neck controller...')
        self._write_position(self.yaw_id, self.yaw_center)
        self._write_position(self.pitch_id, self.pitch_center)
        time.sleep(0.5)

        # Disable torque
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.yaw_id, ADDR_STS_TORQUE_ENABLE, 0)
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.pitch_id, ADDR_STS_TORQUE_ENABLE, 0)

        self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NeckFeetechController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
