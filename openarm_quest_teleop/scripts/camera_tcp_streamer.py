#!/usr/bin/env python3
"""
camera_udp_streamer.py
Quest VR 헤드셋으로 카메라 영상을 UDP로 스트리밍하는 노드.

프로토콜 (fragmentation):
  각 UDP 패킷: [frame_id(2B)][total_frags(1B)][frag_idx(1B)][JPEG chunk]
  Quest는 모든 fragment를 받으면 JPEG를 재조립.
  UDP이므로 backpressure 없음 → 컨트롤러 데이터에 영향 없음.

포트: 5656 (UDPVideoReceiver.cs 기본값)
"""

import rclpy
from rclpy.node import Node

import cv2
import socket
import struct
import threading
import time


MAX_UDP_PAYLOAD = 60000  # UDP safe max (65507 - margin)
FRAG_HEADER_SIZE = 4     # frame_id(2) + total_frags(1) + frag_idx(1)
MAX_CHUNK_SIZE = MAX_UDP_PAYLOAD - FRAG_HEADER_SIZE


class CameraUDPStreamer(Node):
    def __init__(self):
        super().__init__('camera_tcp_streamer')  # keep node name for compatibility

        # 파라미터 선언
        self.declare_parameter('camera_device', 2)
        self.declare_parameter('port', 5656)
        self.declare_parameter('width', 960)
        self.declare_parameter('height', 540)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 70)

        self.camera_device = self.get_parameter('camera_device').value
        self.port = self.get_parameter('port').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        self.running = True
        self.quest_addr = None
        self.frame_id = 0

        # 카메라 열기
        self.cap = cv2.VideoCapture(self.camera_device)
        if not self.cap.isOpened():
            self.get_logger().error(f'카메라 열기 실패: /dev/video{self.camera_device}')
            raise RuntimeError('Camera open failed')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'카메라 열림: /dev/video{self.camera_device} '
            f'({actual_w}x{actual_h} @ {actual_fps:.0f}fps)'
        )

        # UDP 소켓 생성 (non-blocking for registration check)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)

        # 스트리밍 루프 타이머
        interval = 1.0 / self.fps
        self.timer = self.create_timer(interval, self._stream_frame)

        # FPS 모니터링
        self._frame_count = 0
        self._last_fps_time = time.time()
        self.create_timer(5.0, self._log_fps)

        self.get_logger().info(f'UDP 서버 대기 중: 0.0.0.0:{self.port}')

    def _check_registration(self):
        """Quest로부터 등록 패킷("hello") 확인 (non-blocking)"""
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                if self.quest_addr != addr:
                    self.get_logger().info(f'Quest 등록됨: {addr[0]}:{addr[1]}')
                self.quest_addr = addr
            except BlockingIOError:
                break

    def _stream_frame(self):
        """카메라 프레임 캡처 후 UDP로 전송"""
        # 먼저 등록 패킷 확인
        self._check_registration()

        if self.quest_addr is None:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('프레임 읽기 실패')
            return

        # JPEG 인코딩
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        ret, jpeg = cv2.imencode('.jpg', frame, encode_params)
        if not ret:
            return

        jpeg_bytes = jpeg.tobytes()

        # Fragment and send
        total_frags = (len(jpeg_bytes) + MAX_CHUNK_SIZE - 1) // MAX_CHUNK_SIZE
        if total_frags > 255:
            self.get_logger().warn(f'프레임 너무 큼: {len(jpeg_bytes)} bytes, 건너뜀')
            return

        try:
            for i in range(total_frags):
                offset = i * MAX_CHUNK_SIZE
                chunk = jpeg_bytes[offset:offset + MAX_CHUNK_SIZE]
                header = struct.pack('>HBB', self.frame_id, total_frags, i)
                self.sock.sendto(header + chunk, self.quest_addr)

            self.frame_id = (self.frame_id + 1) % 65536
            self._frame_count += 1
        except OSError:
            self.get_logger().info('Quest 전송 실패')

    def _log_fps(self):
        now = time.time()
        elapsed = now - self._last_fps_time
        if elapsed > 0:
            fps = self._frame_count / elapsed
            connected = self.quest_addr is not None
            self.get_logger().info(
                f'스트리밍: {fps:.1f} fps | Quest: {"등록됨" if connected else "대기 중"}'
            )
        self._frame_count = 0
        self._last_fps_time = now

    def destroy_node(self):
        self.running = False
        self.sock.close()
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraUDPStreamer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f'[ERROR] {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
