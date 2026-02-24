#!/usr/bin/env python3
"""
camera_tcp_streamer.py
Quest VR 헤드셋으로 카메라 영상을 TCP로 스트리밍하는 노드.

프로토콜: [4바이트 big-endian 크기][JPEG 데이터] 반복
포트: 5656 (TCPVideoReceiver.cs 기본값)
"""

import rclpy
from rclpy.node import Node

import cv2
import socket
import struct
import threading
import time


class CameraTCPStreamer(Node):
    def __init__(self):
        super().__init__('camera_tcp_streamer')

        # 파라미터 선언
        self.declare_parameter('camera_device', 2)       # /dev/video0
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
        self.client_sock = None
        self.client_lock = threading.Lock()

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

        # TCP 서버 시작
        self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self.server_thread.start()

        # 스트리밍 루프 타이머
        interval = 1.0 / self.fps
        self.timer = self.create_timer(interval, self._stream_frame)

        # FPS 모니터링
        self._frame_count = 0
        self._last_fps_time = time.time()
        self.create_timer(5.0, self._log_fps)

        self.get_logger().info(f'TCP 서버 대기 중: 0.0.0.0:{self.port}')

    def _server_loop(self):
        """Quest로부터 연결 수락"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.port))
        server.listen(1)
        server.settimeout(1.0)

        while self.running:
            try:
                conn, addr = server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info(f'Quest 연결됨: {addr[0]}:{addr[1]}')
                with self.client_lock:
                    if self.client_sock:
                        self.client_sock.close()
                    self.client_sock = conn
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().warn(f'서버 오류: {e}')

        server.close()

    def _stream_frame(self):
        """카메라 프레임 캡처 후 연결된 클라이언트에 전송"""
        with self.client_lock:
            if self.client_sock is None:
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

        data = jpeg.tobytes()
        # [4바이트 big-endian 크기][JPEG 데이터]
        packet = struct.pack('>I', len(data)) + data

        with self.client_lock:
            if self.client_sock is None:
                return
            try:
                self.client_sock.sendall(packet)
                self._frame_count += 1
            except (BrokenPipeError, ConnectionResetError, OSError):
                self.get_logger().info('Quest 연결 끊김')
                self.client_sock.close()
                self.client_sock = None

    def _log_fps(self):
        now = time.time()
        elapsed = now - self._last_fps_time
        if elapsed > 0:
            fps = self._frame_count / elapsed
            connected = self.client_sock is not None
            self.get_logger().info(
                f'스트리밍: {fps:.1f} fps | Quest: {"연결됨" if connected else "대기 중"}'
            )
        self._frame_count = 0
        self._last_fps_time = now

    def destroy_node(self):
        self.running = False
        with self.client_lock:
            if self.client_sock:
                self.client_sock.close()
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraTCPStreamer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f'[ERROR] {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
