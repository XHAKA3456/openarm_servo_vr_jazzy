#!/usr/bin/env python3
"""
camera_tcp_streamer.py

[현재 아키텍처 - 방법 B]
  이 노드가 RealSense 하드웨어를 "단독으로" 소유한다.
  - Quest VR 헤드셋으로 RGB를 TCP 스트리밍 (포트 5656)
  - RGB + Depth colormap을 ROS2 토픽으로 publish
      /camera/head/color/raw       (sensor_msgs/Image, rgb8)
      /camera/head/depth/colormap  (sensor_msgs/Image, rgb8)
  - collect_data.py는 하드웨어에 직접 접근하지 않고 위 토픽을 구독해서 수집

[이 방식의 트레이드오프]
  장점: VR 스트리밍이 데이터 수집과 무관하게 항상 동작
  단점: 이 노드(카메라 스트리머)가 실행 중이어야만 데이터 수집 가능
        → 육안으로 로봇 보면서 조종할 때 런치 파일에서 카메라 노드를 빼면
          데이터 수집도 못 하는 문제가 생김

[방법 A로 돌아가는 방법 - 육안 조종 + 데이터 수집이 필요할 때]
  1. collect_data.yaml: head 카메라 type을 "ros2_topic" → "intelrealsense" 로 변경
  2. collect_data.py: 원래 방식으로 cameras 먼저 초기화 (image_getters 제거)
  3. cameras.py: ros2_topic 분기 삭제 (intelrealsense/opencv만 유지)
  4. 이 파일(camera_tcp_streamer.py): 런치 파일에서 제거하거나 비활성화
  → cameras.py가 RealSense를 직접 열고, TCP 스트리머는 사용하지 않음

TCP 프로토콜:
  각 메시지: [length(4B, big-endian)][JPEG data]
  Quest 앱은 4바이트 읽고 → 해당 길이만큼 JPEG 수신 → 디코딩
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage

import cv2
import numpy as np
import pyrealsense2 as rs
import socket
import struct
import threading
import time


class CameraTCPStreamer(Node):
    def __init__(self):
        super().__init__('camera_tcp_streamer')

        self.declare_parameter('serial_number', '348522076238')
        self.declare_parameter('port', 5656)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_min_m', 0.3)
        self.declare_parameter('depth_max_m', 1.5)
        self.declare_parameter('stream_to_quest', True)

        self.serial_number = self.get_parameter('serial_number').value
        self.port = self.get_parameter('port').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.use_depth = self.get_parameter('use_depth').value
        self.depth_min_m = self.get_parameter('depth_min_m').value
        self.depth_max_m = self.get_parameter('depth_max_m').value
        self.stream_to_quest = self.get_parameter('stream_to_quest').value

        self.running = True
        self.client_sock = None
        self.client_lock = threading.Lock()

        # RealSense 파이프라인 — 이 노드가 단독 소유
        self.pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_device(self.serial_number)
        rs_config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        if self.use_depth:
            rs_config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            self._align = rs.align(rs.stream.color)  # depth → color 해상도 정렬

        self.pipeline.start(rs_config)
        self.get_logger().info(
            f'RealSense 열림: S/N={self.serial_number} '
            f'({self.width}x{self.height} @ {self.fps}fps, depth={self.use_depth})'
        )

        # RealSense depth 후처리 필터 (decimation 제외 — head 카메라는 full resolution 유지)
        if self.use_depth:
            self._depth2disp = rs.disparity_transform(True)
            self._spatial    = rs.spatial_filter()
            self._spatial.set_option(rs.option.filter_magnitude, 2)       # 기본값, 너무 높으면 blur
            self._spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
            self._spatial.set_option(rs.option.filter_smooth_delta, 20)
            self._temporal   = rs.temporal_filter()
            self._temporal.set_option(rs.option.filter_smooth_alpha, 0.1)
            self._disp2depth = rs.disparity_transform(False)
            self._hole       = rs.hole_filling_filter()

        # depth 필터 워밍업 — temporal filter가 안정되기 전까지 publish 안 함
        if self.use_depth:
            WARMUP_FRAMES = 30
            self.get_logger().info(f'depth 필터 워밍업 중 ({WARMUP_FRAMES}프레임)...')
            for _ in range(WARMUP_FRAMES):
                try:
                    raw = self.pipeline.wait_for_frames(timeout_ms=200)
                    aligned = self._align.process(raw)
                    d = aligned.get_depth_frame()
                    if d:
                        self._apply_depth_filters(d)
                except RuntimeError:
                    pass
            self.get_logger().info('depth 필터 워밍업 완료')

        # ROS2 토픽 퍼블리셔 — collect_data.py가 이 토픽을 구독함
        self.color_pub = self.create_publisher(RosImage, '/camera/head/color/raw', 1)
        if self.use_depth:
            self.depth_pub = self.create_publisher(RosImage, '/camera/head/depth/colormap', 1)

        # TCP 서버 소켓 — Quest VR 스트리밍용
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', self.port))
        self.server_sock.listen(1)
        if self.stream_to_quest:
            self.get_logger().info(f'TCP 서버 대기 중: 0.0.0.0:{self.port}')
            self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
            self._accept_thread.start()
        else:
            self.get_logger().info('stream_to_quest=false: Quest 영상 전송 비활성화')

        self.timer = self.create_timer(1.0 / self.fps, self._stream_frame)

        self._frame_count = 0
        self._last_fps_time = time.time()
        self.create_timer(5.0, self._log_fps)

    def _accept_loop(self):
        """Quest 연결 수락 (백그라운드). 기존 연결 있으면 교체."""
        while self.running:
            try:
                self.server_sock.settimeout(1.0)
                conn, addr = self.server_sock.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info(f'Quest 연결됨: {addr[0]}:{addr[1]}')
                with self.client_lock:
                    if self.client_sock is not None:
                        try:
                            self.client_sock.close()
                        except Exception:
                            pass
                    self.client_sock = conn
            except socket.timeout:
                continue
            except OSError:
                break

    def _publish_image(self, publisher, frame_rgb: np.ndarray):
        """RGB numpy 배열을 sensor_msgs/Image로 publish."""
        msg = RosImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = frame_rgb.shape[0]
        msg.width = frame_rgb.shape[1]
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3
        msg.data = frame_rgb.tobytes()
        publisher.publish(msg)

    def _apply_depth_filters(self, depth_frame) -> np.ndarray:
        """rs2 depth frame → 후처리 필터 적용 → uint16 numpy (mm)."""
        depth_frame = self._depth2disp.process(depth_frame)
        depth_frame = self._spatial.process(depth_frame)
        depth_frame = self._temporal.process(depth_frame)
        depth_frame = self._disp2depth.process(depth_frame)
        depth_frame = self._hole.process(depth_frame)
        return np.asanyarray(depth_frame.get_data()).copy()

    def _depth_colormap(self, depth_frame) -> np.ndarray:
        """rs2 depth frame → 후처리 필터 → TURBO colormap RGB."""
        depth_raw = self._apply_depth_filters(depth_frame)
        depth_m = depth_raw.astype(np.float32) / 1000.0
        # 0값(측정 실패 픽셀) → max 거리로 처리해서 검정 hole 방지
        depth_m[depth_m == 0] = self.depth_max_m
        depth_norm = np.clip(
            (depth_m - self.depth_min_m) / (self.depth_max_m - self.depth_min_m),
            0.0, 1.0
        )
        colormap_bgr = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        return cv2.cvtColor(colormap_bgr, cv2.COLOR_BGR2RGB)

    def _stream_frame(self):
        """
        매 프레임:
          1) RealSense에서 color(+depth) 읽기
          2) ROS2 토픽으로 publish  ← collect_data.py가 여기서 받음
          3) JPEG 인코딩 후 Quest로 TCP 전송  ← VR 시야
        """
        try:
            raw_frames = self.pipeline.wait_for_frames(timeout_ms=200)
        except RuntimeError:
            self.get_logger().warn('RealSense 프레임 타임아웃')
            return

        frames = self._align.process(raw_frames) if self.use_depth else raw_frames

        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        frame_bgr = np.asanyarray(color_frame.get_data()).copy()
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # ROS2 publish (데이터 수집용)
        self._publish_image(self.color_pub, frame_rgb)
        if self.use_depth:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                self._publish_image(self.depth_pub, self._depth_colormap(depth_frame))

        # Quest TCP 전송 (VR 시야용) — 연결 없거나 비활성화면 스킵
        if not self.stream_to_quest:
            self._frame_count += 1
            return
        with self.client_lock:
            client = self.client_sock
        if client is not None:
            ret, jpeg = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ret:
                data = struct.pack('>I', len(jpeg.tobytes())) + jpeg.tobytes()
                try:
                    client.sendall(data)
                except OSError:
                    self.get_logger().info('Quest 연결 끊김, 재연결 대기 중...')
                    with self.client_lock:
                        if self.client_sock is client:
                            try:
                                self.client_sock.close()
                            except Exception:
                                pass
                            self.client_sock = None

        self._frame_count += 1

    def _log_fps(self):
        now = time.time()
        elapsed = now - self._last_fps_time
        if elapsed > 0:
            with self.client_lock:
                connected = self.client_sock is not None
            self.get_logger().info(
                f'스트리밍: {self._frame_count / elapsed:.1f} fps | '
                f'Quest: {"연결됨" if connected else "대기 중"}'
            )
        self._frame_count = 0
        self._last_fps_time = now

    def destroy_node(self):
        self.running = False
        self.server_sock.close()
        with self.client_lock:
            if self.client_sock is not None:
                self.client_sock.close()
        self.pipeline.stop()
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
