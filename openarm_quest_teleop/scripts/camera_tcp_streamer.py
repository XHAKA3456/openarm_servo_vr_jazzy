#!/usr/bin/env python3
"""
camera_tcp_streamer.py  (이름은 레거시 — 실제로는 UDP 전송)

[현재 아키텍처 - 방법 B]
  이 노드가 RealSense 하드웨어를 "단독으로" 소유한다.
  - Quest VR 헤드셋으로 RGB를 UDP 스트리밍 (포트 5656)
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

UDP 프로토콜 (per-packet):
  [frame_id(2B, big-endian)][eye(1B: 0=left, 1=right, 0xFF=mono)][JPEG data]
  Quest에서 "HI" (0x48 0x49) 등록 패킷을 보내면 해당 주소로 프레임 전송
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


def synthesize_stereo(color_image, depth_image_mm, intrinsics, ipd=0.063):
    """
    RGB + Depth로 좌/우 눈 스테레오 이미지를 합성한다.

    Args:
        color_image: BGR 이미지 (H, W, 3)
        depth_image_mm: uint16 depth (mm 단위, 0=invalid)
        intrinsics: pyrealsense2 intrinsics (fx, fy, ppx, ppy)
        ipd: 동공간 거리 (미터, 기본 0.063)

    Returns:
        left_bgr, right_bgr: 좌/우 눈 BGR 이미지
    """
    h, w = color_image.shape[:2]
    fx = intrinsics.fx
    fy = intrinsics.fy
    cx = intrinsics.ppx
    cy = intrinsics.ppy

    # 깊이를 미터로 변환, 0은 무한대로 처리
    depth_m = depth_image_mm.astype(np.float32) / 1000.0
    valid_mask = depth_m > 0
    depth_m[~valid_mask] = 10.0  # invalid → 먼 거리 (시차 최소화)

    # 픽셀 좌표 그리드
    u = np.arange(w, dtype=np.float32)
    v = np.arange(h, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)

    # 카메라 좌표로 역투영: X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy
    X = (uu - cx) * depth_m / fx
    Y = (vv - cy) * depth_m / fy
    Z = depth_m

    half_ipd = ipd / 2.0

    # 좌안: 카메라를 -half_ipd만큼 이동 → X' = X + half_ipd
    # 재투영: u' = fx * (X + half_ipd) / Z + cx
    left_u = fx * (X + half_ipd) / Z + cx
    left_v = vv.copy()

    # 우안: 카메라를 +half_ipd만큼 이동 → X' = X - half_ipd
    right_u = fx * (X - half_ipd) / Z + cx
    right_v = vv.copy()

    # cv2.remap으로 효율적 리매핑
    left_bgr = cv2.remap(
        color_image,
        left_u, left_v,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE
    )
    right_bgr = cv2.remap(
        color_image,
        right_u, right_v,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE
    )

    return left_bgr, right_bgr


class CameraTCPStreamer(Node):
    def __init__(self):
        super().__init__('camera_tcp_streamer')

        self.declare_parameter('serial_number', '348522076238')
        self.declare_parameter('port', 5656)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 55)  # UDP용: 65KB에 맞추기 위해 55
        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_min_m', 0.3)
        self.declare_parameter('depth_max_m', 1.5)
        self.declare_parameter('stream_to_quest', True)
        self.declare_parameter('stereo_mode', True)
        self.declare_parameter('ipd', 0.063)

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
        self.stereo_mode = self.get_parameter('stereo_mode').value
        self.ipd = self.get_parameter('ipd').value

        # stereo_mode에는 depth가 필수
        if self.stereo_mode and not self.use_depth:
            self.get_logger().warn('stereo_mode=True이지만 use_depth=False입니다. use_depth를 True로 강제합니다.')
            self.use_depth = True

        self.running = True

        # UDP 소켓 — Quest 주소 등록 방식
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp_sock.bind(('0.0.0.0', self.port))
        self._udp_sock.setblocking(False)
        self._quest_addr = None  # (ip, port) — Quest 등록 시 설정
        self._quest_lock = threading.Lock()
        self._frame_id = 0

        # Quest 등록 패킷 수신 스레드
        if self.stream_to_quest:
            self._register_thread = threading.Thread(target=self._register_loop, daemon=True)
            self._register_thread.start()
            self.get_logger().info(f'UDP 서버 대기 중: 0.0.0.0:{self.port} (Quest 등록 대기)')
        else:
            self.get_logger().info('stream_to_quest=false: Quest 영상 전송 비활성화')

        # RealSense 파이프라인 — 이 노드가 단독 소유
        self.pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_device(self.serial_number)
        rs_config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        if self.use_depth:
            rs_config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            self._align = rs.align(rs.stream.color)

        profile = self.pipeline.start(rs_config)

        # 카메라 intrinsics 저장 (스테레오 합성에 사용)
        if self.use_depth:
            color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
            self._intrinsics = color_profile.get_intrinsics()
            self.get_logger().info(
                f'Intrinsics: fx={self._intrinsics.fx:.1f} fy={self._intrinsics.fy:.1f} '
                f'cx={self._intrinsics.ppx:.1f} cy={self._intrinsics.ppy:.1f}'
            )

        self.get_logger().info(
            f'RealSense 열림: S/N={self.serial_number} '
            f'({self.width}x{self.height} @ {self.fps}fps, depth={self.use_depth}, '
            f'stereo={self.stereo_mode}, ipd={self.ipd})'
        )

        # RealSense depth 후처리 필터
        if self.use_depth:
            self._depth2disp = rs.disparity_transform(True)
            self._spatial    = rs.spatial_filter()
            self._spatial.set_option(rs.option.filter_magnitude, 2)
            self._spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
            self._spatial.set_option(rs.option.filter_smooth_delta, 20)
            self._temporal   = rs.temporal_filter()
            self._temporal.set_option(rs.option.filter_smooth_alpha, 0.1)
            self._disp2depth = rs.disparity_transform(False)
            self._hole       = rs.hole_filling_filter()

        # depth 필터 워밍업
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

        # ROS2 토픽 퍼블리셔
        self.color_pub = self.create_publisher(RosImage, '/camera/head/color/raw', 1)
        if self.use_depth:
            self.depth_pub = self.create_publisher(RosImage, '/camera/head/depth/colormap', 1)

        self.timer = self.create_timer(1.0 / self.fps, self._stream_frame)

        self._frame_count = 0
        self._last_fps_time = time.time()
        self.create_timer(5.0, self._log_fps)

    def _register_loop(self):
        """Quest 등록 패킷 수신 (백그라운드). Quest가 "HI" 패킷을 보내면 주소 기억."""
        while self.running:
            try:
                data, addr = self._udp_sock.recvfrom(64)
                if len(data) >= 2 and data[0] == 0x48 and data[1] == 0x49:  # "HI"
                    with self._quest_lock:
                        if self._quest_addr != addr:
                            self.get_logger().info(f'Quest 등록: {addr[0]}:{addr[1]}')
                            self._quest_addr = addr
            except BlockingIOError:
                time.sleep(0.1)
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
        return self._depth_raw_to_colormap(depth_raw)

    def _depth_raw_to_colormap(self, depth_raw: np.ndarray) -> np.ndarray:
        """필터 적용된 uint16 depth (mm) → TURBO colormap RGB."""
        depth_m = depth_raw.astype(np.float32) / 1000.0
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
          1) RealSense에서 최신 프레임만 가져오기 (큐 드레인)
          2) ROS2 토픽으로 publish  ← collect_data.py가 여기서 받음
          3) JPEG 인코딩 후 Quest로 UDP 전송  ← VR 시야
        """
        frame_t0 = time.monotonic()

        # RealSense 큐 드레인 — 쌓인 프레임 버리고 최신만 사용
        try:
            raw_frames = self.pipeline.wait_for_frames(timeout_ms=200)
        except RuntimeError:
            self.get_logger().warn('RealSense 프레임 타임아웃')
            return
        while True:
            success, newer = self.pipeline.try_wait_for_frames(0)
            if not success:
                break
            raw_frames = newer

        frames = self._align.process(raw_frames) if self.use_depth else raw_frames

        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        frame_bgr = np.asanyarray(color_frame.get_data()).copy()
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # ROS2 publish (데이터 수집용)
        self._publish_image(self.color_pub, frame_rgb)

        depth_frame = None
        depth_raw = None  # 필터 결과 캐싱 (temporal filter가 stateful → 1회만 호출)
        if self.use_depth:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_raw = self._apply_depth_filters(depth_frame)
                self._publish_image(self.depth_pub, self._depth_raw_to_colormap(depth_raw))

        # Quest UDP 전송 — 연결 없거나 비활성화면 스킵
        if not self.stream_to_quest:
            self._frame_count += 1
            return

        with self._quest_lock:
            quest_addr = self._quest_addr

        if quest_addr is None:
            self._frame_count += 1
            return

        # 시간 예산 체크
        frame_budget = 1.0 / self.fps
        elapsed = time.monotonic() - frame_t0
        if elapsed > frame_budget * 1.5:
            self.get_logger().debug(
                f'프레임 처리 시간 초과 ({elapsed*1000:.0f}ms), UDP 전송 스킵')
            self._frame_count += 1
            return

        frame_id = self._frame_id & 0xFFFF
        self._frame_id += 1
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]

        try:
            if self.stereo_mode and depth_raw is not None:
                # 스테레오 모드: 좌/우 눈 각각 UDP 패킷
                left_bgr, right_bgr = synthesize_stereo(
                    frame_bgr, depth_raw, self._intrinsics, self.ipd
                )
                ret_l, left_jpeg = cv2.imencode('.jpg', left_bgr, encode_params)
                ret_r, right_jpeg = cv2.imencode('.jpg', right_bgr, encode_params)

                if ret_l:
                    left_bytes = left_jpeg.tobytes()
                    # [frame_id(2B)][eye=0(1B)][JPEG]
                    header = struct.pack('>HB', frame_id, 0)
                    pkt = header + left_bytes
                    if len(pkt) <= 65507:  # UDP max
                        self._udp_sock.sendto(pkt, quest_addr)
                    else:
                        self.get_logger().warn(f'Left eye 패킷 초과: {len(pkt)}B')

                if ret_r:
                    right_bytes = right_jpeg.tobytes()
                    header = struct.pack('>HB', frame_id, 1)
                    pkt = header + right_bytes
                    if len(pkt) <= 65507:
                        self._udp_sock.sendto(pkt, quest_addr)
                    else:
                        self.get_logger().warn(f'Right eye 패킷 초과: {len(pkt)}B')
            else:
                # 모노 모드
                ret, jpeg = cv2.imencode('.jpg', frame_bgr, encode_params)
                if ret:
                    jpeg_bytes = jpeg.tobytes()
                    header = struct.pack('>HB', frame_id, 0xFF)
                    pkt = header + jpeg_bytes
                    if len(pkt) <= 65507:
                        self._udp_sock.sendto(pkt, quest_addr)
                    else:
                        self.get_logger().warn(f'Mono 패킷 초과: {len(pkt)}B')
        except OSError as e:
            self.get_logger().debug(f'UDP send 오류: {e}')
            # UDP는 connectionless — 오류가 나도 주소 유지

        self._frame_count += 1

    def _log_fps(self):
        now = time.time()
        elapsed = now - self._last_fps_time
        if elapsed > 0:
            with self._quest_lock:
                connected = self._quest_addr is not None
            mode = 'stereo' if self.stereo_mode else 'mono'
            addr_str = f'{self._quest_addr[0]}:{self._quest_addr[1]}' if connected else '대기 중'
            self.get_logger().info(
                f'스트리밍 ({mode}): {self._frame_count / elapsed:.1f} fps | '
                f'Quest: {addr_str}'
            )
        self._frame_count = 0
        self._last_fps_time = now

    def destroy_node(self):
        self.running = False
        self._udp_sock.close()
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
