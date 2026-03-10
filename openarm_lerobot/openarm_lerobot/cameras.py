"""
Camera manager for OpenArm data collection.
Creates RealSense/OpenCV/ROS2-topic cameras from YAML config with unified interface.

[현재 아키텍처 - 방법 B]
  "ros2_topic" 타입: camera_tcp_streamer.py가 publish한 ROS2 토픽에서 프레임을 받음.
  하드웨어에 직접 접근하지 않으므로 카메라 스트리머와 충돌 없음.
  image_getters dict를 통해 ros2_subscribers.get_image() 콜러블을 주입받음.

[방법 A로 전환 시 — 스트리머 없이 직접 하드웨어 접근]
  1. ros2_topic 분기 전체 삭제 (CameraManager.__init__, connect_all, read_all)
  2. image_getters 파라미터 삭제
  3. collect_data.yaml: head type: "ros2_topic" → "intelrealsense"
  4. collect_data.py: image_getters 관련 코드 삭제, cameras 먼저 초기화
"""

from typing import Callable
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
from numpy.typing import NDArray


class CameraManager:
    """Manages multiple cameras (RealSense/OpenCV/ROS2-topic) from YAML config."""

    def __init__(self, camera_configs: dict, image_getters: dict[str, Callable] | None = None):
        self._cameras = {}
        self._configs = camera_configs
        # [방법 B] ros2_topic 카메라용 getter dict. 방법 A로 전환 시 이 인자 삭제.
        self._image_getters = image_getters or {}

        for name, cfg in camera_configs.items():
            cam_type = cfg["type"]
            if cam_type == "intelrealsense":
                self._cameras[name] = self._create_realsense(name, cfg)
            elif cam_type == "opencv":
                self._cameras[name] = self._create_opencv(name, cfg)
            elif cam_type == "ros2_topic":
                pass  # [방법 B] 하드웨어 없음 — image_getters로 프레임 수신
            else:
                raise ValueError(f"Unknown camera type: {cam_type} for camera '{name}'")

    def _create_realsense(self, name: str, cfg: dict):
        from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
        from lerobot.cameras.realsense.camera_realsense import RealSenseCamera

        config = RealSenseCameraConfig(
            serial_number_or_name=cfg["serial_number"],
            fps=cfg.get("fps"),
            width=cfg.get("width"),
            height=cfg.get("height"),
            use_depth=cfg.get("use_depth", False),
        )
        return RealSenseCamera(config)

    def _create_opencv(self, name: str, cfg: dict):
        from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
        from lerobot.cameras.opencv.camera_opencv import OpenCVCamera

        config = OpenCVCameraConfig(
            index_or_path=cfg["index"],
            fps=cfg.get("fps"),
            width=cfg.get("width"),
            height=cfg.get("height"),
        )
        return OpenCVCamera(config)

    def connect_all(self):
        """Connect hardware cameras. ros2_topic cameras skip (no hardware)."""
        for name, cam in self._cameras.items():
            cam.connect()

    def disconnect_all(self):
        """Disconnect hardware cameras."""
        for name, cam in self._cameras.items():
            if cam.is_connected:
                cam.disconnect()

    def read_all(self) -> dict[str, NDArray]:
        """Read frames from all cameras.

        Returns:
            dict with keys like:
                "{cam_name}":       RGB frame (H, W, 3)
                "{cam_name}_depth": Depth colormap (H, W, 3) — RealSense/ros2_topic with use_depth=True
        """
        frames = {}
        for name, cfg in self._configs.items():
            cam_type = cfg.get("type")

            # [방법 B] ros2_topic: image_getters의 콜러블 호출로 프레임 수신
            # [방법 A로 전환 시] 아래 if 블록 전체 삭제
            if cam_type == "ros2_topic":
                getter = self._image_getters.get(name)
                if getter:
                    frame = getter()
                    if frame is None:
                        raise RuntimeError(
                            f"ros2_topic camera '{name}' has no frame yet. "
                            f"Is camera_tcp_streamer running and publishing?"
                        )
                    frames[name] = frame

                if cfg.get("use_depth", False):
                    depth_getter = self._image_getters.get(f"{name}_depth")
                    if depth_getter:
                        depth_frame = depth_getter()
                        if depth_frame is not None:
                            frames[f"{name}_depth"] = depth_frame
                continue

        # opencv 카메라는 병렬로 동시에 읽어 카메라 간 sync 최소화
        opencv_names = [
            n for n, c in self._configs.items()
            if c.get("type") == "opencv"
        ]
        if opencv_names:
            def _read_opencv(n):
                return n, self._cameras[n].async_read()

            with ThreadPoolExecutor(max_workers=len(opencv_names)) as ex:
                for name, frame in ex.map(_read_opencv, opencv_names):
                    frames[name] = frame

        # 하드웨어 카메라 중 intelrealsense 처리
        for name, cfg in self._configs.items():
            cam_type = cfg.get("type")
            if cam_type != "intelrealsense":
                continue

            use_depth = cfg.get("use_depth", False)
            cam = self._cameras[name]

            if use_depth:
                # try_wait_for_frames() 한 번으로 color + depth를 같은 frameset에서 추출.
                # read() → read_depth() 두 번 호출하면 서로 다른 frameset을 소비하거나
                # 백그라운드 스레드와 파이프라인 충돌이 발생함.
                ret, frameset = cam.rs_pipeline.try_wait_for_frames(timeout_ms=500)
                if not ret:
                    raise RuntimeError(f"RealSense({name}) frameset read failed")

                # .copy()로 파이프라인 내부 버퍼 참조를 끊어야 함.
                color_raw = np.asanyarray(frameset.get_color_frame().get_data()).copy()
                frames[name] = color_raw

                depth_raw = np.asanyarray(frameset.get_depth_frame().get_data()).copy()
                del frameset  # 파이프라인 버퍼 즉시 해제
                min_m = cfg.get("depth_min_m", 0.3)
                max_m = cfg.get("depth_max_m", 1.5)
                depth_m = depth_raw.astype(np.float32) / 1000.0
                depth_norm = np.clip((depth_m - min_m) / (max_m - min_m), 0.0, 1.0)
                depth_uint8 = (depth_norm * 255).astype(np.uint8)
                colormap_bgr = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_TURBO)
                frames[f"{name}_depth"] = cv2.cvtColor(colormap_bgr, cv2.COLOR_BGR2RGB)
            else:
                frames[name] = cam.async_read()

        return frames

    @property
    def names(self) -> list[str]:
        """Return all camera names (hardware + ros2_topic)."""
        return list(self._configs.keys())

    @property
    def configs(self) -> dict:
        """Return camera configs."""
        return self._configs
