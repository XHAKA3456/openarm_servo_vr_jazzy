"""
Camera manager for OpenArm data collection.
Creates RealSense/OpenCV cameras from YAML config with unified interface.
"""

import numpy as np
from numpy.typing import NDArray


class CameraManager:
    """Manages multiple cameras (RealSense/OpenCV) from YAML config."""

    def __init__(self, camera_configs: dict):
        self._cameras = {}
        self._configs = camera_configs

        for name, cfg in camera_configs.items():
            cam_type = cfg["type"]
            if cam_type == "intelrealsense":
                self._cameras[name] = self._create_realsense(name, cfg)
            elif cam_type == "opencv":
                self._cameras[name] = self._create_opencv(name, cfg)
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
        """Connect all cameras."""
        for name, cam in self._cameras.items():
            cam.connect()

    def disconnect_all(self):
        """Disconnect all cameras."""
        for name, cam in self._cameras.items():
            if cam.is_connected:
                cam.disconnect()

    def read_all(self) -> dict[str, NDArray]:
        """Read frames from all cameras.

        Returns:
            dict with keys like:
                "{cam_name}": RGB frame (H, W, 3)
                "{cam_name}_depth": Depth frame (H, W) - only for RealSense with use_depth=True
        """
        frames = {}
        for name, cam in self._cameras.items():
            frames[name] = cam.async_read()

            cfg = self._configs[name]
            if cfg.get("use_depth", False) and cfg["type"] == "intelrealsense":
                depth = cam.read_depth()
                frames[f"{name}_depth"] = np.repeat(depth[:, :, np.newaxis], 3, axis=2)

        return frames

    @property
    def names(self) -> list[str]:
        """Return camera names."""
        return list(self._cameras.keys())

    @property
    def configs(self) -> dict:
        """Return camera configs."""
        return self._configs
