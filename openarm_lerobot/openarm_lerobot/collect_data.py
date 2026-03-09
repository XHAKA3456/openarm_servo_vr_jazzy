"""
OpenArm + LeRobot data collection entry point.

Usage:
    python collect_data.py --config config/collect_data.yaml

[현재 아키텍처 - 방법 B]
  ros2_topic 타입 카메라를 위해 ROS2Subscribers를 먼저 초기화하고,
  이미지 토픽을 등록한 뒤 getter를 CameraManager에 주입한다.
  camera_tcp_streamer.py가 실행 중이어야 /camera/head/color/raw 등이 publish된다.

[방법 A로 전환 시 — 스트리머 없이 직접 하드웨어 접근]
  1. collect_data.yaml: head type: "intelrealsense" 로 변경
  2. 아래 코드에서 image_getters 블록 삭제
  3. subscribers 초기화 순서를 cameras 뒤로 되돌림 (또는 순서 무관)
  4. CameraManager(config["cameras"]) — image_getters 인자 제거
  5. cameras.py: ros2_topic 분기 삭제
"""

import argparse
import logging
import time
from pathlib import Path

import yaml
import rclpy

from .cameras import CameraManager
from .ros2_subscribers import ROS2Subscribers
from .dataset_builder import create_dataset
from .recorder import run_recording


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    logger = logging.getLogger(__name__)

    parser = argparse.ArgumentParser(description="OpenArm + LeRobot data collection")
    parser.add_argument("--config", type=str, required=True, help="Path to YAML config file")
    args = parser.parse_args()

    config_path = Path(args.config).expanduser()
    with open(config_path) as f:
        config = yaml.safe_load(f)

    logger.info(f"Config loaded from {config_path}")

    rclpy.init()

    cameras = None
    subscribers = None

    try:
        # [방법 B] ros2_topic 카메라가 있으면 subscribers를 먼저 초기화해야 함.
        # subscribers 노드에 이미지 토픽 subscription을 등록한 뒤,
        # getter 콜러블을 CameraManager에 주입한다.
        # [방법 A로 전환 시] subscribers 초기화를 cameras 뒤로 옮기고 image_getters 블록 삭제.
        subscribers = ROS2Subscribers(config["topics"])

        # ros2_topic 카메라 토픽 등록 및 getter 생성
        image_getters = {}
        for name, cfg in config["cameras"].items():
            if cfg.get("type") == "ros2_topic":
                topic = cfg["topic"]
                subscribers.add_image_topic(topic)
                image_getters[name] = lambda t=topic: subscribers.get_image(t)

                if cfg.get("use_depth", False) and cfg.get("depth_topic"):
                    depth_topic = cfg["depth_topic"]
                    subscribers.add_image_topic(depth_topic)
                    image_getters[f"{name}_depth"] = lambda t=depth_topic: subscribers.get_image(t)

        logger.info("ROS 2 subscribers initialized")

        # ros2_topic 카메라가 있으면 첫 프레임 수신까지 대기 (최대 10초)
        ros2_cam_names = [n for n, c in config["cameras"].items() if c.get("type") == "ros2_topic"]
        if ros2_cam_names:
            # 대기할 토픽 목록: color + depth(use_depth=true인 경우)
            wait_topics = []
            for n in ros2_cam_names:
                cfg = config["cameras"][n]
                wait_topics.append(cfg["topic"])
                if cfg.get("use_depth", False) and cfg.get("depth_topic"):
                    wait_topics.append(cfg["depth_topic"])

            logger.info(f"Waiting for ROS2 camera topics: {wait_topics} ...")
            deadline = time.time() + 10.0
            while time.time() < deadline:
                rclpy.spin_once(subscribers, timeout_sec=0.1)
                if all(subscribers.get_image(t) is not None for t in wait_topics):
                    logger.info("ROS2 camera topics ready")
                    break
            else:
                missing = [t for t in wait_topics if subscribers.get_image(t) is None]
                raise RuntimeError(
                    f"ROS2 camera topics not ready after 10s. "
                    f"Is camera_tcp_streamer running? Missing: {missing}"
                )

        # 카메라 초기화 (ros2_topic은 하드웨어 연결 없음)
        cameras = CameraManager(config["cameras"], image_getters=image_getters)
        cameras.connect_all()
        logger.info(f"Cameras connected: {cameras.names}")

        # 데이터셋 생성
        dataset = create_dataset(
            dataset_config=config["dataset"],
            camera_configs=config["cameras"],
            resume=config.get("resume", False),
        )
        logger.info(f"Dataset ready: {config['dataset']['repo_id']}")

        # 녹화 시작
        run_recording(
            subscribers=subscribers,
            cameras=cameras,
            dataset=dataset,
            config=config,
        )

    except KeyboardInterrupt:
        logger.info("Interrupted by user (Ctrl+C)")
    finally:
        if cameras is not None:
            cameras.disconnect_all()
            logger.info("Cameras disconnected")
        if subscribers is not None:
            subscribers.destroy_node()
        rclpy.shutdown()
        logger.info("Done.")


if __name__ == "__main__":
    main()
