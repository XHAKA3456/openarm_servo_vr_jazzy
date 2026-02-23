"""
OpenArm + LeRobot data collection entry point.

Usage:
    python collect_data.py --config config/collect_data.yaml
"""

import argparse
import logging
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

    # Parse args
    parser = argparse.ArgumentParser(description="OpenArm + LeRobot data collection")
    parser.add_argument("--config", type=str, required=True, help="Path to YAML config file")
    args = parser.parse_args()

    # Load config
    config_path = Path(args.config).expanduser()
    with open(config_path) as f:
        config = yaml.safe_load(f)

    logger.info(f"Config loaded from {config_path}")

    # Init ROS 2
    rclpy.init()

    cameras = None
    subscribers = None

    try:
        # Create cameras
        cameras = CameraManager(config["cameras"])
        cameras.connect_all()
        logger.info(f"Cameras connected: {cameras.names}")

        # Create ROS 2 subscribers
        subscribers = ROS2Subscribers(config["topics"])
        logger.info("ROS 2 subscribers initialized")

        # Create dataset
        dataset = create_dataset(
            dataset_config=config["dataset"],
            camera_configs=config["cameras"],
            resume=config.get("resume", False),
        )
        logger.info(f"Dataset ready: {config['dataset']['repo_id']}")

        # Run recording
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
