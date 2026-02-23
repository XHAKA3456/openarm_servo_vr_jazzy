"""
LeRobot dataset builder for OpenArm data collection.
Handles dataset creation, feature definition, and saving.
"""

import logging
from pathlib import Path

import numpy as np
from huggingface_hub import create_repo, HfApi
from lerobot.datasets.lerobot_dataset import LeRobotDataset

logger = logging.getLogger(__name__)


def build_features(camera_configs: dict) -> dict:
    """Build LeRobot dataset feature definitions from camera config.

    Returns:
        Feature dict compatible with LeRobotDataset.create()
    """
    features = {}

    # Observation: joint states (16 = left 7 + right 7 + gripper 2)
    features["observation.state"] = {
        "dtype": "float32",
        "shape": (16,),
        "names": [
            "left_joint1.pos", "left_joint2.pos", "left_joint3.pos",
            "left_joint4.pos", "left_joint5.pos", "left_joint6.pos",
            "left_joint7.pos",
            "right_joint1.pos", "right_joint2.pos", "right_joint3.pos",
            "right_joint4.pos", "right_joint5.pos", "right_joint6.pos",
            "right_joint7.pos",
            "left_gripper.pos", "right_gripper.pos",
        ],
    }
    features["observation.velocity"] = {
        "dtype": "float32",
        "shape": (16,),
        "names": [
            "left_joint1.vel", "left_joint2.vel", "left_joint3.vel",
            "left_joint4.vel", "left_joint5.vel", "left_joint6.vel",
            "left_joint7.vel",
            "right_joint1.vel", "right_joint2.vel", "right_joint3.vel",
            "right_joint4.vel", "right_joint5.vel", "right_joint6.vel",
            "right_joint7.vel",
            "left_gripper.vel", "right_gripper.vel",
        ],
    }
    features["observation.effort"] = {
        "dtype": "float32",
        "shape": (16,),
        "names": [
            "left_joint1.tau", "left_joint2.tau", "left_joint3.tau",
            "left_joint4.tau", "left_joint5.tau", "left_joint6.tau",
            "left_joint7.tau",
            "right_joint1.tau", "right_joint2.tau", "right_joint3.tau",
            "right_joint4.tau", "right_joint5.tau", "right_joint6.tau",
            "right_joint7.tau",
            "left_gripper.tau", "right_gripper.tau",
        ],
    }

    # Observation: cameras
    for cam_name, cam_cfg in camera_configs.items():
        h = cam_cfg["height"]
        w = cam_cfg["width"]

        # RGB
        features[f"observation.images.{cam_name}"] = {
            "dtype": "video",
            "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        }

        # Depth (RealSense only)
        if cam_cfg.get("use_depth", False):
            features[f"observation.images.{cam_name}_depth"] = {
                "dtype": "image",
                "shape": (h, w, 3),
                "names": ["height", "width", "channels"],
            }

    # Action: twist + gripper
    features["action.left_twist"] = {
        "dtype": "float32",
        "shape": (6,),
        "names": ["vx", "vy", "vz", "wx", "wy", "wz"],
    }
    features["action.right_twist"] = {
        "dtype": "float32",
        "shape": (6,),
        "names": ["vx", "vy", "vz", "wx", "wy", "wz"],
    }
    features["action.left_gripper"] = {
        "dtype": "float32",
        "shape": (1,),
        "names": ["gripper"],
    }
    features["action.right_gripper"] = {
        "dtype": "float32",
        "shape": (1,),
        "names": ["gripper"],
    }

    return features


def create_dataset(dataset_config: dict, camera_configs: dict, resume: bool) -> LeRobotDataset:
    """Create or resume a LeRobotDataset.

    Args:
        dataset_config: Dataset section from YAML config.
        camera_configs: Cameras section from YAML config.
        resume: Whether to resume an existing dataset.

    Returns:
        LeRobotDataset instance.
    """
    repo_id = dataset_config["repo_id"]
    root = dataset_config.get("root")
    if root:
        root = Path(root).expanduser()

    # HuggingFace repo 없으면 자동 생성
    if dataset_config.get("push_to_hub", False):
        try:
            create_repo(repo_id, repo_type="dataset", exist_ok=True)
            logger.info(f"HuggingFace repo ready: {repo_id}")
        except Exception as e:
            logger.warning(f"Could not create HF repo: {e}")

    if resume:
        dataset = LeRobotDataset(
            repo_id,
            root=root,
        )
    else:
        features = build_features(camera_configs)
        dataset = LeRobotDataset.create(
            repo_id,
            fps=dataset_config["fps"],
            root=root,
            robot_type="openarm_bimanual",
            features=features,
            use_videos=dataset_config.get("video", True),
        )

    return dataset
