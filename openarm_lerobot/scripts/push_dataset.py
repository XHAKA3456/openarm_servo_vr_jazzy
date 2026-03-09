"""Push a local LeRobot dataset to HuggingFace Hub."""

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from pathlib import Path


def main():

    repo_id = "xhaka3456/openarm_test2"
    root = Path("~/openarm/src/openarm_lerobot/datasets/openarm_test").expanduser()

    ds = LeRobotDataset(repo_id, root=root)
    ds.push_to_hub()
    print(f"Done! https://huggingface.co/datasets/{repo_id}")


if __name__ == "__main__":
    main()
