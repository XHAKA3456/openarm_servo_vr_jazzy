from setuptools import setup, find_packages

setup(
    name="openarm-lerobot",
    version="0.1.0",
    description="OpenArm + LeRobot data collection and inference",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=[
        "numpy",
        "pyyaml",
        "lerobot",
        "pynput",
    ],
    extras_require={
        "realsense": ["pyrealsense2"],
    },
    entry_points={
        "console_scripts": [
            "openarm-collect=openarm_lerobot.collect_data:main",
            "openarm-infer=openarm_lerobot.inference:main",
        ],
    },
)
