"""
Main recording loop for OpenArm data collection.
Handles episode management, 30Hz loop, keyboard/Quest events, and Rerun visualization.
"""

import logging
import time

import rclpy
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.robot_utils import precise_sleep

from .cameras import CameraManager
from .ros2_subscribers import ROS2Subscribers

logger = logging.getLogger(__name__)


def init_events() -> dict:
    """Initialize event flags for episode control."""
    return {
        "exit_early": False,
        "rerecord_episode": False,
        "stop_recording": False,
    }


def init_keyboard_listener(events: dict):
    """Start keyboard listener for episode control (stdin-based, works over SSH).

    Keys:
        n: save current episode & next
        r: rerecord current episode
        q: stop all recording
    """
    import sys
    import tty
    import termios
    import threading
    import select

    fd = sys.stdin.fileno()
    try:
        old_settings = termios.tcgetattr(fd)
    except termios.error:
        logger.warning("Cannot configure terminal. Keyboard control disabled.")
        return None

    tty.setcbreak(fd)

    def reader_loop():
        try:
            while not events["stop_recording"]:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    if ch == '\x1b':
                        # Drain remaining escape sequence chars
                        while select.select([sys.stdin], [], [], 0.05)[0]:
                            sys.stdin.read(1)
                        # Ignore arrow keys / escape — use n, r, q instead
                        continue
                    elif ch == 'n':
                        logger.info("'n' pressed. Saving episode & next...")
                        events["exit_early"] = True
                    elif ch == 'r':
                        logger.info("'r' pressed. Rerecording episode...")
                        events["rerecord_episode"] = True
                        events["exit_early"] = True
                    elif ch == 'q':
                        logger.info("'q' pressed. Stopping recording...")
                        events["stop_recording"] = True
                        events["exit_early"] = True
        except Exception as e:
            logger.error(f"Keyboard reader error: {e}")

    thread = threading.Thread(target=reader_loop, daemon=True)
    thread.start()

    class StdinListener:
        def __init__(self, fd, old_settings):
            self._fd = fd
            self._old = old_settings
        def stop(self):
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    return StdinListener(fd, old_settings)


def record_episode(
    subscribers: ROS2Subscribers,
    cameras: CameraManager,
    dataset: LeRobotDataset,
    events: dict,
    fps: int,
    control_time_s: float,
    single_task: str,
    display_data: bool = False,
):
    """Record a single episode at the specified fps.

    Args:
        subscribers: ROS 2 subscriber node.
        cameras: Camera manager.
        dataset: LeRobot dataset to write frames to.
        events: Event flags dict.
        fps: Target frames per second.
        control_time_s: Max duration for this episode in seconds.
        single_task: Task description string.
        display_data: Whether to log to Rerun.
    """
    if display_data:
        from lerobot.utils.visualization_utils import log_rerun_data

    timestamp = 0
    start_t = time.perf_counter()
    QUEST_BUTTON_GRACE_S = 0.5  # Ignore Quest buttons for first 0.5s of episode

    while timestamp < control_time_s:
        loop_start = time.perf_counter()

        if events["exit_early"]:
            events["exit_early"] = False
            break

        # Check Quest joystick: next episode (ignore during grace period)
        if timestamp >= QUEST_BUTTON_GRACE_S and subscribers.get_quest_x_button():
            logger.info("Quest: next episode triggered")
            break

        # Check Quest joystick: rerecord episode (ignore during grace period)
        if timestamp >= QUEST_BUTTON_GRACE_S and subscribers.get_quest_rerecord():
            logger.info("Quest: rerecord triggered")
            events["rerecord_episode"] = True
            events["exit_early"] = True
            break

        # Spin ROS 2 to drain all pending messages
        for _ in range(10):
            rclpy.spin_once(subscribers, timeout_sec=0)

        # During grace period: drain Quest button signals so stale ones don't carry over
        if timestamp < QUEST_BUTTON_GRACE_S:
            subscribers.get_quest_x_button()
            subscribers.get_quest_rerecord()

        # Get observation and action
        obs = subscribers.get_observation()
        action = subscribers.get_action()

        # Get camera frames
        cam_frames = cameras.read_all()
        for cam_key, frame in cam_frames.items():
            obs[f"images.{cam_key}"] = frame

        # Build frame for dataset
        frame = {"task": single_task}
        for k, v in obs.items():
            frame[f"observation.{k}"] = v
        for k, v in action.items():
            frame[f"action.{k}" if k else "action"] = v

        dataset.add_frame(frame)

        # Rerun visualization
        if display_data:
            log_rerun_data(observation=obs, action=action)

        # Maintain target fps
        dt = time.perf_counter() - loop_start
        precise_sleep(max(1.0 / fps - dt, 0.0))

        timestamp = time.perf_counter() - start_t


def run_recording(
    subscribers: ROS2Subscribers,
    cameras: CameraManager,
    dataset: LeRobotDataset,
    config: dict,
):
    """Run the full recording session with multiple episodes.

    Args:
        subscribers: ROS 2 subscriber node.
        cameras: Camera manager.
        dataset: LeRobot dataset.
        config: Full YAML config dict.
    """
    dataset_cfg = config["dataset"]
    fps = dataset_cfg["fps"]
    episode_time_s = dataset_cfg["episode_time_s"]
    reset_time_s = dataset_cfg["reset_time_s"]
    num_episodes = dataset_cfg["num_episodes"]
    single_task = dataset_cfg["single_task"]
    display_data = config.get("display_data", False)

    # Init Rerun
    if display_data:
        from lerobot.utils.visualization_utils import init_rerun
        init_rerun(session_name="openarm_recording")

    # Init events and keyboard listener
    events = init_events()
    listener = init_keyboard_listener(events)

    try:
        recorded = 0
        while recorded < num_episodes and not events["stop_recording"]:
            logger.info(f"Recording episode {dataset.num_episodes} ({recorded + 1}/{num_episodes})")

            # Record episode
            record_episode(
                subscribers=subscribers,
                cameras=cameras,
                dataset=dataset,
                events=events,
                fps=fps,
                control_time_s=episode_time_s,
                single_task=single_task,
                display_data=display_data,
            )

            # Handle rerecord
            if events["rerecord_episode"]:
                logger.info("Rerecording episode...")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            # Save episode
            dataset.save_episode()
            recorded += 1
            logger.info(f"Episode saved. Total: {recorded}/{num_episodes}")

            # Reset period (skip for last episode or if stopping)
            if not events["stop_recording"] and recorded < num_episodes:
                logger.info(f"Reset environment. Waiting {reset_time_s}s...")
                reset_start = time.perf_counter()
                while time.perf_counter() - reset_start < reset_time_s:
                    if events["exit_early"] or events["stop_recording"]:
                        events["exit_early"] = False
                        break
                    # Check Quest X button during reset
                    rclpy.spin_once(subscribers, timeout_sec=0)
                    if subscribers.get_quest_x_button():
                        break
                    time.sleep(0.1)

    finally:
        logger.info("Finalizing dataset...")
        dataset.finalize()

        if config["dataset"].get("push_to_hub", False):
            logger.info("Pushing to HuggingFace Hub...")
            dataset.push_to_hub()

        if listener is not None:
            listener.stop()

        logger.info("Recording complete.")
