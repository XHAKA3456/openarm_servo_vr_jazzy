"""
OpenArm inference with trained LeRobot policy.

Loads a trained policy (HuggingFace Hub or local path),
subscribes to ROS 2 observation topics, and publishes
joint trajectory commands at the training fps.

Usage:
    python3 -m openarm_lerobot.inference --config config/inference.yaml
"""

import logging
import threading
import time

import numpy as np
import rclpy
import torch
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .cameras import CameraManager
from .ros2_subscribers import ROS2Subscribers

logger = logging.getLogger(__name__)

LEFT_JOINTS = [
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7",
]
RIGHT_JOINTS = [
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7",
]


class InferenceNode(Node):
    """ROS 2 node that publishes policy actions to trajectory controllers."""

    def __init__(self):
        super().__init__("openarm_inference")

        self._left_traj_pub = self.create_publisher(
            JointTrajectory,
            "/left_joint_trajectory_controller/joint_trajectory",
            10,
        )
        self._right_traj_pub = self.create_publisher(
            JointTrajectory,
            "/right_joint_trajectory_controller/joint_trajectory",
            10,
        )
        self._left_gripper_client = ActionClient(
            self, GripperCommand, "/left_gripper_controller/gripper_cmd"
        )
        self._right_gripper_client = ActionClient(
            self, GripperCommand, "/right_gripper_controller/gripper_cmd"
        )
        self._prev_left_gripper = -1.0
        self._prev_right_gripper = -1.0

    def send_trajectory_chunk(self, action_chunk: list, step_period: float, infer_dt: float = 0.0):
        """Send a full chunk of N actions as a single multi-waypoint JointTrajectory.

        JTC receives all N waypoints at once and executes them smoothly in one go,
        without interruption. This matches how MoveIt Servo works (continuous motion).

        Args:
            action_chunk: list of (16,) np.ndarray  [L_joint1~7, R_joint1~7, L_finger, R_finger]
            step_period:  time per waypoint in seconds (= 1/fps)
            infer_dt:     inference duration in seconds — used to give the first waypoint
                          enough time so the robot doesn't jerk after the inference pause.
        """
        now = self.get_clock().now().to_msg()

        left_traj = JointTrajectory()
        left_traj.header.stamp = now
        left_traj.joint_names = LEFT_JOINTS

        right_traj = JointTrajectory()
        right_traj.header.stamp = now
        right_traj.joint_names = RIGHT_JOINTS

        # Shift all waypoints by infer_dt so the first waypoint has enough
        # travel time after the inference pause (robot was holding position).
        time_offset = infer_dt

        for i, jp in enumerate(action_chunk):
            t = time_offset + (i + 1) * step_period
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            dur = Duration(sec=sec, nanosec=nanosec)

            pt_l = JointTrajectoryPoint()
            pt_l.positions = jp[:7].tolist()
            pt_l.time_from_start = dur
            left_traj.points.append(pt_l)

            pt_r = JointTrajectoryPoint()
            pt_r.positions = jp[7:14].tolist()
            pt_r.time_from_start = dur
            right_traj.points.append(pt_r)

        self._left_traj_pub.publish(left_traj)
        self._right_traj_pub.publish(right_traj)

        # Gripper: scan ALL waypoints for snap transitions and fire at the right time.
        # Training data is binary (0 or 0.0264m, motor moves in <33ms), so we snap
        # predicted values and schedule each open/close command at its actual waypoint time.
        GRIPPER_OPEN = 0.0264
        THRESH = GRIPPER_OPEN / 2  # 0.0132m

        schedule = []  # list of (delay_sec, arm, position)
        prev_l, prev_r = self._prev_left_gripper, self._prev_right_gripper

        for i, jp in enumerate(action_chunk):
            t = time_offset + (i + 1) * step_period
            left  = GRIPPER_OPEN if jp[14] >= THRESH else 0.0
            right = GRIPPER_OPEN if jp[15] >= THRESH else 0.0
            if abs(left  - prev_l) > 0.001:
                schedule.append((t, "left",  left));  prev_l = left
            if abs(right - prev_r) > 0.001:
                schedule.append((t, "right", right)); prev_r = right

        self._prev_left_gripper  = prev_l
        self._prev_right_gripper = prev_r

        if schedule:
            t0 = time.perf_counter()
            def _run(sched=schedule, start=t0):
                for delay, arm, pos in sched:
                    wait = delay - (time.perf_counter() - start)
                    if wait > 0:
                        time.sleep(wait)
                    self._send_gripper(arm, pos)
            threading.Thread(target=_run, daemon=True).start()

        return schedule  # for logging

    def _send_gripper(self, arm: str, position: float):
        # Action is already finger_joint1 position in metres (same as observation.state[14:15]).
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0
        client = self._left_gripper_client if arm == "left" else self._right_gripper_client
        if client.server_is_ready():
            client.send_goal_async(goal)
        else:
            self.get_logger().warning(f"Gripper action server not ready ({arm}), skipping position={position:.3f}")


def _load_policy(policy_cfg: dict):
    """Load policy + pre/post processors directly from checkpoint (no dataset needed)."""
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors

    path = policy_cfg["path"]
    device_str = policy_cfg.get("device", "cuda")
    device = torch.device(device_str if torch.cuda.is_available() else "cpu")

    logger.info(f"Loading policy from: {path}")
    pretrained_cfg = PreTrainedConfig.from_pretrained(path)
    pretrained_cfg.pretrained_path = path
    pretrained_cfg.device = str(device)

    policy_cls = get_policy_class(pretrained_cfg.type)
    policy = policy_cls.from_pretrained(path, config=pretrained_cfg)
    policy = policy.to(device)
    policy.eval()

    device_override = {"device_processor": {"device": str(device)}}
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=pretrained_cfg,
        pretrained_path=path,
        preprocessor_overrides=device_override,
        postprocessor_overrides=device_override,
    )

    return policy, preprocessor, postprocessor, device


def run_inference(config: dict):
    """Main inference loop."""
    policy_cfg = config["policy"]
    inf_cfg = config["inference"]
    topic_cfg = config["topics"]
    cam_cfg = config.get("cameras", {})

    fps = inf_cfg["fps"]
    single_task = inf_cfg.get("single_task", None)

    # Load policy directly from checkpoint (features + stats embedded in checkpoint)
    policy, preprocessor, postprocessor, device = _load_policy(policy_cfg)

    # Init ROS 2
    rclpy.init()
    subscribers = ROS2Subscribers(topic_cfg)
    inference_node = InferenceNode()

    # Init cameras
    cameras = None
    if cam_cfg:
        cameras = CameraManager(cam_cfg)
        cameras.connect_all()

    chunk_size = policy.config.n_action_steps
    step_period = 1.0 / fps   # 0.033s per waypoint

    policy.reset()
    preprocessor.reset()
    postprocessor.reset()

    logger.info(
        f"Inference | fps={fps} | chunk_size={chunk_size} | "
        f"chunk_duration={chunk_size * step_period:.2f}s. Ctrl+C to stop."
    )

    try:
        from lerobot.utils.control_utils import predict_action

        chunk_idx = 0
        while rclpy.ok():
            # ── 1. Get observation ──────────────────────────────────────
            rclpy.spin_once(subscribers, timeout_sec=0)
            rclpy.spin_once(inference_node, timeout_sec=0)

            obs_raw = subscribers.get_observation()
            obs = {f"observation.{k}": v for k, v in obs_raw.items()}
            if cameras:
                for cam_key, frame in cameras.read_all().items():
                    obs[f"observation.images.{cam_key}"] = frame

            # ── 2. Generate full chunk (triggers one forward pass) ──────
            infer_start = time.perf_counter()
            policy.reset()  # clear queue → force fresh chunk on first call
            action_chunk = []
            for _ in range(chunk_size):
                action = predict_action(
                    observation=obs,
                    policy=policy,
                    device=device,
                    preprocessor=preprocessor,
                    postprocessor=postprocessor,
                    use_amp=getattr(policy.config, "use_amp", False),
                    task=single_task,
                )
                # Extract (16,) joint positions
                if isinstance(action, dict):
                    jp = action.get("", action.get("joint_positions"))
                    if jp is None:
                        jp = next(iter(action.values()))
                else:
                    jp = action
                if isinstance(jp, torch.Tensor):
                    jp = jp.detach().cpu().numpy()
                jp = np.array(jp, dtype=np.float32).flatten()
                if len(jp) == 16:
                    action_chunk.append(jp)
            infer_dt = time.perf_counter() - infer_start

            if not action_chunk:
                logger.warning("Empty action chunk, skipping.")
                continue

            # ── 3. Send all waypoints as one smooth trajectory ──────────
            gripper_schedule = inference_node.send_trajectory_chunk(action_chunk, step_period, infer_dt=infer_dt)

            chunk_idx += 1
            gripper_log = (
                ", ".join(f"{arm}={'OPEN' if pos>0 else 'CLOSE'}@{delay:.2f}s"
                          for delay, arm, pos in gripper_schedule)
                if gripper_schedule else "no change"
            )
            logger.info(
                f"[chunk={chunk_idx:4d}] infer={infer_dt*1000:.0f}ms  "
                f"waypoints={len(action_chunk)}  exec={len(action_chunk)*step_period:.2f}s\n"
                f"  first(L): [{' '.join(f'{v:+.3f}' for v in action_chunk[0][:7])}]\n"
                f"  last(L):  [{' '.join(f'{v:+.3f}' for v in action_chunk[-1][:7])}]\n"
                f"  gripper:  {gripper_log}\n"
                f"  obs:      [{' '.join(f'{v:+.3f}' for v in obs_raw['state'][:7])}] "
                f"grip=[L:{obs_raw['state'][14]:+.4f} R:{obs_raw['state'][15]:+.4f}]"
            )

            # ── 4. Wait for trajectory to finish, then re-observe ───────
            time.sleep(infer_dt + len(action_chunk) * step_period)

    except KeyboardInterrupt:
        logger.info("Stopped by user.")
    finally:
        if cameras:
            cameras.disconnect_all()
        subscribers.destroy_node()
        inference_node.destroy_node()
        rclpy.shutdown()


def main():
    import argparse

    parser = argparse.ArgumentParser(description="OpenArm inference with trained LeRobot policy")
    parser.add_argument("--config", type=str, required=True, help="Path to inference config YAML")
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    run_inference(config)


if __name__ == "__main__":
    main()
