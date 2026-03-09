#!/usr/bin/env python3
"""
LeRobot 데이터셋에 Depth Anything V2 depth colormap 비디오를 추가합니다.

- 원본 비디오와 동일한 fps/프레임 수 보장 (불일치 시 에러)
- h264/yuv420p 인코딩 (LeRobot 호환)
- meta/info.json 자동 업데이트

Usage:
  python lerobot_add_depth.py <dataset_path> --camera right_wrist
  python lerobot_add_depth.py <dataset_path> --camera right_wrist left_wrist
  python lerobot_add_depth.py <dataset_path> --camera right_wrist --model small
"""

import argparse
import json
import subprocess
import sys
import numpy as np
import cv2
from pathlib import Path


# ── Depth Anything V2 ────────────────────────────────────────────────────────

def load_model(model_size="small"):
    from transformers import pipeline
    model_id = {
        "small": "depth-anything/Depth-Anything-V2-Small-hf",
        "base":  "depth-anything/Depth-Anything-V2-Base-hf",
        "large": "depth-anything/Depth-Anything-V2-Large-hf",
    }[model_size]
    print(f"[*] Loading model: {model_id}")
    pipe = pipeline(task="depth-estimation", model=model_id)
    return pipe


def infer_depth(pipe, frame_bgr):
    from PIL import Image
    pil = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    depth = np.array(pipe(pil)["depth"])
    d_min, d_max = depth.min(), depth.max()
    if d_max - d_min > 1e-6:
        depth = (depth - d_min) / (d_max - d_min)
    return (depth * 255).astype(np.uint8)


def depth_to_colormap(depth_u8, cmap=cv2.COLORMAP_INFERNO):
    return cv2.applyColorMap(depth_u8, cmap)


# ── FFmpeg writer (h264/yuv420p — LeRobot 호환) ───────────────────────────────

def open_ffmpeg_writer(output_path: Path, width: int, height: int, fps: float):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg", "-y",
        "-f", "rawvideo", "-vcodec", "rawvideo",
        "-pix_fmt", "bgr24",
        "-s", f"{width}x{height}",
        "-r", str(fps),
        "-i", "pipe:0",
        "-vcodec", "libx264",
        "-pix_fmt", "yuv420p",
        "-crf", "18",
        str(output_path),
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)


# ── 프레임 수 검증 ─────────────────────────────────────────────────────────────

def count_frames(video_path: Path) -> int:
    cap = cv2.VideoCapture(str(video_path))
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    return count


def verify_frame_count(src_path: Path, dst_path: Path):
    src_count = count_frames(src_path)
    dst_count = count_frames(dst_path)
    if src_count != dst_count:
        raise RuntimeError(
            f"Frame count mismatch! "
            f"원본: {src_count} frames, depth: {dst_count} frames\n"
            f"  src: {src_path}\n"
            f"  dst: {dst_path}"
        )
    print(f"  [✓] Frame count verified: {src_count} frames")


# ── 비디오 처리 ───────────────────────────────────────────────────────────────

def process_video(pipe, src_path: Path, dst_path: Path, colormap):
    cap = cv2.VideoCapture(str(src_path))
    fps    = cap.get(cv2.CAP_PROP_FPS)
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    proc = open_ffmpeg_writer(dst_path, width, height, fps)
    written = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        depth_u8 = infer_depth(pipe, frame)
        colored  = cv2.resize(depth_to_colormap(depth_u8, colormap), (width, height))
        proc.stdin.write(colored.tobytes())
        written += 1
        print(f"\r  {written}/{total} frames", end="", flush=True)

    print()
    cap.release()
    proc.stdin.close()
    proc.wait()

    # 프레임 수 검증 — 불일치 시 즉시 에러
    verify_frame_count(src_path, dst_path)
    return written


# ── info.json 업데이트 ────────────────────────────────────────────────────────

def update_info_json(info_path: Path, src_key: str, dst_key: str):
    with open(info_path) as f:
        info = json.load(f)

    if dst_key in info["features"]:
        print(f"  [*] {dst_key} already in info.json, skipping")
        return

    src_feature = json.loads(json.dumps(info["features"][src_key]))  # deep copy
    src_feature["info"]["video.is_depth_map"] = True
    info["features"][dst_key] = src_feature

    with open(info_path, "w") as f:
        json.dump(info, f, indent=4)
    print(f"  [✓] info.json updated: added {dst_key}")


# ── CLI ───────────────────────────────────────────────────────────────────────

COLORMAP_MAP = {
    "inferno": cv2.COLORMAP_INFERNO,
    "magma":   cv2.COLORMAP_MAGMA,
    "plasma":  cv2.COLORMAP_PLASMA,
    "viridis": cv2.COLORMAP_VIRIDIS,
    "turbo":   cv2.COLORMAP_TURBO,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", help="LeRobot dataset root path")
    parser.add_argument("--camera", "-c", nargs="+", required=True,
                        help="카메라 키 (e.g. right_wrist left_wrist)")
    parser.add_argument("--model", "-m", default="small",
                        choices=["small", "base", "large"])
    parser.add_argument("--colormap", default="inferno",
                        choices=list(COLORMAP_MAP.keys()))
    args = parser.parse_args()

    dataset_path = Path(args.dataset)
    info_path    = dataset_path / "meta" / "info.json"
    cmap         = COLORMAP_MAP[args.colormap]

    pipe = load_model(args.model)

    for camera in args.camera:
        src_key = f"observation.images.{camera}"
        dst_key = f"observation.images.{camera}_depth"
        src_dir = dataset_path / "videos" / src_key
        dst_dir = dataset_path / "videos" / dst_key

        if not src_dir.exists():
            print(f"[!] Not found: {src_dir}, skipping")
            continue

        src_videos = sorted(src_dir.rglob("*.mp4"))
        print(f"\n[*] {src_key} → {dst_key} ({len(src_videos)} video(s))")

        for src_path in src_videos:
            rel      = src_path.relative_to(src_dir)
            dst_path = dst_dir / rel
            print(f"  Processing: {rel}")
            process_video(pipe, src_path, dst_path, cmap)

        update_info_json(info_path, src_key, dst_key)

    print("\n[+] Done!")


if __name__ == "__main__":
    main()
