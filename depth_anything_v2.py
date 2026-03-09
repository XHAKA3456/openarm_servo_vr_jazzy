#!/usr/bin/env python3
"""
Depth Anything V2 - Video Depth Colormap Extractor
Usage: python depth_anything_v2.py <video_path> [--output output.mp4] [--model small|base|large]
"""

import argparse
import os
import sys
import numpy as np
import cv2
from pathlib import Path

def load_model(model_size="small"):
    """Load Depth Anything V2 via HuggingFace transformers"""
    try:
        from transformers import pipeline
        model_id = {
            "small": "depth-anything/Depth-Anything-V2-Small-hf",
            "base":  "depth-anything/Depth-Anything-V2-Base-hf",
            "large": "depth-anything/Depth-Anything-V2-Large-hf",
        }[model_size]
        print(f"[*] Loading model: {model_id}")
        pipe = pipeline(task="depth-estimation", model=model_id)
        return pipe, "transformers"
    except ImportError:
        print("[!] transformers not found, trying torch hub...")

    try:
        import torch
        from torchvision.transforms import Compose
        # fallback: try loading from local depth_anything_v2 repo if cloned
        sys.path.insert(0, str(Path(__file__).parent / "Depth-Anything-V2"))
        from depth_anything_v2.dpt import DepthAnythingV2
        configs = {
            "small": {"encoder": "vits", "features": 64,  "out_channels": [48, 96, 192, 384]},
            "base":  {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768]},
            "large": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
        }
        model = DepthAnythingV2(**configs[model_size])
        ckpt_path = Path(__file__).parent / f"depth_anything_v2_vit{model_size[0]}.pth"
        if not ckpt_path.exists():
            print(f"[!] Checkpoint not found: {ckpt_path}")
            print("    Download from: https://huggingface.co/depth-anything/Depth-Anything-V2-Small")
            sys.exit(1)
        device = "cuda" if torch.cuda.is_available() else "cpu"
        model.load_state_dict(torch.load(ckpt_path, map_location=device))
        model = model.to(device).eval()
        return model, "torch"
    except (ImportError, ModuleNotFoundError) as e:
        print(f"[!] Error: {e}")
        print("\nInstall with:\n  pip install transformers torch Pillow")
        sys.exit(1)


def process_frame_transformers(pipe, frame_bgr):
    from PIL import Image
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(frame_rgb)
    result = pipe(pil_img)
    depth = np.array(result["depth"])
    return depth


def process_frame_torch(model, frame_bgr):
    import torch
    device = next(model.parameters()).device
    depth = model.infer_image(frame_bgr)  # returns HxW numpy array
    return depth


def depth_to_colormap(depth, colormap=cv2.COLORMAP_INFERNO):
    """Normalize depth and apply colormap"""
    depth_min, depth_max = depth.min(), depth.max()
    if depth_max - depth_min > 1e-6:
        normalized = (depth - depth_min) / (depth_max - depth_min)
    else:
        normalized = np.zeros_like(depth)
    depth_uint8 = (normalized * 255).astype(np.uint8)
    colored = cv2.applyColorMap(depth_uint8, colormap)
    return colored


def process_video(video_path, output_path, model_size="small", side_by_side=True, max_frames=None):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[!] Cannot open video: {video_path}")
        sys.exit(1)

    fps    = cap.get(cv2.CAP_PROP_FPS)
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    out_width = width * 2 if side_by_side else width
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_path, fourcc, fps, (out_width, height))

    if max_frames is not None:
        total = min(total, max_frames)

    model, backend = load_model(model_size)
    print(f"[*] Video: {width}x{height} @ {fps:.1f}fps, {total} frames")
    print(f"[*] Output: {output_path}")
    print(f"[*] Backend: {backend}")

    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if backend == "transformers":
            depth = process_frame_transformers(model, frame)
        else:
            depth = process_frame_torch(model, frame)

        colored = depth_to_colormap(depth)

        # Resize colormap to match original frame size
        colored = cv2.resize(colored, (width, height))

        if side_by_side:
            combined = np.hstack([frame, colored])
        else:
            combined = colored

        out.write(combined)
        frame_idx += 1
        if max_frames is not None and frame_idx >= max_frames:
            break
        if frame_idx % 10 == 0 or frame_idx == total:
            print(f"\r[*] Progress: {frame_idx}/{total} frames", end="", flush=True)

    print()
    cap.release()
    out.release()
    print(f"[+] Done! Saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Depth Anything V2 - Video Depth Colormap")
    parser.add_argument("video", help="Input video path")
    parser.add_argument("--output", "-o", default=None, help="Output video path")
    parser.add_argument("--model", "-m", default="small", choices=["small", "base", "large"],
                        help="Model size (default: small)")
    parser.add_argument("--depth-only", action="store_true",
                        help="Output depth colormap only (no side-by-side)")
    parser.add_argument("--frames", "-f", default=None, type=int,
                        help="Max number of frames to process (e.g. --frames 310)")
    parser.add_argument("--colormap", default="inferno",
                        choices=["inferno", "magma", "plasma", "viridis", "turbo"],
                        help="Colormap style (default: inferno)")
    args = parser.parse_args()

    video_path = args.video
    if not os.path.exists(video_path):
        print(f"[!] File not found: {video_path}")
        sys.exit(1)

    if args.output is None:
        stem = Path(video_path).stem
        args.output = str(Path(video_path).parent / f"{stem}_depth.mp4")

    colormap_map = {
        "inferno": cv2.COLORMAP_INFERNO,
        "magma":   cv2.COLORMAP_MAGMA,
        "plasma":  cv2.COLORMAP_PLASMA,
        "viridis": cv2.COLORMAP_VIRIDIS,
        "turbo":   cv2.COLORMAP_TURBO,
    }
    # monkey-patch colormap choice into depth_to_colormap
    global _colormap
    _colormap = colormap_map[args.colormap]
    original_depth_to_colormap = depth_to_colormap

    def depth_to_colormap_patched(depth, colormap=_colormap):
        return original_depth_to_colormap(depth, colormap=_colormap)

    import builtins
    globals()["depth_to_colormap"] = depth_to_colormap_patched

    process_video(
        video_path=video_path,
        output_path=args.output,
        model_size=args.model,
        side_by_side=not args.depth_only,
        max_frames=args.frames,
    )


if __name__ == "__main__":
    main()
