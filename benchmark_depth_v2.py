#!/usr/bin/env python3
"""
Depth Anything V2 단일 프레임 추론 시간 측정.

Usage:
    python benchmark_depth_v2.py                     # small, cuda, 카메라 0
    python benchmark_depth_v2.py --model base        # base 모델
    python benchmark_depth_v2.py --device cpu        # CPU
    python benchmark_depth_v2.py --image test.jpg    # 이미지 파일로 테스트
    python benchmark_depth_v2.py --camera 2          # 카메라 index 지정
"""

import argparse
import time
import numpy as np
import cv2


def load_model(model_size: str, device: str):
    from transformers import pipeline
    import torch

    if device == "cuda" and not torch.cuda.is_available():
        print("[!] CUDA 없음. CPU로 fallback.")
        device = "cpu"

    model_id = {
        "small": "depth-anything/Depth-Anything-V2-Small-hf",
        "base":  "depth-anything/Depth-Anything-V2-Base-hf",
        "large": "depth-anything/Depth-Anything-V2-Large-hf",
    }[model_size]

    print(f"[*] 모델 로드 중: {model_id} on {device}")
    t0 = time.perf_counter()
    pipe = pipeline(task="depth-estimation", model=model_id, device=device)
    print(f"[*] 모델 로드 완료: {time.perf_counter() - t0:.1f}s")
    return pipe, device


def infer_once(pipe, frame_bgr: np.ndarray) -> float:
    from PIL import Image
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(frame_rgb)
    t0 = time.perf_counter()
    pipe(pil_img)
    return time.perf_counter() - t0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model",  default="small", choices=["small", "base", "large"])
    parser.add_argument("--device", default="cuda",  choices=["cuda", "cpu"])
    parser.add_argument("--camera", default=0, type=int, help="OpenCV 카메라 index")
    parser.add_argument("--image",  default=None,    help="카메라 대신 이미지 파일 사용")
    parser.add_argument("--width",  default=640, type=int)
    parser.add_argument("--height", default=480, type=int)
    parser.add_argument("--runs",   default=20,  type=int, help="반복 횟수")
    args = parser.parse_args()

    # 프레임 준비
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"[!] 이미지 로드 실패: {args.image}")
            return
        frame = cv2.resize(frame, (args.width, args.height))
        print(f"[*] 이미지: {args.image} ({args.width}x{args.height})")
    else:
        cap = cv2.VideoCapture(args.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        ret, frame = cap.read()
        cap.release()
        if not ret:
            print(f"[!] 카메라 {args.camera} 읽기 실패. --image 옵션으로 이미지 파일을 사용하세요.")
            return
        print(f"[*] 카메라 index={args.camera} ({args.width}x{args.height})")

    pipe, device = load_model(args.model, args.device)

    # 웜업
    print("[*] 웜업 중 (3회)...")
    for _ in range(3):
        infer_once(pipe, frame)

    # 벤치마크
    print(f"[*] 벤치마크 {args.runs}회...")
    times = []
    for i in range(args.runs):
        dt = infer_once(pipe, frame)
        times.append(dt * 1000)
        print(f"    [{i+1:2d}/{args.runs}] {dt*1000:.1f} ms")

    times = np.array(times)
    print()
    print("=" * 40)
    print(f"  모델:   {args.model} / {device}")
    print(f"  해상도: {args.width}x{args.height}")
    print(f"  평균:   {times.mean():.1f} ms")
    print(f"  최소:   {times.min():.1f} ms")
    print(f"  최대:   {times.max():.1f} ms")
    print(f"  std:    {times.std():.1f} ms")
    print(f"  → 최대 가능 fps: {1000/times.mean():.1f} fps")
    print(f"  → 30fps 대비 lag: ~{times.mean()/33.3:.1f} 프레임")
    print("=" * 40)


if __name__ == "__main__":
    main()
