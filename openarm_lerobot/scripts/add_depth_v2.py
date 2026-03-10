#!/usr/bin/env python3
"""
LeRobot 데이터셋에 Depth Anything V2 depth를 후가공으로 추가.

안전 원칙:
  - 원본 비디오 절대 수정하지 않음
  - 임시 파일(.tmp.mp4)에 먼저 쓰고, 프레임 수 검증 후 이동
  - 이미 처리된 파일은 건너뜀 (중단 후 재시작 가능)
  - 모든 비디오 처리 성공 후에만 info.json/stats.json 업데이트
  - --dry-run으로 실제 변경 없이 미리 확인 가능

Usage:
    python add_depth_v2.py --dataset ~/openarm/src/openarm_lerobot/datasets/my_dataset
    python add_depth_v2.py --dataset ~/datasets/my_dataset --cameras left_wrist right_wrist
    python add_depth_v2.py --dataset ~/datasets/my_dataset --dry-run
    python add_depth_v2.py --dataset ~/datasets/my_dataset --model base --device cuda
"""

import argparse
import json
import logging
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


# ──────────────────────────────────────────────
# Depth Anything V2
# ──────────────────────────────────────────────

def load_model(model_size: str, device: str):
    import torch
    from transformers import pipeline

    if device == "cuda" and not torch.cuda.is_available():
        logger.warning("CUDA 없음. CPU로 fallback.")
        device = "cpu"

    model_id = {
        "small": "depth-anything/Depth-Anything-V2-Small-hf",
        "base":  "depth-anything/Depth-Anything-V2-Base-hf",
        "large": "depth-anything/Depth-Anything-V2-Large-hf",
    }[model_size]

    logger.info(f"모델 로드 중: {model_id} on {device}")
    pipe = pipeline(task="depth-estimation", model=model_id, device=device)
    logger.info("모델 로드 완료")
    return pipe


def infer_depth(pipe, frame_bgr: np.ndarray) -> np.ndarray:
    """BGR 프레임 → depth colormap BGR (원본과 동일 해상도)."""
    from PIL import Image
    h, w = frame_bgr.shape[:2]
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(frame_rgb)
    result = pipe(pil_img)
    depth = np.array(result["depth"], dtype=np.float32)

    dmin, dmax = depth.min(), depth.max()
    norm = (depth - dmin) / (dmax - dmin + 1e-6)
    depth_uint8 = (norm * 255).astype(np.uint8)
    colormap_bgr = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_INFERNO)

    if colormap_bgr.shape[:2] != (h, w):
        colormap_bgr = cv2.resize(colormap_bgr, (w, h))
    return colormap_bgr


# ──────────────────────────────────────────────
# 비디오 처리
# ──────────────────────────────────────────────

def count_frames(video_path: Path) -> int:
    cap = cv2.VideoCapture(str(video_path))
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    # PROP_FRAME_COUNT가 부정확한 경우 직접 세기
    if count <= 0:
        cap = cv2.VideoCapture(str(video_path))
        count = 0
        while True:
            ret, _ = cap.read()
            if not ret:
                break
            count += 1
        cap.release()
    return count


def process_video(
    pipe,
    src_path: Path,
    dst_path: Path,
    dry_run: bool = False,
) -> bool:
    """
    src_path의 각 프레임에 V2 depth를 적용해 dst_path에 저장.
    성공 시 True, 실패 시 False 반환.

    안전 전략:
      1. dst_path.parent 에 임시 파일(.tmp.mp4) 먼저 생성
      2. 프레임 수 검증
      3. 검증 통과 시 dst_path로 이동
    """
    if dry_run:
        src_frames = count_frames(src_path)
        logger.info(f"[DRY-RUN] {src_path.name} → {dst_path.name} ({src_frames} frames)")
        return True

    cap = cv2.VideoCapture(str(src_path))
    if not cap.isOpened():
        logger.error(f"비디오 열기 실패: {src_path}")
        return False

    fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    dst_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = dst_path.with_suffix(".tmp.mp4")

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(str(tmp_path), fourcc, fps, (width, height))
    if not out.isOpened():
        logger.error(f"VideoWriter 열기 실패: {tmp_path}")
        cap.release()
        return False

    written = 0
    t_start = time.perf_counter()
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            depth_bgr = infer_depth(pipe, frame)
            out.write(depth_bgr)
            written += 1
            if written % 30 == 0:
                elapsed = time.perf_counter() - t_start
                fps_actual = written / elapsed
                remaining = (total - written) / fps_actual if fps_actual > 0 else 0
                logger.info(f"  {written}/{total} frames | {fps_actual:.1f} fps | 남은 시간 ~{remaining:.0f}s")
    except Exception as e:
        logger.error(f"처리 중 에러: {e}")
        cap.release()
        out.release()
        tmp_path.unlink(missing_ok=True)
        return False
    finally:
        cap.release()
        out.release()

    # ── 프레임 수 검증 ──────────────────────────
    written_check = count_frames(tmp_path)
    src_count     = count_frames(src_path)

    if written_check != src_count:
        logger.error(
            f"프레임 수 불일치! 원본={src_count}, 생성={written_check}. "
            f"임시 파일 삭제: {tmp_path}"
        )
        tmp_path.unlink(missing_ok=True)
        return False

    # ── 검증 통과 → 최종 위치로 이동 ───────────
    shutil.move(str(tmp_path), str(dst_path))
    elapsed = time.perf_counter() - t_start
    logger.info(f"완료: {dst_path.name} ({written_check} frames, {elapsed:.1f}s)")
    return True


# ──────────────────────────────────────────────
# info.json 업데이트
# ──────────────────────────────────────────────

def update_info_json(dataset_dir: Path, new_camera_keys: list[str], dry_run: bool):
    info_path = dataset_dir / "meta" / "info.json"
    if not info_path.exists():
        logger.warning("info.json 없음. 건너뜀.")
        return

    with open(info_path) as f:
        info = json.load(f)

    features = info.get("features", {})
    added = []

    for key in new_camera_keys:
        feature_key = f"observation.images.{key}"
        if feature_key in features:
            logger.info(f"info.json: '{feature_key}' 이미 존재. 건너뜀.")
            continue

        # 원본 RGB feature에서 shape/dtype 복사
        src_key = feature_key.replace("_depth", "")
        if src_key in features:
            template = dict(features[src_key])
            features[feature_key] = template
            added.append(feature_key)
        else:
            # 기본값으로 추가
            features[feature_key] = {
                "dtype": "video",
                "shape": [480, 640, 3],
                "names": ["height", "width", "channel"],
                "video_info": {"video.fps": info.get("fps", 30), "video.codec": "mp4v",
                               "video.pix_fmt": "yuv420p", "video.is_depth_map": False,
                               "has_audio": False}
            }
            added.append(feature_key)

    if not added:
        logger.info("info.json: 추가할 feature 없음.")
        return

    if dry_run:
        logger.info(f"[DRY-RUN] info.json에 추가될 feature: {added}")
        return

    # 백업 후 저장
    backup = info_path.with_suffix(".json.bak")
    shutil.copy(info_path, backup)
    info["features"] = features
    with open(info_path, "w") as f:
        json.dump(info, f, indent=2)
    logger.info(f"info.json 업데이트 완료. 백업: {backup.name}")
    logger.info(f"추가된 feature: {added}")


def update_stats_json(dataset_dir: Path, new_camera_keys: list[str], dry_run: bool):
    """depth colormap은 0~255 정규화이므로 mean=0.5, std=0.5로 근사 추가."""
    stats_path = dataset_dir / "meta" / "stats.json"
    if not stats_path.exists():
        logger.warning("stats.json 없음. 건너뜀.")
        return

    with open(stats_path) as f:
        stats = json.load(f)

    added = []
    for key in new_camera_keys:
        feature_key = f"observation.images.{key}"
        if feature_key in stats:
            logger.info(f"stats.json: '{feature_key}' 이미 존재. 건너뜀.")
            continue

        # RGB와 동일한 통계 구조로 placeholder 추가
        # (정확한 값은 lerobot compute_stats로 재계산 권장)
        src_key = feature_key.replace("_depth", "")
        if src_key in stats:
            template = dict(stats[src_key])
            stats[feature_key] = template
        else:
            stats[feature_key] = {
                "mean": [[[[0.5]], [[0.5]], [[0.5]]]],
                "std":  [[[[0.5]], [[0.5]], [[0.5]]]],
                "max":  [[[[1.0]], [[1.0]], [[1.0]]]],
                "min":  [[[[0.0]], [[0.0]], [[0.0]]]],
            }
        added.append(feature_key)

    if not added:
        logger.info("stats.json: 추가할 항목 없음.")
        return

    if dry_run:
        logger.info(f"[DRY-RUN] stats.json에 추가될 항목: {added}")
        logger.warning("[DRY-RUN] stats.json placeholder 추가 후 lerobot compute_stats 재실행 권장")
        return

    backup = stats_path.with_suffix(".json.bak")
    shutil.copy(stats_path, backup)
    with open(stats_path, "w") as f:
        json.dump(stats, f, indent=2)
    logger.info(f"stats.json 업데이트 완료 (placeholder). 백업: {backup.name}")
    logger.warning("stats.json은 placeholder 값입니다. 학습 전 lerobot compute_stats 재실행 권장.")


# ──────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="LeRobot 데이터셋에 Depth Anything V2 depth 추가")
    parser.add_argument("--dataset",  required=True, help="데이터셋 루트 경로")
    parser.add_argument("--cameras",  nargs="+", default=["left_wrist", "right_wrist"],
                        help="depth를 추가할 카메라 이름 (기본: left_wrist right_wrist)")
    parser.add_argument("--model",    default="small", choices=["small", "base", "large"])
    parser.add_argument("--device",   default="cuda",  choices=["cuda", "cpu"])
    parser.add_argument("--dry-run",  action="store_true", help="실제 변경 없이 미리 확인")
    parser.add_argument("--skip-meta", action="store_true", help="info.json/stats.json 업데이트 건너뜀")
    args = parser.parse_args()

    dataset_dir = Path(args.dataset).expanduser().resolve()
    if not dataset_dir.exists():
        logger.error(f"데이터셋 경로 없음: {dataset_dir}")
        sys.exit(1)

    videos_dir = dataset_dir / "videos"
    if not videos_dir.exists():
        logger.error(f"videos 디렉토리 없음: {videos_dir}")
        sys.exit(1)

    # 처리할 비디오 파일 수집
    jobs = []  # (src_path, dst_path, depth_key)
    for cam_name in args.cameras:
        src_key = f"observation.images.{cam_name}"
        dst_key = f"observation.images.{cam_name}_depth"
        src_dir = videos_dir / src_key
        dst_dir = videos_dir / dst_key

        if not src_dir.exists():
            logger.warning(f"카메라 디렉토리 없음: {src_dir}. 건너뜀.")
            continue

        for src_file in sorted(src_dir.rglob("*.mp4")):
            # 상대 경로 유지해서 dst 경로 구성
            rel = src_file.relative_to(src_dir)
            dst_file = dst_dir / rel

            if dst_file.exists():
                src_count = count_frames(src_file)
                dst_count = count_frames(dst_file)
                if src_count == dst_count:
                    logger.info(f"이미 처리됨 (프레임 일치): {dst_file.name}. 건너뜀.")
                    continue
                else:
                    logger.warning(
                        f"기존 파일 프레임 불일치 (src={src_count}, dst={dst_count}). 재처리: {dst_file.name}"
                    )
                    if not args.dry_run:
                        dst_file.unlink()

            jobs.append((src_file, dst_file, dst_key))

    if not jobs:
        logger.info("처리할 비디오 없음.")
        if not args.skip_meta:
            depth_keys = [f"{c}_depth" for c in args.cameras]
            update_info_json(dataset_dir, depth_keys, args.dry_run)
            update_stats_json(dataset_dir, depth_keys, args.dry_run)
        return

    logger.info(f"처리할 비디오: {len(jobs)}개")
    if args.dry_run:
        for src, dst, _ in jobs:
            logger.info(f"  {src.relative_to(videos_dir)} → {dst.relative_to(videos_dir)}")
        logger.info("[DRY-RUN] 실제 처리 없이 종료.")
        if not args.skip_meta:
            depth_keys = [f"{c}_depth" for c in args.cameras]
            update_info_json(dataset_dir, depth_keys, args.dry_run)
            update_stats_json(dataset_dir, depth_keys, args.dry_run)
        return

    # 모델 로드
    pipe = load_model(args.model, args.device)

    # 비디오 처리
    failed = []
    for i, (src, dst, _) in enumerate(jobs):
        logger.info(f"[{i+1}/{len(jobs)}] {src.relative_to(videos_dir)}")
        ok = process_video(pipe, src, dst, dry_run=False)
        if not ok:
            failed.append(src)
            logger.error(f"실패: {src}")

    # 결과 요약
    logger.info("=" * 50)
    logger.info(f"완료: {len(jobs) - len(failed)}/{len(jobs)}")
    if failed:
        logger.error(f"실패한 파일 {len(failed)}개:")
        for f in failed:
            logger.error(f"  {f}")
        logger.error("실패 파일이 있어 meta 업데이트를 건너뜁니다. 문제 해결 후 재실행하세요.")
        sys.exit(1)

    # 모든 비디오 성공 시에만 meta 업데이트
    if not args.skip_meta:
        depth_keys = [f"{c}_depth" for c in args.cameras]
        update_info_json(dataset_dir, depth_keys, dry_run=False)
        update_stats_json(dataset_dir, depth_keys, dry_run=False)

    logger.info("모든 처리 완료.")


if __name__ == "__main__":
    main()
