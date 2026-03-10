"""
Depth Anything V2 비동기 추론 워커.

카메라별로 백그라운드 스레드를 생성해 V2 depth 추론을 수행.
메인 30Hz 루프를 블로킹하지 않고 최신 결과를 제공.

사용법 (cameras.py 내부에서):
    worker = DepthV2Worker(model_size="small", device="cuda")
    worker.start()
    worker.submit(rgb_frame)          # 최신 프레임 제출 (논블로킹)
    depth_colormap = worker.get()     # 최신 결과 반환 (없으면 None)
    worker.stop()
"""

import logging
import threading
import time

import cv2
import numpy as np
from numpy.typing import NDArray

logger = logging.getLogger(__name__)


class DepthV2Worker:
    """Depth Anything V2를 백그라운드 스레드에서 실행하는 워커."""

    def __init__(self, model_size: str = "small", device: str = "cuda"):
        self._model_size = model_size
        self._device = device
        self._pipe = None

        self._lock = threading.Lock()
        self._input_frame: NDArray | None = None  # 최신 입력 프레임
        self._output_depth: NDArray | None = None  # 최신 depth colormap 결과
        self._new_input = threading.Event()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._ready = False  # 모델 로드 완료 여부

    def start(self):
        """모델 로드 + 추론 스레드 시작."""
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        """추론 스레드 종료."""
        self._stop_event.set()
        self._new_input.set()  # 대기 중인 스레드 깨우기
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    def submit(self, frame_rgb: NDArray):
        """최신 프레임 제출 (논블로킹). 처리 못한 이전 프레임은 덮어씀."""
        with self._lock:
            self._input_frame = frame_rgb.copy()
        self._new_input.set()

    def get(self) -> NDArray | None:
        """최신 depth colormap 반환. 아직 결과 없으면 None."""
        with self._lock:
            return self._output_depth.copy() if self._output_depth is not None else None

    @property
    def is_ready(self) -> bool:
        return self._ready

    def _load_model(self):
        from transformers import pipeline as hf_pipeline
        import torch

        model_id = {
            "small": "depth-anything/Depth-Anything-V2-Small-hf",
            "base":  "depth-anything/Depth-Anything-V2-Base-hf",
            "large": "depth-anything/Depth-Anything-V2-Large-hf",
        }[self._model_size]

        device = self._device if self._device != "cuda" else (
            "cuda" if __import__("torch").cuda.is_available() else "cpu"
        )
        logger.info(f"[DepthV2] Loading model {model_id} on {device}...")
        self._pipe = hf_pipeline(
            task="depth-estimation",
            model=model_id,
            device=device,
        )
        logger.info(f"[DepthV2] Model ready ({self._model_size})")

    def _infer(self, frame_rgb: NDArray) -> NDArray:
        """RGB (H,W,3) → depth colormap (H,W,3) RGB."""
        from PIL import Image
        h, w = frame_rgb.shape[:2]
        pil_img = Image.fromarray(frame_rgb)
        result = self._pipe(pil_img)
        depth = np.array(result["depth"], dtype=np.float32)

        # normalize → colormap
        dmin, dmax = depth.min(), depth.max()
        if dmax - dmin > 1e-6:
            norm = (depth - dmin) / (dmax - dmin)
        else:
            norm = np.zeros_like(depth)
        depth_uint8 = (norm * 255).astype(np.uint8)
        colormap_bgr = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_INFERNO)
        colormap_rgb = cv2.cvtColor(colormap_bgr, cv2.COLOR_BGR2RGB)
        # 원본 해상도로 맞추기
        if colormap_rgb.shape[:2] != (h, w):
            colormap_rgb = cv2.resize(colormap_rgb, (w, h))
        return colormap_rgb

    def _run(self):
        try:
            self._load_model()
            self._ready = True
        except Exception as e:
            logger.error(f"[DepthV2] Model load failed: {e}")
            return

        while not self._stop_event.is_set():
            self._new_input.wait(timeout=1.0)
            self._new_input.clear()

            if self._stop_event.is_set():
                break

            with self._lock:
                frame = self._input_frame
                self._input_frame = None

            if frame is None:
                continue

            try:
                t0 = time.perf_counter()
                depth_colormap = self._infer(frame)
                dt = time.perf_counter() - t0
                logger.debug(f"[DepthV2] infer {dt*1000:.0f}ms")

                with self._lock:
                    self._output_depth = depth_colormap
            except Exception as e:
                logger.warning(f"[DepthV2] Inference error: {e}")
