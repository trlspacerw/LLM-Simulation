#!/usr/bin/env python3
"""
bench_vlm.py — VLM binary-response-validity benchmarker.

Measures what fraction of each VLM's responses to "Is <target> visible?"
are valid binary answers (yes / no).

Two image sources are supported (selected automatically):
  1. Saved test images in <output_dir>/test_images/   (preferred, offline)
  2. Live /front_camera/image ROS2 topic              (needs simulation running)

To pre-populate test images from a running simulation:
    ros2 run drone_nl_control bench_vlm --ros-args -p capture_images:=true

Results saved to <output_dir>/vlm_results_<timestamp>.yaml
Run:
    ros2 run drone_nl_control bench_vlm
"""

import os
import sys
import glob
import time
import base64
import datetime
import random
import yaml

import cv2
import requests
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import threading

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_TARGETS = [
    "a walking person",
    "a red car",
    "a military truck",
    "a hatchback vehicle",
    "a tree",
    "a road",
]


def _default_config_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory('drone_nl_control')
        return os.path.join(pkg, 'config', 'benchmark_config.yaml')
    except Exception:
        return os.path.join(
            os.path.dirname(__file__), '..', 'config', 'benchmark_config.yaml')


def _load_config(path: str) -> dict:
    with open(os.path.expanduser(path)) as f:
        return yaml.safe_load(f)


def _is_valid_vlm_response(content: str) -> bool:
    """Return True if the response starts with 'yes' or 'no' (case-insensitive)."""
    c = content.strip().lower()
    return c.startswith('yes') or c.startswith('no')


def _encode_frame(frame: np.ndarray) -> str:
    """Encode a BGR numpy frame as base64 JPEG."""
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise RuntimeError('JPEG encode failed')
    return base64.b64encode(buf.tobytes()).decode('utf-8')


def _query_vlm(base_url: str, model: str, temperature: float,
               img_b64: str, target: str) -> tuple[str | None, float]:
    """Send one VLM query. Returns (raw_content, latency_sec)."""
    prompt = f'Is {target} visible in this image? Answer ONLY Yes or No.'
    options = {'temperature': temperature, 'num_predict': 5}
    # llava's CLIP vision encoder crashes on GPU (SIGSEGV in CUDA); force CPU
    if 'llava' in model.lower():
        options['num_gpu'] = 0
    payload = {
        'model': model,
        'messages': [{'role': 'user', 'content': prompt, 'images': [img_b64]}],
        'stream': False,
        'options': options,
    }
    t0 = time.time()
    try:
        resp = requests.post(
            f'{base_url.rstrip("/")}/api/chat',
            json=payload,
            timeout=60,
        )
        resp.raise_for_status()
        content = resp.json().get('message', {}).get('content', '').strip()
        return content, time.time() - t0
    except Exception as e:
        print(f'    [VLM] request failed: {e}')
        return None, time.time() - t0


# ---------------------------------------------------------------------------
# Image source: saved files
# ---------------------------------------------------------------------------

def _load_test_images(image_dir: str) -> list[np.ndarray]:
    patterns = ['*.jpg', '*.jpeg', '*.png']
    paths = []
    for p in patterns:
        paths.extend(glob.glob(os.path.join(image_dir, p)))
    if not paths:
        return []
    frames = []
    for p in sorted(paths):
        img = cv2.imread(p)
        if img is not None:
            frames.append(img)
    return frames


# ---------------------------------------------------------------------------
# Image source: live ROS2 camera topic
# ---------------------------------------------------------------------------

class _CameraCapture(Node):
    """Temporary ROS2 node to collect N frames from /front_camera/image."""

    def __init__(self, n_frames: int):
        super().__init__('_bench_vlm_capture')
        self._n = n_frames
        self._frames: list[np.ndarray] = []
        self._bridge = CvBridge()
        self._done = threading.Event()
        self._sub = self.create_subscription(
            Image, '/front_camera/image', self._cb, 10)

    def _cb(self, msg: Image):
        if len(self._frames) >= self._n:
            self._done.set()
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._frames.append(frame.copy())
            print(f'  [capture] frame {len(self._frames)}/{self._n}')
            if len(self._frames) >= self._n:
                self._done.set()
        except Exception as e:
            print(f'  [capture] error: {e}')

    def collect(self, timeout: float = 30.0) -> list[np.ndarray]:
        self._done.wait(timeout=timeout)
        return self._frames


def _capture_live_frames(n_frames: int) -> list[np.ndarray]:
    """Collect n_frames from the live camera topic."""
    print(f'[bench_vlm] Waiting for {n_frames} frames from /front_camera/image ...')
    node = _CameraCapture(n_frames)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    frames = node.collect(timeout=60.0)
    ex.shutdown()
    node.destroy_node()
    if not frames:
        print('[bench_vlm] WARNING: no frames received from camera topic')
    return frames


# ---------------------------------------------------------------------------
# Core benchmark
# ---------------------------------------------------------------------------

def run_vlm_benchmark(cfg: dict, frames: list[np.ndarray]) -> dict:
    bench = cfg['benchmark']
    ollama = cfg['ollama']
    n_trials = bench['vlm_trials']
    base_url = ollama['base_url']
    temperature = ollama['temperature']

    if not frames:
        print('[bench_vlm] ERROR: no images available — aborting VLM benchmark')
        return {}

    # Pre-encode all frames to base64 (avoid re-encoding each query)
    encoded = [_encode_frame(f) for f in frames]
    print(f'[bench_vlm] {len(encoded)} test images ready')

    all_results = {}

    for model_entry in cfg['vlm_models']:
        model_name = model_entry['name']
        label = model_entry['label']
        print(f'\n[VLM bench] {label} ({model_name})  — {n_trials} queries')

        valid = 0
        invalid = 0
        errors = 0
        latencies = []
        raw_samples = []

        for i in range(n_trials):
            img_b64 = encoded[i % len(encoded)]
            target = _TARGETS[i % len(_TARGETS)]

            content, elapsed = _query_vlm(
                base_url, model_name, temperature, img_b64, target)

            if content is None:
                errors += 1
                print(f'  [{i+1:3d}] ERROR')
                continue

            latencies.append(elapsed)
            is_valid = _is_valid_vlm_response(content)

            if is_valid:
                valid += 1
                status = 'OK '
            else:
                invalid += 1
                status = 'BAD'

            if len(raw_samples) < 5:
                raw_samples.append({
                    'target': target,
                    'response': content[:80],
                    'valid': is_valid,
                })

            print(f'  [{i+1:3d}] {status}  lat={elapsed:.2f}s  '
                  f'target="{target}"  response="{content[:40]}"')

        validity_pct = (valid / (valid + invalid) * 100) if (valid + invalid) > 0 else 0.0
        avg_lat = (sum(latencies) / len(latencies)) if latencies else 0.0

        result = {
            'model': model_name,
            'label': label,
            'trials': n_trials,
            'valid': valid,
            'invalid': invalid,
            'errors': errors,
            'validity_pct': round(validity_pct, 1),
            'avg_latency_sec': round(avg_lat, 3),
            'samples': raw_samples,
        }
        all_results[label] = result
        print(f'  → {label}: {valid}/{valid+invalid} valid '
              f'({validity_pct:.1f}%)  avg_lat={avg_lat:.2f}s')

    return all_results


# ---------------------------------------------------------------------------
# ROS2 entry point
# ---------------------------------------------------------------------------

class BenchVLMNode(Node):
    def __init__(self):
        super().__init__('bench_vlm')
        self.declare_parameter('config_path', '')
        self.declare_parameter('capture_images', False)

    def params(self):
        return {
            'config_path': self.get_parameter('config_path').value,
            'capture_images': self.get_parameter('capture_images').value,
        }


def main(args=None):
    rclpy.init(args=args)
    node = BenchVLMNode()
    p = node.params()
    node.destroy_node()

    config_path = p['config_path'] or _default_config_path()
    print(f'[bench_vlm] Loading config: {config_path}')
    cfg = _load_config(config_path)

    output_dir = os.path.expanduser(cfg['benchmark']['output_dir'])
    os.makedirs(output_dir, exist_ok=True)
    image_dir = os.path.join(output_dir, 'test_images')
    os.makedirs(image_dir, exist_ok=True)

    # --- Determine image source ---
    frames = _load_test_images(image_dir)

    if not frames or p['capture_images']:
        print(f'[bench_vlm] No saved images in {image_dir} '
              f'(or capture_images=true) — collecting from live camera ...')
        n_capture = cfg['benchmark']['vlm_trials']
        frames = _capture_live_frames(n_capture)
        # Save for future offline runs
        for idx, frame in enumerate(frames):
            cv2.imwrite(os.path.join(image_dir, f'frame_{idx:04d}.jpg'), frame)
        print(f'[bench_vlm] Saved {len(frames)} frames to {image_dir}')
    else:
        print(f'[bench_vlm] Loaded {len(frames)} saved test images from {image_dir}')

    rclpy.shutdown()

    results = run_vlm_benchmark(cfg, frames)

    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = os.path.join(output_dir, f'vlm_results_{ts}.yaml')
    with open(out_path, 'w') as f:
        yaml.dump({
            'timestamp': ts,
            'config': cfg['benchmark'],
            'results': results,
        }, f, default_flow_style=False, allow_unicode=True)

    print(f'\n[bench_vlm] Results saved → {out_path}')
    _print_vlm_table(results)


def _print_vlm_table(results: dict):
    print('\n' + '=' * 60)
    print(f'  {"Model":<24} {"Valid":>6} {"Invalid":>8} {"Errors":>7} {"Valid%":>7} {"Lat(s)":>7}')
    print('  ' + '-' * 58)
    for label, r in results.items():
        print(f'  {label:<24} {r["valid"]:>6} {r["invalid"]:>8} '
              f'{r["errors"]:>7} {r["validity_pct"]:>6.1f}% '
              f'{r["avg_latency_sec"]:>6.2f}s')
    print('=' * 60)


if __name__ == '__main__':
    main()
