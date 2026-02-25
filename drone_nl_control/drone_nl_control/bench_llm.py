#!/usr/bin/env python3
"""
bench_llm.py — LLM command-validity benchmarker.

No ROS2 or simulation required. Generates synthetic drone-state prompts
and measures what fraction of each LLM's responses parse as valid
Turn(θ) / Move(d) commands.

Results saved to <output_dir>/llm_results_<timestamp>.yaml
Run:
    ros2 run drone_nl_control bench_llm
    ros2 run drone_nl_control bench_llm --ros-args -p config_path:=/path/to/benchmark_config.yaml
"""

import os
import sys
import time
import random
import datetime
import yaml

import rclpy
from rclpy.node import Node

from .llm_agent import LLMAgent


# ---------------------------------------------------------------------------
# Synthetic prompt helpers
# ---------------------------------------------------------------------------

_TASKS = [
    "find the walking person",
    "locate the red car",
    "find the military truck",
    "search for the hatchback vehicle",
    "find the pedestrian",
]

_HEADINGS = list(range(0, 360, 45))   # 0, 45, 90, … 315


def _random_state_text() -> str:
    alt = round(random.uniform(2.0, 10.0), 1)
    hdg = random.choice(_HEADINGS)
    spd = round(random.uniform(0.0, 3.0), 1)
    return f"Drone state: altitude {alt}m, heading {hdg}°, speed {spd}m/s, armed"


def _random_history(length: int) -> list:
    cmds = []
    for _ in range(length):
        if random.random() < 0.5:
            cmds.append(f"Turn({random.choice([-90, -45, 45, 90])}.0)")
        else:
            cmds.append(f"Move({random.choice([3, 5, 8])}.0)")
    return cmds


# ---------------------------------------------------------------------------
# Benchmark runner
# ---------------------------------------------------------------------------

def _load_config(path: str) -> dict:
    with open(os.path.expanduser(path)) as f:
        return yaml.safe_load(f)


def _default_config_path() -> str:
    """Return the installed benchmark_config.yaml path."""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory('drone_nl_control')
        return os.path.join(pkg, 'config', 'benchmark_config.yaml')
    except Exception:
        return os.path.join(
            os.path.dirname(__file__), '..', 'config', 'benchmark_config.yaml')


def run_llm_benchmark(cfg: dict) -> dict:
    """Run the LLM validity benchmark and return a results dict."""
    bench = cfg['benchmark']
    ollama = cfg['ollama']
    n_trials = bench['llm_trials']
    task = bench['mission_task']
    base_url = ollama['base_url']
    temperature = ollama['temperature']

    all_results = {}

    for model_entry in cfg['llm_models']:
        model_name = model_entry['name']
        label = model_entry['label']
        print(f'\n[LLM bench] {label} ({model_name})  — {n_trials} prompts')

        agent = LLMAgent(
            ollama_base_url=base_url,
            model=model_name,
            temperature=temperature,
        )

        valid = 0
        invalid = 0
        errors = 0
        latencies = []
        raw_samples = []   # keep up to 5 samples for inspection

        for i in range(n_trials):
            state_text = _random_state_text()
            history = _random_history(random.randint(0, 4))
            vlm_detected = random.random() < 0.5

            t0 = time.time()
            raw = agent.generate(
                task=task,
                state_text=state_text,
                vlm_detected=vlm_detected,
                history=history,
            )
            elapsed = time.time() - t0

            if raw is None:
                errors += 1
                print(f'  [{i+1:3d}] ERROR (request failed)')
                continue

            latencies.append(elapsed)
            parsed = LLMAgent.parse_command(raw)

            if parsed is not None:
                valid += 1
                status = 'OK '
            else:
                invalid += 1
                status = 'BAD'

            if len(raw_samples) < 5:
                raw_samples.append({'raw': raw, 'valid': parsed is not None})

            print(f'  [{i+1:3d}] {status}  lat={elapsed:.2f}s  "{raw[:60]}"')

        total_attempted = valid + invalid + errors
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

class BenchLLMNode(Node):
    def __init__(self):
        super().__init__('bench_llm')
        self.declare_parameter('config_path', '')

    def get_config_path(self) -> str:
        path = self.get_parameter('config_path').value
        return path if path else _default_config_path()


def main(args=None):
    rclpy.init(args=args)
    node = BenchLLMNode()
    config_path = node.get_config_path()
    node.destroy_node()
    rclpy.shutdown()

    print(f'[bench_llm] Loading config: {config_path}')
    cfg = _load_config(config_path)

    output_dir = os.path.expanduser(cfg['benchmark']['output_dir'])
    os.makedirs(output_dir, exist_ok=True)

    results = run_llm_benchmark(cfg)

    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = os.path.join(output_dir, f'llm_results_{ts}.yaml')
    with open(out_path, 'w') as f:
        yaml.dump({
            'timestamp': ts,
            'config': cfg['benchmark'],
            'results': results,
        }, f, default_flow_style=False, allow_unicode=True)

    print(f'\n[bench_llm] Results saved → {out_path}')
    _print_llm_table(results)


def _print_llm_table(results: dict):
    print('\n' + '=' * 56)
    print(f'  {"Model":<20} {"Valid":>6} {"Invalid":>8} {"Errors":>7} {"Valid%":>7} {"Lat(s)":>7}')
    print('  ' + '-' * 54)
    for label, r in results.items():
        print(f'  {label:<20} {r["valid"]:>6} {r["invalid"]:>8} '
              f'{r["errors"]:>7} {r["validity_pct"]:>6.1f}% '
              f'{r["avg_latency_sec"]:>6.2f}s')
    print('=' * 56)


if __name__ == '__main__':
    main()
