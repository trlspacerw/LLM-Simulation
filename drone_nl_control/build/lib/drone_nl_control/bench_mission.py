#!/usr/bin/env python3
"""
bench_mission.py — Mission success-rate benchmarker.

Runs N missions for every LLM × VLM combination and records
success / timeout / error outcomes plus steps taken.

Requires: simulation running + unpaused, Ollama serving all models.

Results saved to <output_dir>/mission_results_<timestamp>.yaml
Run:
    ros2 run drone_nl_control bench_mission
    ros2 run drone_nl_control bench_mission --ros-args \
        -p config_path:=/path/to/benchmark_config.yaml \
        -p trials:=3
"""

import os
import time
import datetime
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from .drone_interface import DroneInterface
from .vlm_agent import VLMAgent
from .llm_agent import LLMAgent


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Single-trial mission runner (reuses MissionController logic)
# ---------------------------------------------------------------------------

def _run_one_mission(drone: DroneInterface,
                     vlm: VLMAgent,
                     llm: LLMAgent,
                     task: str,
                     max_steps: int,
                     history_len: int,
                     turn_dur: float,
                     move_dur: float,
                     log_prefix: str) -> dict:
    """
    Execute one mission and return a result dict.

    The drone must already be connected and hovering when this is called
    (connect + takeoff are done once per trial in run_mission_benchmark).
    """
    steps_log = []
    history: list[str] = []
    consecutive = 0
    result = 'timeout'

    for step in range(max_steps):
        print(f'{log_prefix} step {step+1}/{max_steps}')

        # VLM detection
        vlm_detected = vlm.ask(task)
        consecutive = (consecutive + 1) if vlm_detected else 0
        print(f'{log_prefix}   VLM → {"YES" if vlm_detected else "no"} '
              f'(consecutive={consecutive})')

        if consecutive >= 2:
            result = 'success'
            steps_log.append({'step': step + 1, 'vlm': True, 'cmd': None,
                               'note': 'SUCCESS'})
            break

        # Drone state
        state = drone.get_state()
        state_text = drone.format_state_text(state)

        # LLM command generation (up to 3 retries)
        parsed = None
        raw_cmd = None
        for retry in range(3):
            raw = llm.generate(
                task=task,
                state_text=state_text,
                vlm_detected=vlm_detected,
                history=history,
            )
            parsed = LLMAgent.parse_command(raw)
            if parsed is not None:
                raw_cmd = raw
                break
            print(f'{log_prefix}   LLM parse fail (retry {retry+1}/3): "{raw}"')

        if parsed is None:
            steps_log.append({'step': step + 1, 'vlm': vlm_detected,
                               'cmd': None, 'note': 'llm_parse_failed'})
            continue

        action, value = parsed
        history.append(f'{action}({value})')
        if len(history) > history_len:
            history = history[-history_len:]

        steps_log.append({
            'step': step + 1,
            'vlm': vlm_detected,
            'cmd': f'{action}({value})',
            'state': state_text,
        })

        print(f'{log_prefix}   CMD → {action}({value})')
        if action == 'Turn':
            drone.execute_turn(value, duration=turn_dur)
        else:
            drone.execute_move(value, duration=move_dur)

    drone.hold_position()

    return {
        'result': result,
        'steps_taken': len(steps_log),
        'steps': steps_log,
    }


# ---------------------------------------------------------------------------
# Full benchmark over all LLM × VLM pairs
# ---------------------------------------------------------------------------

def run_mission_benchmark(cfg: dict, vlm_node: VLMAgent,
                          status_pub) -> dict:
    bench = cfg['benchmark']
    ollama_cfg = cfg['ollama']
    mission_cfg = cfg['mission']

    task = bench['mission_task']
    n_trials = bench['mission_trials']
    max_steps = mission_cfg['max_steps']
    history_len = 5
    hover_alt = mission_cfg['hover_altitude_m']
    turn_dur = mission_cfg['turn_duration_sec']
    move_dur = mission_cfg['move_duration_sec']
    mavlink = mission_cfg['mavlink_connection']
    base_url = ollama_cfg['base_url']
    temperature = ollama_cfg['temperature']

    all_results = {}

    llm_models = cfg['llm_models']
    vlm_models = cfg['vlm_models']
    total_combos = len(llm_models) * len(vlm_models)
    combo_idx = 0

    for llm_entry in llm_models:
        for vlm_entry in vlm_models:
            combo_idx += 1
            llm_name = llm_entry['name']
            llm_label = llm_entry['label']
            vlm_name = vlm_entry['name']
            vlm_label = vlm_entry['label']
            combo_key = f'{llm_label} × {vlm_label}'

            print(f'\n{"="*60}')
            print(f'[mission bench] Combo {combo_idx}/{total_combos}: '
                  f'LLM={llm_label}  VLM={vlm_label}')
            print(f'  Task: "{task}"  trials={n_trials}')
            print(f'{"="*60}')

            _publish(status_pub, f'Benchmarking: {combo_key}')

            # Swap VLM model on the shared VLMAgent node
            vlm_node._model = vlm_name

            llm = LLMAgent(
                ollama_base_url=base_url,
                model=llm_name,
                temperature=temperature,
            )

            trial_results = []

            for trial in range(1, n_trials + 1):
                prefix = f'  [trial {trial}/{n_trials}]'
                print(f'\n{prefix} --- START ---')
                _publish(status_pub,
                         f'{combo_key} trial {trial}/{n_trials}')

                drone = DroneInterface(
                    connection_string=mavlink,
                    hover_alt_m=hover_alt,
                )

                trial_log = {
                    'trial': trial,
                    'result': 'error',
                    'steps_taken': 0,
                    'steps': [],
                    'error': None,
                }

                try:
                    drone.connect()
                    drone.takeoff_and_hover()

                    mission_result = _run_one_mission(
                        drone=drone,
                        vlm=vlm_node,
                        llm=llm,
                        task=task,
                        max_steps=max_steps,
                        history_len=history_len,
                        turn_dur=turn_dur,
                        move_dur=move_dur,
                        log_prefix=prefix,
                    )
                    trial_log.update(mission_result)

                except Exception as e:
                    trial_log['error'] = str(e)
                    print(f'{prefix} ERROR: {e}')
                finally:
                    try:
                        drone.hold_position()
                    except Exception:
                        pass

                trial_results.append(trial_log)
                print(f'{prefix} result={trial_log["result"]}  '
                      f'steps={trial_log["steps_taken"]}')

                # Return to launch between trials to reset position
                if trial < n_trials:
                    print(f'{prefix} returning to launch position ...')
                    try:
                        drone.return_to_launch()
                    except Exception as e:
                        print(f'{prefix} RTL error: {e}')
                    time.sleep(3)

            # Aggregate
            successes = sum(1 for t in trial_results if t['result'] == 'success')
            success_rate = (successes / n_trials * 100) if n_trials else 0.0
            avg_steps = (
                sum(t['steps_taken'] for t in trial_results) / n_trials
                if n_trials else 0.0
            )

            all_results[combo_key] = {
                'llm': llm_label,
                'llm_model': llm_name,
                'vlm': vlm_label,
                'vlm_model': vlm_name,
                'trials': n_trials,
                'successes': successes,
                'success_rate_pct': round(success_rate, 1),
                'avg_steps': round(avg_steps, 1),
                'trial_details': trial_results,
            }

            print(f'\n  → {combo_key}: '
                  f'{successes}/{n_trials} success '
                  f'({success_rate:.1f}%)  avg_steps={avg_steps:.1f}')

    return all_results


def _publish(pub, text: str):
    if pub is not None:
        from std_msgs.msg import String as Str
        msg = Str()
        msg.data = text
        pub.publish(msg)


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class BenchMissionNode(Node):
    def __init__(self):
        super().__init__('bench_mission')
        self.declare_parameter('config_path', '')
        self.declare_parameter('trials', 0)   # 0 = use config value

    def params(self):
        return {
            'config_path': self.get_parameter('config_path').value,
            'trials': self.get_parameter('trials').value,
        }


def main(args=None):
    rclpy.init(args=args)
    node = BenchMissionNode()
    p = node.params()

    config_path = p['config_path'] or _default_config_path()
    print(f'[bench_mission] Loading config: {config_path}')
    cfg = _load_config(config_path)

    # CLI override for trial count
    if p['trials'] > 0:
        cfg['benchmark']['mission_trials'] = p['trials']
        print(f'[bench_mission] trial count overridden → {p["trials"]}')

    output_dir = os.path.expanduser(cfg['benchmark']['output_dir'])
    os.makedirs(output_dir, exist_ok=True)

    # Shared VLM node (model name is swapped per combo during the run)
    vlm_node = VLMAgent(
        ollama_base_url=cfg['ollama']['base_url'],
        vlm_model=cfg['vlm_models'][0]['name'],
        temperature=cfg['ollama']['temperature'],
    )

    status_pub = node.create_publisher(String, '/benchmark_status', 10)

    # Spin VLM camera subscriber in a background thread
    vlm_ex = SingleThreadedExecutor()
    vlm_ex.add_node(vlm_node)
    vlm_thread = threading.Thread(target=vlm_ex.spin, daemon=True,
                                  name='vlm_spin')
    vlm_thread.start()

    print('[bench_mission] Waiting 3s for camera topic to stabilise ...')
    time.sleep(3)

    try:
        results = run_mission_benchmark(cfg, vlm_node, status_pub)
    finally:
        vlm_ex.shutdown()
        node.destroy_node()
        vlm_node.destroy_node()
        rclpy.shutdown()

    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = os.path.join(output_dir, f'mission_results_{ts}.yaml')
    with open(out_path, 'w') as f:
        yaml.dump({
            'timestamp': ts,
            'config': cfg['benchmark'],
            'results': results,
        }, f, default_flow_style=False, allow_unicode=True)

    print(f'\n[bench_mission] Results saved → {out_path}')
    _print_mission_table(results, cfg)


def _print_mission_table(results: dict, cfg: dict):
    llm_labels = [m['label'] for m in cfg['llm_models']]
    vlm_labels = [m['label'] for m in cfg['vlm_models']]

    col_w = 16
    row_label = 'LLM \\ VLM'
    header = f'  {row_label:<16}' + ''.join(f'{v:>{col_w}}' for v in vlm_labels)
    sep = '  ' + '-' * (16 + col_w * len(vlm_labels))

    print('\n  Mission success rate (%)')
    print(sep)
    print(header)
    print(sep)
    for llm in llm_labels:
        row = f'  {llm:<16}'
        for vlm in vlm_labels:
            key = f'{llm} × {vlm}'
            r = results.get(key)
            cell = f'{r["success_rate_pct"]:.0f}%' if r else 'N/A'
            row += f'{cell:>{col_w}}'
        print(row)
    print(sep)


if __name__ == '__main__':
    main()
