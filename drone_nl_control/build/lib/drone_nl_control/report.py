#!/usr/bin/env python3
"""
report.py — Benchmark results reporter.

Reads the most-recent (or specified) YAML result files from
<output_dir> and prints formatted tables + writes a CSV summary.

Run:
    ros2 run drone_nl_control bench_report
    ros2 run drone_nl_control bench_report --ros-args \
        -p output_dir:=~/.drone_benchmark
"""

import os
import glob
import csv
import datetime
import yaml

import rclpy
from rclpy.node import Node


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _default_output_dir() -> str:
    return os.path.expanduser('~/.drone_benchmark')


def _latest_file(directory: str, pattern: str) -> str | None:
    files = glob.glob(os.path.join(directory, pattern))
    return max(files, key=os.path.getmtime) if files else None


def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Table printers
# ---------------------------------------------------------------------------

def _print_llm_table(data: dict):
    results = data.get('results', {})
    if not results:
        print('  (no LLM results)')
        return

    print('\n  LLM Command Validity')
    print('  ' + '=' * 64)
    print(f'  {"Model":<22} {"Trials":>7} {"Valid":>6} {"Invalid":>8} '
          f'{"Errors":>7} {"Valid%":>7} {"Lat(s)":>7}')
    print('  ' + '-' * 64)
    for label, r in results.items():
        print(f'  {label:<22} {r["trials"]:>7} {r["valid"]:>6} '
              f'{r["invalid"]:>8} {r["errors"]:>7} '
              f'{r["validity_pct"]:>6.1f}% {r["avg_latency_sec"]:>6.2f}s')
    print('  ' + '=' * 64)


def _print_vlm_table(data: dict):
    results = data.get('results', {})
    if not results:
        print('  (no VLM results)')
        return

    print('\n  VLM Binary-Response Validity')
    print('  ' + '=' * 68)
    print(f'  {"Model":<26} {"Trials":>7} {"Valid":>6} {"Invalid":>8} '
          f'{"Errors":>7} {"Valid%":>7} {"Lat(s)":>7}')
    print('  ' + '-' * 68)
    for label, r in results.items():
        print(f'  {label:<26} {r["trials"]:>7} {r["valid"]:>6} '
              f'{r["invalid"]:>8} {r["errors"]:>7} '
              f'{r["validity_pct"]:>6.1f}% {r["avg_latency_sec"]:>6.2f}s')
    print('  ' + '=' * 68)


def _print_mission_table(data: dict):
    results = data.get('results', {})
    if not results:
        print('  (no mission results)')
        return

    # Derive LLM/VLM label lists from result keys
    llm_labels = []
    vlm_labels = []
    for r in results.values():
        if r['llm'] not in llm_labels:
            llm_labels.append(r['llm'])
        if r['vlm'] not in vlm_labels:
            vlm_labels.append(r['vlm'])

    col_w = 18
    row_label = 'LLM \\ VLM'

    print('\n  Mission Success Rate (%)  —  LLM (rows) × VLM (cols)')
    sep = '  ' + '-' * (22 + col_w * len(vlm_labels))
    print(sep)
    print(f'  {row_label:<22}' + ''.join(f'{v:>{col_w}}' for v in vlm_labels))
    print(sep)
    for llm in llm_labels:
        row = f'  {llm:<22}'
        for vlm in vlm_labels:
            key = f'{llm} × {vlm}'
            r = results.get(key)
            if r:
                cell = f'{r["success_rate_pct"]:.0f}% ({r["successes"]}/{r["trials"]})'
            else:
                cell = 'N/A'
            row += f'{cell:>{col_w}}'
        print(row)
    print(sep)

    # Average steps sub-table
    print(f'\n  Average Steps to Success (or max if timeout)')
    print(sep)
    print(f'  {row_label:<22}' + ''.join(f'{v:>{col_w}}' for v in vlm_labels))
    print(sep)
    for llm in llm_labels:
        row = f'  {llm:<22}'
        for vlm in vlm_labels:
            key = f'{llm} × {vlm}'
            r = results.get(key)
            cell = f'{r["avg_steps"]:.1f}' if r else 'N/A'
            row += f'{cell:>{col_w}}'
        print(row)
    print(sep)


# ---------------------------------------------------------------------------
# CSV export
# ---------------------------------------------------------------------------

def _write_csv_llm(data: dict, out_path: str):
    results = data.get('results', {})
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['model', 'label', 'trials', 'valid', 'invalid',
                    'errors', 'validity_pct', 'avg_latency_sec'])
        for label, r in results.items():
            w.writerow([r['model'], label, r['trials'], r['valid'],
                        r['invalid'], r['errors'],
                        r['validity_pct'], r['avg_latency_sec']])


def _write_csv_vlm(data: dict, out_path: str):
    results = data.get('results', {})
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['model', 'label', 'trials', 'valid', 'invalid',
                    'errors', 'validity_pct', 'avg_latency_sec'])
        for label, r in results.items():
            w.writerow([r['model'], label, r['trials'], r['valid'],
                        r['invalid'], r['errors'],
                        r['validity_pct'], r['avg_latency_sec']])


def _write_csv_mission(data: dict, out_path: str):
    results = data.get('results', {})
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['combo', 'llm', 'vlm', 'trials', 'successes',
                    'success_rate_pct', 'avg_steps'])
        for combo, r in results.items():
            w.writerow([combo, r['llm'], r['vlm'], r['trials'],
                        r['successes'], r['success_rate_pct'], r['avg_steps']])


# ---------------------------------------------------------------------------
# ROS2 entry point
# ---------------------------------------------------------------------------

class BenchReportNode(Node):
    def __init__(self):
        super().__init__('bench_report')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('llm_file', '')
        self.declare_parameter('vlm_file', '')
        self.declare_parameter('mission_file', '')

    def params(self):
        return {
            'output_dir': self.get_parameter('output_dir').value,
            'llm_file': self.get_parameter('llm_file').value,
            'vlm_file': self.get_parameter('vlm_file').value,
            'mission_file': self.get_parameter('mission_file').value,
        }


def main(args=None):
    rclpy.init(args=args)
    node = BenchReportNode()
    p = node.params()
    node.destroy_node()
    rclpy.shutdown()

    output_dir = os.path.expanduser(p['output_dir'] or _default_output_dir())
    print(f'[bench_report] Reading results from: {output_dir}')

    llm_file = p['llm_file'] or _latest_file(output_dir, 'llm_results_*.yaml')
    vlm_file = p['vlm_file'] or _latest_file(output_dir, 'vlm_results_*.yaml')
    mission_file = (p['mission_file'] or
                    _latest_file(output_dir, 'mission_results_*.yaml'))

    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    print('\n' + '#' * 68)
    print('  DRONE NL CONTROL — BENCHMARK REPORT')
    print(f'  Generated: {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
    print('#' * 68)

    # --- LLM ---
    if llm_file:
        print(f'\n[LLM]  {llm_file}')
        llm_data = _load_yaml(llm_file)
        _print_llm_table(llm_data)
        csv_path = os.path.join(output_dir, f'llm_summary_{ts}.csv')
        _write_csv_llm(llm_data, csv_path)
        print(f'  CSV → {csv_path}')
    else:
        print('\n[LLM]  No results file found.')

    # --- VLM ---
    if vlm_file:
        print(f'\n[VLM]  {vlm_file}')
        vlm_data = _load_yaml(vlm_file)
        _print_vlm_table(vlm_data)
        csv_path = os.path.join(output_dir, f'vlm_summary_{ts}.csv')
        _write_csv_vlm(vlm_data, csv_path)
        print(f'  CSV → {csv_path}')
    else:
        print('\n[VLM]  No results file found.')

    # --- Mission ---
    if mission_file:
        print(f'\n[Mission]  {mission_file}')
        mission_data = _load_yaml(mission_file)
        _print_mission_table(mission_data)
        csv_path = os.path.join(output_dir, f'mission_summary_{ts}.csv')
        _write_csv_mission(mission_data, csv_path)
        print(f'  CSV → {csv_path}')
    else:
        print('\n[Mission]  No results file found.')

    print('\n' + '#' * 68)


if __name__ == '__main__':
    main()
