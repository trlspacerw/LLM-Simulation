#!/usr/bin/env python3
"""
MissionController — ROS2 node that orchestrates natural language drone missions.

Architecture:
  - Main thread: rclpy.spin() to keep camera callbacks alive
  - Daemon thread: decision loop (connect → takeoff → LLM/VLM loop)

Mission input:
  - ROS2 param 'mission' (set at launch)
  - ROS2 topic /nl_mission (std_msgs/String, can be published at runtime)

Status output:
  - ROS2 topic /mission_status (std_msgs/String)
"""

import os
import time
import threading
import datetime
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from .drone_interface import DroneInterface
from .vlm_agent import VLMAgent
from .llm_agent import LLMAgent


class MissionController(Node):
    """Orchestrates the LLM→VLM→MAVLink decision loop."""

    def __init__(self):
        super().__init__('mission_controller')

        # Declare parameters with defaults
        self.declare_parameter('mission', '')
        self.declare_parameter('ollama_base_url', 'http://localhost:11434')
        self.declare_parameter('llm_model', 'gemma3:4b')
        self.declare_parameter('vlm_model', 'gemma3:4b')
        self.declare_parameter('temperature', 0.2)
        self.declare_parameter('max_steps', 20)
        self.declare_parameter('command_history_len', 5)
        self.declare_parameter('hover_altitude_m', 8.0)
        self.declare_parameter('turn_duration_sec', 2.0)
        self.declare_parameter('move_duration_sec', 2.0)
        self.declare_parameter('mavlink_connection', 'udpin:127.0.0.1:14550')

        # Read params
        p = self._get_params()

        # Sub-components
        self._vlm = VLMAgent(
            ollama_base_url=p['ollama_base_url'],
            vlm_model=p['vlm_model'],
            temperature=p['temperature'],
        )
        self._llm = LLMAgent(
            ollama_base_url=p['ollama_base_url'],
            model=p['llm_model'],
            temperature=p['temperature'],
        )
        self._drone = DroneInterface(
            connection_string=p['mavlink_connection'],
            hover_alt_m=p['hover_altitude_m'],
        )

        # Mission state
        self._mission_task = p['mission']
        self._mission_lock = threading.Lock()
        self._mission_running = False

        # ROS2 I/O
        self._mission_sub = self.create_subscription(
            String, '/nl_mission', self._mission_cb, 10)
        self._status_pub = self.create_publisher(String, '/mission_status', 10)

        self.get_logger().info('MissionController ready.')
        if self._mission_task:
            self.get_logger().info(
                f'Starting mission from param: "{self._mission_task}"')
            self._start_mission(self._mission_task)

    # ── helpers ───────────────────────────────────────────────────────────

    def _get_params(self):
        return {
            'mission': self.get_parameter('mission').value,
            'ollama_base_url': self.get_parameter('ollama_base_url').value,
            'llm_model': self.get_parameter('llm_model').value,
            'vlm_model': self.get_parameter('vlm_model').value,
            'temperature': self.get_parameter('temperature').value,
            'max_steps': self.get_parameter('max_steps').value,
            'command_history_len': self.get_parameter('command_history_len').value,
            'hover_altitude_m': self.get_parameter('hover_altitude_m').value,
            'turn_duration_sec': self.get_parameter('turn_duration_sec').value,
            'move_duration_sec': self.get_parameter('move_duration_sec').value,
            'mavlink_connection': self.get_parameter('mavlink_connection').value,
        }

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(f'[STATUS] {text}')

    # ── mission topic callback ────────────────────────────────────────────

    def _mission_cb(self, msg: String):
        task = msg.data.strip()
        if not task:
            return
        self.get_logger().info(f'Received mission via topic: "{task}"')
        self._start_mission(task)

    # ── mission start ─────────────────────────────────────────────────────

    def _start_mission(self, task: str):
        with self._mission_lock:
            if self._mission_running:
                self.get_logger().warn('Mission already running — ignoring new request')
                return
            self._mission_running = True

        t = threading.Thread(
            target=self._mission_loop,
            args=(task,),
            daemon=True,
        )
        t.start()

    # ── decision loop ─────────────────────────────────────────────────────

    def _mission_loop(self, task: str):
        """The main LLM→VLM→action loop. Runs in a daemon thread."""
        p = self._get_params()
        max_steps = p['max_steps']
        history_len = p['command_history_len']
        turn_dur = p['turn_duration_sec']
        move_dur = p['move_duration_sec']

        log = {
            'task': task,
            'start_time': datetime.datetime.utcnow().isoformat(),
            'steps': [],
            'result': 'incomplete',
        }

        try:
            # ── connect and take off ──────────────────────────────────────
            self._publish_status('Connecting to drone ...')
            self._drone.connect()
            self._publish_status(f'Taking off to {self._drone.hover_alt}m ...')
            self._drone.takeoff_and_hover()
            self._publish_status('Hover reached — starting decision loop')

            history = []
            consecutive_detections = 0

            for step in range(max_steps):
                self._publish_status(f'Step {step+1}/{max_steps}')

                # ── VLM detection ─────────────────────────────────────────
                # (image callbacks handled by vlm_thread in main())
                vlm_detected = self._vlm.ask(task)

                if vlm_detected:
                    consecutive_detections += 1
                    self._publish_status(
                        f'VLM: target detected ({consecutive_detections}/2)')
                else:
                    consecutive_detections = 0

                if consecutive_detections >= 2:
                    self._publish_status(
                        f'SUCCESS — target confirmed after {step+1} steps!')
                    log['result'] = 'success'
                    log['steps_taken'] = step + 1
                    break

                # ── Drone state ───────────────────────────────────────────
                state = self._drone.get_state()
                state_text = self._drone.format_state_text(state)

                # ── LLM command generation ────────────────────────────────
                cmd_str = None
                parsed = None
                for retry in range(3):
                    raw = self._llm.generate(
                        task=task,
                        state_text=state_text,
                        vlm_detected=vlm_detected,
                        history=history,
                    )
                    self.get_logger().info(f'LLM raw: "{raw}"')
                    parsed = LLMAgent.parse_command(raw)
                    if parsed is not None:
                        cmd_str = raw
                        break
                    self.get_logger().warn(
                        f'LLM parse failed (retry {retry+1}/3): "{raw}"')

                if parsed is None:
                    self._publish_status(f'Step {step+1}: LLM failed — skipping')
                    log['steps'].append({'step': step + 1,
                                         'vlm': vlm_detected,
                                         'cmd': None,
                                         'error': 'parse_failed'})
                    continue

                action, value = parsed
                history.append(f'{action}({value})')
                if len(history) > history_len:
                    history = history[-history_len:]

                step_log = {
                    'step': step + 1,
                    'state': state_text,
                    'vlm': vlm_detected,
                    'cmd': f'{action}({value})',
                }
                log['steps'].append(step_log)

                # ── Execute command ───────────────────────────────────────
                self._publish_status(
                    f'Step {step+1}: executing {action}({value})')
                if action == 'Turn':
                    self._drone.execute_turn(value, duration=turn_dur)
                else:
                    self._drone.execute_move(value, duration=move_dur)

            else:
                self._publish_status(
                    f'TIMEOUT — max steps ({max_steps}) reached without success')
                log['result'] = 'timeout'

        except Exception as e:
            self.get_logger().error(f'Mission error: {e}')
            self._publish_status(f'ERROR: {e}')
            log['result'] = f'error: {e}'
        finally:
            # Always stop the drone
            try:
                self._drone.hold_position()
            except Exception:
                pass

            log['end_time'] = datetime.datetime.utcnow().isoformat()
            self._save_log(log)

            with self._mission_lock:
                self._mission_running = False

    # ── log saving ────────────────────────────────────────────────────────

    def _save_log(self, log: dict):
        try:
            log_dir = '/tmp/drone_nl_logs'
            os.makedirs(log_dir, exist_ok=True)
            ts = datetime.datetime.utcnow().strftime('%Y%m%d_%H%M%S')
            path = os.path.join(log_dir, f'mission_{ts}.yaml')
            with open(path, 'w') as f:
                yaml.dump(log, f, default_flow_style=False, allow_unicode=True)
            self.get_logger().info(f'Log saved to {path}')
        except Exception as e:
            self.get_logger().error(f'Log save failed: {e}')


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    # Spin the VLM node in a dedicated thread with its own executor so that
    # image callbacks are processed independently of the main executor.
    # This avoids the rclpy "wait set index too big" crash that occurs when
    # spin_once is called from a daemon thread on a node owned by another executor.
    vlm_executor = SingleThreadedExecutor()
    vlm_executor.add_node(node._vlm)
    vlm_thread = threading.Thread(
        target=vlm_executor.spin, daemon=True, name='vlm_spin')
    vlm_thread.start()

    # Main thread spins the controller node (handles /nl_mission sub, status pub)
    main_executor = SingleThreadedExecutor()
    main_executor.add_node(node)
    try:
        main_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        vlm_executor.shutdown()
        main_executor.shutdown()
        node.destroy_node()
        node._vlm.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
