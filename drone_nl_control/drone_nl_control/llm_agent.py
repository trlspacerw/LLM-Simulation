#!/usr/bin/env python3
"""
LLMAgent — stateless class that calls Ollama to generate Turn/Move commands.

Based on the "Taking Flight with Dialogue" paper algorithm.
"""

import re
import requests

_SYSTEM_PROMPT = """\
You are a drone navigation controller. Your ONLY job is to output a single \
movement command to help the drone find the target.

Rules:
- Output EXACTLY ONE command, nothing else — no explanation, no chain-of-thought.
- Turn(θ): rotate θ degrees. Positive = clockwise. Range: -180 to 180.
- Move(d): move forward d meters. Negative = backward. Range: -20 to 20.
- If the VLM reports the target is visible, prefer Move commands to approach.
- If the target is not visible, prefer Turn commands to search.

Valid output examples:
  Turn(45.0)
  Turn(-90.0)
  Move(5.0)
  Move(-3.0)
"""


class LLMAgent:
    """Stateless LLM command generator. No ROS2 dependency."""

    def __init__(self, ollama_base_url='http://localhost:11434',
                 model='gemma3:4b', temperature=0.2):
        self._base_url = ollama_base_url.rstrip('/')
        self._model = model
        self._temperature = temperature

    # ── command generation ────────────────────────────────────────────────

    def generate(self, task: str, state_text: str,
                 vlm_detected: bool, history: list) -> str | None:
        """Call the LLM and return the raw command string, or None on failure.

        Args:
            task:         Natural-language mission description.
            state_text:   Compact drone state string from DroneInterface.
            vlm_detected: Whether the VLM detected the target in the last frame.
            history:      List of recent command strings (most recent last).

        Returns:
            Raw command string like "Turn(45.0)" or None if the call failed.
        """
        vlm_status = ('YES — target detected in camera view'
                      if vlm_detected
                      else 'NO — target not visible')

        history_str = (
            '\n'.join(f'  {i+1}. {cmd}' for i, cmd in enumerate(history))
            if history else '  (none yet)'
        )

        user_msg = (
            f'Mission: {task}\n'
            f'{state_text}\n'
            f'Target visible: {vlm_status}\n'
            f'Recent commands:\n{history_str}\n\n'
            f'Output your single command now:'
        )

        payload = {
            'model': self._model,
            'messages': [
                {'role': 'system', 'content': _SYSTEM_PROMPT},
                {'role': 'user', 'content': user_msg},
            ],
            'stream': False,
            'options': {
                'temperature': self._temperature,
                'num_predict': 20,
            },
        }

        try:
            resp = requests.post(
                f'{self._base_url}/api/chat',
                json=payload,
                timeout=30,
            )
            resp.raise_for_status()
            raw = resp.json().get('message', {}).get('content', '').strip()
            return raw
        except requests.exceptions.RequestException as e:
            print(f'[LLM] Request failed: {e}')
            return None

    # ── command parsing ───────────────────────────────────────────────────

    @staticmethod
    def parse_command(raw: str):
        """Parse a raw LLM output string into (action, value) or None.

        Valid patterns: Turn(θ)  Move(d)  (case-insensitive)
        Range checks: Turn ∈ [-180, 180], Move ∈ [-20, 20].

        Returns:
            ('Turn', float) | ('Move', float) | None
        """
        if raw is None:
            return None
        # Extract first occurrence of Turn(...) or Move(...)
        pattern = re.compile(
            r'\b(Turn|Move)\(\s*([+-]?\d+(?:\.\d+)?)\s*\)',
            re.IGNORECASE
        )
        m = pattern.search(raw)
        if not m:
            return None
        action = m.group(1).capitalize()  # 'Turn' or 'Move'
        value = float(m.group(2))

        if action == 'Turn' and not (-180.0 <= value <= 180.0):
            print(f'[LLM] Turn value out of range: {value}')
            return None
        if action == 'Move' and not (-20.0 <= value <= 20.0):
            print(f'[LLM] Move value out of range: {value}')
            return None

        return (action, value)
