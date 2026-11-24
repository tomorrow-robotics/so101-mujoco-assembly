"""The fastening signal on the real cell: the operator says when M1 is seated.

In the simulator the station checks the part against the CAD gates after every action and fastens it (stands in for
the screws). On the real cell nobody fastens anything automatically, so after every action that used compliance (the
press) the operator is asked in the terminal whether M1 is fully seated. The answer is what the agent sees as
observation['station'] -- the same format as the simulator -- and the skills tell it to let go once it is true.
`touch <out>/FASTENED` also sets it (e.g. from a second terminal).
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path


class OperatorStation:
    PART = 'M1 servo'

    def __init__(self, backend, out: Path, ask: str = 'compliant'):
        """ask: 'compliant' = after actions that used compliance; 'every' = after every action; 'never'."""
        self.b, self.out, self.ask = backend, Path(out), ask
        self.fastened = False
        self.actions_checked = 0
        self.events = []

    def _flag(self):
        return (self.out/'FASTENED').exists()

    def after_action(self, step):
        self.actions_checked += 1
        if self.fastened:
            return
        if self._flag():
            self._set(step, 'flag file')
            return
        pressed = bool(getattr(self.b, 'last_group_compliant', False))
        if self.ask == 'never' or (self.ask == 'compliant' and not pressed):
            return
        print(f'\n[station] action {self.actions_checked} ended (control step {step}). Arms are holding.', flush=True)
        while True:
            sys.stdout.write('[station] Is M1 fully seated in the Base (pushed all the way in, flush, not tilted)? [y/n] ')
            sys.stdout.flush()
            ans = sys.stdin.readline().strip().lower()
            if ans in ('y', 'yes'):
                self._set(step, 'operator')
                return
            if ans in ('n', 'no', ''):
                self.events.append({'event': 'operator_not_seated', 'control_step': step, 'unix_s': time.time()})
                return

    def _set(self, step, source):
        self.fastened = True
        self.events.append({'event': 'fastened', 'control_step': step, 'source': source, 'unix_s': time.time()})
        print(f'[station] fastened = true ({source})', flush=True)

    def status(self):
        return {'part': self.PART, 'fastened': self.fastened, 'earlier_parts_fastened': {},
                'actions_checked': self.actions_checked,
                'meaning': ('After every finished action the station checks whether the part is physically seated within '
                            'tolerance; if it is, the station fastens it (the screws) and reports fastened=true from then on. '
                            'A fastened part is rigidly joined to the Base: let go of it; pulling or pushing it moves the Base. '
                            'fastened=false after an action means the part was not seated within tolerance when that action ended.')}

    def assess(self, backend, interactive=True):
        """Operator-side result (nothing here was shown to the agent)."""
        res = {'part': 'motor', 'fastened_signal': self.fastened, 'events': self.events,
               'note': 'Operator-side result on the real cell; never shown to the policy.'}
        if interactive and sys.stdin.isatty():
            def ask(q):
                sys.stdout.write(q); sys.stdout.flush()
                return sys.stdin.readline().strip()
            res['operator_seated'] = ask('[result] M1 seated at the end? [y/n] ').lower().startswith('y')
            res['operator_hands_released'] = ask('[result] Both grippers released and withdrawn? [y/n] ').lower().startswith('y')
            res['measured_height_offset_mm'] = ask('[result] Measured M1 height above full seat in mm (Enter to skip): ') or None
            res['operator_notes'] = ask('[result] Notes (Enter to skip): ') or None
            res['success'] = bool(res['operator_seated'] and res['operator_hands_released'])
        return res
