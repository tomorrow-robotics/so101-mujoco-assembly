"""Environment/evaluator side of the SO-101 Base+M1 task (never visible to the policy).

- Fastening station: after every acknowledged control step, if M1 is physically seated in Base within
  the project gates (config/trajectory.yaml: lateral 0.25 mm, axial 0.5 mm, 1 deg) and the weld is
  inactive, activate `motor_virtual_fastening` with the *measured* relative pose (same construction as
  run_assembly.Assembly.fasten). It is never activated before seating.
- Final assessment: seated-and-fastened, Base still on the table, M1 pose error, both grippers open
  and away, and post-planning hold. This is the official result; the policy never sees it.
"""
import json

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from build_bimanual_scene import ROOT, settings
from run_assembly import pose_error


def motor_target():
    assets = settings('assembly_assets')
    return (np.array(json.loads((ROOT/assets['original']/'cad_report.json').read_text())['T_base_motor_target']),
            np.asarray(assets['advance_axis_base'], float))


def fork_target():
    """Shoulder fork on the M1 output (shoulder-pan merge): CAD pose of the fork in the Base frame with the output at
    zero angle, from the audited merge gate (RoboTwin/assets/objects/so101_full_arm/shoulder_motor2). The fork slides
    in along Base +Y (the gate's validated withdrawal axis is Base -Y)."""
    gate = json.loads((ROOT.parent/'RoboTwin/assets/objects/so101_full_arm/shoulder_motor2/shoulder_fork_gate.json').read_text())
    return np.array(gate['T_receiver_moving_target'], float), np.array([0., 1., 0.])


def shoulder_target(part):
    """Shoulder seat / M2 in the shoulder-fork frame (v1.1 gate files via scripts/build_shoulder_targets.py) and the
    direction it moves in at the end of its insertion, in that frame: axial error > 0 = past the CAD pose, < 0 = short."""
    d = json.loads((ROOT/'q0_harness/assets/tasks/shoulder_targets.json').read_text())[part]
    if 'T_receiver_part' in d:   # later steps: target in the named receiver body's frame
        return np.array(d['T_receiver_part'], float), np.array(d['insertion_dir_receiver'], float)
    return np.array(d['T_fork_part'], float), np.array(d['insertion_dir_fork'], float)


def receiver_of(part):
    d = json.loads((ROOT/'q0_harness/assets/tasks/shoulder_targets.json').read_text()).get(part, {})
    return d.get('receiver', 'shoulder_fork')


# part -> (fastening weld, CAD target in the Base frame, axis used to split lateral/axial error).
# Cap: assembled cap frame = Base frame; it seats along Base +Z (same convention as scripts/run_base_pan.py).
# Fork: axial error along Base +Y, negative = short of its depth (as for M1); its weld joins it to the M1 output.
PARTS = {'motor': ('motor_virtual_fastening', motor_target),
         'holder': ('holder_virtual_fastening', lambda: (np.eye(4), np.array([0., 0., 1.]))),
         'shoulder_fork': ('shoulder_pan_merge_weld', fork_target),
         # Seat and M2: targets in the shoulder-fork frame (both welds join the part to the fork).
         'shoulder_seat': ('shoulder_seat_virtual_fastening', lambda: shoulder_target('shoulder_seat')),
         'motor_2': ('motor_2_virtual_fastening', lambda: shoulder_target('motor_2')),
         # Upper arm, M3, forearm: targets in their receiver's frame (M2 output, upper print, M3 output).
         'upper_print': ('upper_shoulder_merge_weld', lambda: shoulder_target('upper_print')),
         'motor3': ('motor3_virtual_fastening', lambda: shoulder_target('motor3')),
         'forearm': ('forearm_elbow_merge_weld', lambda: shoulder_target('forearm'))}


# Per-part acceptance gates (metres / degrees). Default: config/trajectory.yaml (symmetric axial gate).
# M1 is asymmetric along its insertion axis because the retaining cap installed next needs it fully seated:
#   results/compliant_probe/m1_tolerance.json      cap seats with M1 0.05 mm short, jams at 0.1 mm short;
#   results/compliant_probe/m1_tolerance_over.json  cap seats with M1 0.1-0.6 mm past the CAD pose (the v1.1
#   colliders stop M1 about +0.5 mm past it), also with 0.22 mm lateral and 0.62 deg about any axis combined.
# Fork (measured against the M1 output frame, see SO101Station.reference): with the 0.2 mm sideways clearance of the
# v1.2 fork patch, a fork pushed onto its stop rests 0.16-0.30 mm sideways, +0.46..+0.50 mm along the axis at its
# origin and 0.73-0.83 deg tilted (results/fork_seat_probe/grid_v12.json, 31 trials); the gate leaves margin around that.
PART_GATES = {'motor': {'lateral': .00025, 'axial_min': -.00005, 'axial_max': .0006, 'angle_deg': .75},
              'shoulder_fork': {'lateral': .0004, 'axial_min': -.0003, 'axial_max': .0008, 'angle_deg': 1.2}}


class SO101Station:
    PART_NAMES = {'motor': 'M1 servo', 'holder': 'retaining cap', 'shoulder_fork': 'shoulder fork',
                  'shoulder_seat': 'shoulder seat', 'motor_2': 'M2 servo', 'upper_print': 'upper arm',
                  'motor3': 'M3 servo', 'forearm': 'forearm'}

    KEEP_DRIFT = {'lateral_mm': .2, 'axial_mm': .2, 'angle_deg': .2}

    def __init__(self, model, data, part='motor', keep=(), fasten='step', gates=None, keep_rule='gate'):
        """part: the part this stage installs; keep: parts fastened by earlier stages that must stay in place.
        fasten: when the station checks seating and fastens. 'step' = at every control step (original behaviour,
        kept so earlier episodes replay exactly); 'action_end' = only when an action group has finished
        (after_action), and that check is what status() reports to the policy as the station signal.
        gates: per-episode gate overrides {part: {'lateral': m, 'axial_min': m, 'axial_max': m, 'angle_deg': deg}}.
        keep_rule: how earlier parts are judged at the end. 'gate' (original) = still fastened and still inside their
        own seating gate; 'held' = still fastened and moved less than KEEP_DRIFT from where they were when this stage
        started (a fastened part only moves by the fastener's compliance; its seating gate was already passed)."""
        if fasten not in ('step', 'action_end'):
            raise ValueError("fasten must be 'step' or 'action_end'")
        if keep_rule not in ('gate', 'held'):
            raise ValueError("keep_rule must be 'gate' or 'held'")
        self.keep_rule = keep_rule
        self.fasten = fasten
        self.actions_checked = 0
        self.model, self.data = model, data
        layout = settings('trajectory')
        self.part = part
        weld, target = PARTS[part]
        self.target, self.axis = target()
        default = {'lateral': layout['position_gate_m']['lateral'], 'axial': layout['position_gate_m']['axial'],
                   'angle_deg': float(layout['rotation_gate_deg'])}
        self.gates = {p: dict(default, **PART_GATES.get(p, {}), **(gates or {}).get(p, {})) for p in PARTS}
        self.gate = {k: v for k, v in self.gates[part].items() if k != 'angle_deg'}
        self.rot_gate = self.gates[part]['angle_deg']
        self.weld = model.equality(weld).id
        # The weld stores the part's pose in its own first body (Base for M1 and the cap, the M1 output for the fork).
        self.weld_parent = model.body(int(model.eq_obj1id[self.weld])).name
        self.keep = {k: (model.equality(PARTS[k][0]).id, *PARTS[k][1]()) for k in keep}
        self.keep_start = {k: self.relative(k) for k in keep}   # earlier parts as they were when this stage started
        self.base0 = self.body('base')
        self.base_max_rise = 0.
        self.events = []
        self.best = None

    def body(self, name):
        b = self.data.body(name)
        T = np.eye(4)
        T[:3, :3] = b.xmat.reshape(3, 3)
        T[:3, 3] = b.xpos
        return T

    def reference(self, part=None):
        """Frame a part's CAD target is expressed in. M1 and the cap: the Base. The fork is mounted on the M1 output, so
        its reference is the Base frame as carried by the output (output pose with the M1 CAD offset removed): M1's own
        allowed seating error is then not counted against the fork."""
        if (part or self.part) == 'shoulder_fork':
            return self.body('motor_output') @ np.linalg.inv(motor_target()[0])
        if (part or self.part) in ('shoulder_seat', 'motor_2', 'upper_print', 'motor3', 'forearm'):
            return self.body(receiver_of(part or self.part))   # the receiver's own seating error is not counted
        return self.body('base')

    def relative(self, part=None):
        return np.linalg.solve(self.reference(part), self.body(part or self.part))

    def error(self, part=None, target=None, axis=None):
        lateral, axial, angle = pose_error(self.relative(part), self.target if target is None else target,
                                           self.axis if axis is None else axis)
        return {'lateral_mm': lateral*1000, 'axial_mm': axial*1000, 'angle_deg': angle}

    def seated(self, e=None, part=None):
        e = e or self.error()
        g = self.gates[part or self.part]
        lo, hi = g.get('axial_min', -g['axial']), g.get('axial_max', g['axial'])
        return (e['lateral_mm'] <= g['lateral']*1000 and lo*1000 <= e['axial_mm'] <= hi*1000
                and e['angle_deg'] <= g['angle_deg'])

    BASE_LIFT_LIMIT_M = .001  # process rule: Base stays on the table (rise above this counts as lifted)

    def after_step(self, step):
        rise = float(self.body('base')[2, 3] - self.base0[2, 3])
        if rise > self.base_max_rise:
            self.base_max_rise = rise
            if rise > self.BASE_LIFT_LIMIT_M and not any(x['event'] == 'base_lifted' for x in self.events):
                self.events.append({'event': 'base_lifted', 'control_step': step, 'rise_mm': rise*1000})
        e = self.error()
        score = e['lateral_mm'] + abs(e['axial_mm']) + e['angle_deg']
        if self.best is None or score < self.best[0]:
            self.best = (score, step, e)
        if self.fasten == 'step':
            self._fasten_if_seated(step, e)

    def after_action(self, step):
        """Called when an action group has finished (completed or interrupted): the station looks at the part once."""
        self.actions_checked += 1
        e = self.error()
        if self.fasten == 'action_end':
            self._fasten_if_seated(step, e)
        trace = getattr(self, 'trace_path', None)
        if trace is not None:   # evaluator-side log (run directory, never the policy's workspace)
            row = {'control_step': step, 'action': self.actions_checked, 'part': e, 'within_gate': self.seated(e),
                   'fastened': bool(self.data.eq_active[self.weld]),
                   'base_rise_mm': self._base_rise_mm(),
                   'earlier': {k: self.error(k, T, ax) for k, (w, T, ax) in self.keep.items()}}
            with open(trace, 'a') as f:
                f.write(json.dumps(row) + '\n')

    def _base_rise_mm(self):
        z = float(self.data.xpos[self.model.body('base').id][2])
        if not hasattr(self, '_base_z0'):
            self._base_z0 = z
        self._base_rise_max = max(getattr(self, '_base_rise_max', 0.), (z - self._base_z0)*1e3)
        return round(self._base_rise_max, 3)

    def status(self):
        """The station signal shown to the policy: fastened or not, nothing about poses or errors."""
        out = {'part': self.PART_NAMES[self.part], 'fastened': bool(self.data.eq_active[self.weld]),
               'earlier_parts_fastened': {self.PART_NAMES[k]: bool(self.data.eq_active[w]) for k, (w, _, _) in self.keep.items()},
               'actions_checked': self.actions_checked,
               'meaning': ('After every finished action the station checks whether the part is physically seated within '
                           'tolerance; if it is, the station fastens it (the screws) and reports fastened=true from then on. '
                           'A fastened part is rigidly joined to the Base: let go of it; pulling or pushing it moves the Base. '
                           'fastened=false after an action means the part was not seated within tolerance when that action ended.')}
        return out

    def _fasten_if_seated(self, step, e):
        if self.data.eq_active[self.weld] or not self.seated(e):
            return
        actual = np.linalg.solve(self.body(self.weld_parent), self.body(self.part))
        rel = self.model.eq_data[self.weld]
        rel[:3] = 0.
        rel[3:6] = actual[:3, 3]
        rel[6:10] = np.roll(Rotation.from_matrix(actual[:3, :3]).as_quat(), 1)
        self.data.eq_active[self.weld] = 1
        mujoco.mj_forward(self.model, self.data)
        self.events.append({'event': 'fastened_after_seating', 'control_step': step, 'error_at_activation': e})

    def assess(self, backend):
        e = self.error()
        base = self.body('base')
        drift = float(np.linalg.norm(base[:3, 3]-self.base0[:3, 3])*1000)
        tilt = float(np.degrees(Rotation.from_matrix(base[:3, :3] @ self.base0[:3, :3].T).magnitude()))
        fastened = bool(self.data.eq_active[self.weld])
        hands = {}
        for a, v in backend.arm.items():
            p = backend.pose(a)[0]
            hands[a] = {'commanded_gripper': backend.cmd_g[a], 'distance_to_base_mm': float(np.linalg.norm(p-base[:3, 3])*1000),
                        'home_error_mm': float(np.linalg.norm(p-backend.origin_poses[a][0])*1000)}
        kept = {}
        for k, (w, T, ax) in self.keep.items():
            ek = self.error(k, T, ax)
            drift = self.error(k, self.keep_start[k], ax)
            held = all(abs(drift[q]) <= lim for q, lim in self.KEEP_DRIFT.items())
            kept[k] = {'still_fastened': bool(self.data.eq_active[w]), 'error': ek, 'within_gates': self.seated(ek, k),
                       'drift_since_stage_start': drift, 'held_in_place': held,
                       'ok': bool(self.data.eq_active[w]) and (held if self.keep_rule == 'held' else self.seated(ek, k))}
        violations = [x for x in self.events if x['event'] == 'base_lifted']
        if getattr(backend, 'sim_invalid', False) or any(x.get('simulation_reset') for x in getattr(backend, 'physics_events', [])):
            violations.append({'event': 'physics_instability_reset', 'note': 'MuJoCo reset MjData; the run is invalid'})
        success = fastened and self.seated(e) and all(v['ok'] for v in kept.values()) and not violations
        return {'success': bool(success), 'part': self.part, 'process_violations': violations,
                'base_max_rise_mm': self.base_max_rise*1000, 'fastened_after_seating': fastened, 'final_error': e,
                'earlier_parts': kept,
                'seated_within_gates_at_end': self.seated(e),
                'gates': {**{k+'_mm': v*1000 for k, v in self.gate.items()}, 'angle_deg': self.rot_gate},
                'keep_rule': self.keep_rule,
                'best_error_during_episode': None if self.best is None else {'control_step': self.best[1], **self.best[2]},
                'base_drift_mm': drift, 'base_tilt_deg': tilt, 'hands': hands, 'events': self.events,
                'physics_events': backend.physics_events[:50], 'n_physics_events': len(backend.physics_events),
                'note': 'Evaluator-side result; never shown to the policy.'}
