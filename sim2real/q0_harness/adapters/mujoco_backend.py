"""Native Panthera/MuJoCo backend with the interface of upstream Q0 controller/backend.py.

Upstream Backend talks to a remote RoboDojo evaluator (X5 arms, CuRobo planner, collision spheres).
This class replaces only that boundary; the reused upstream Episode, Executor, motion
(PosePathSearch, path diagnostics, legal-equivalent search, group compilation) call it unchanged.

Equivalences (upstream -> here)
- evaluator step at 25 Hz            -> one control step = control_dt of native MuJoCo physics, joint
                                        targets tracked by the existing torque PD (config/physics.yaml)
- X5 URDF FK / CuRobo planner        -> native MjModel FK; planner='curobo': the upstream CuroboPlanner on the
                                        GPU pod (adapters/pod_curobo.py, Panthera config in
                                        controller/calibration_panthera/curobo, flange = link6 targets, table
                                        cuboid at the compiled lab_table top, FK-checked against the TCP site);
                                        planner='mujoco' (default/fallback): MujocoPlanner, multi-seed IK +
                                        collision-checked joint path against the table and the arm itself
- collision spheres, 2 mm inter-arm  -> native arm collision geoms, 2 mm inter-arm, fingers posed at the
                                        commanded/held-width gap range (same gap rule as upstream)
- remote RGB + K + camera-to-world   -> rendered native cameras (adapters/mujoco_cameras.py)
- evaluator closes the episode       -> `post_planning_hold_steps` acknowledged hold steps after END

Object poses are NOT exposed unless `expose_object_ground_truth=True` (upstream exposes none).
"""
import copy
import math
import time
from types import SimpleNamespace

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from ..controller import robot_geometry as rg
from ..controller.executor import Replan, Ended, StepBudgetExhausted, angle, execution_limits
from ..controller.joint_limits import JointLimits
from ..runtime.protocol import ARMS
from .mujoco_cameras import CameraRig

PREFIX = {'left': 'a', 'right': 'b'}
INTER_ARM_CLEARANCE_M = .002          # upstream validate_pair
IK_POSITION_TOL_M = 2e-4
IK_ROTATION_TOL_RAD = math.radians(.2)
IK_SEEDS = 16
PATH_CHECK_RAD = .05                  # same sampling as upstream motion.PATH_SAMPLE_RAD
VIA_ATTEMPTS = 40


class MujocoPlanner:
    """CuroboPlanner-shaped: plan_path / plan_joint / solve_ik_to_joint, results as status dicts."""

    def __init__(self, backend, arm):
        self.b, self.arm = backend, arm
        self.rng = np.random.default_rng(0)

    # --- IK -------------------------------------------------------------------------------------
    def _solve(self, seed, p, r):
        b, info = self.b, self.b.arm[self.arm]
        lo, hi = b.joint_limits[self.arm].bounds.T
        lo_i, hi_i = lo+1e-6, hi-1e-6

        def residual(q):
            p_, r_ = b.fk(self.arm, q)
            return np.r_[p_-p, .2*Rotation.from_matrix(r_ @ r.T).as_rotvec()]
        sol = least_squares(residual, np.clip(seed, lo_i, hi_i), bounds=(lo_i, hi_i), max_nfev=200,
                            ftol=1e-12, xtol=1e-12, gtol=1e-12)
        p_, r_ = b.fk(self.arm, sol.x)
        ok = np.linalg.norm(p_-p) <= IK_POSITION_TOL_M and angle(r_, r) <= IK_ROTATION_TOL_RAD
        return sol.x, ok

    def solve_ik_to_joint(self, seed, target_pose, origin_pose=None):
        p, r = target_pose
        seeds = [np.asarray(seed, float)]
        lo, hi = self.b.joint_limits[self.arm].bounds.T
        rng = np.random.default_rng(abs(hash((tuple(np.round(p, 5)), self.arm))) % 2**32)
        for k in range(IK_SEEDS-1):
            scale = .3 if k < IK_SEEDS//2 else 1.
            seeds.append(np.clip(seeds[0] + scale*rng.uniform(-1, 1, len(lo))*(hi-lo)/2, lo, hi))
        found = []
        for s in seeds:
            q, ok = self._solve(s, p, r)
            if ok and self.b.arm_world_collision(self.arm, q) is None:
                if not any(np.max(abs(q-x)) < 1e-3 for x in found):
                    found.append(q)
        if not found:
            return {'status': 'IK Fail'}
        return {'status': 'Success', 'joint_value': found[0], 'joint_candidates': found}

    # --- joint paths ----------------------------------------------------------------------------
    def _segment_free(self, a, c):
        n = max(1, math.ceil(float(np.max(abs(c-a)))/PATH_CHECK_RAD))
        for u in np.linspace(0, 1, n+1)[1:]:
            if self.b.arm_world_collision(self.arm, (1-u)*a+u*c) is not None:
                return False
        return True

    def plan_joint(self, start, goal):
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        if self.b.arm_world_collision(self.arm, goal) is not None:
            return {'status': 'Goal in collision'}
        if self._segment_free(start, goal):
            return {'status': 'Success', 'position': np.vstack([goal])}
        # Two-segment detour through a sampled collision-free via configuration (bounded search).
        lo, hi = self.b.joint_limits[self.arm].bounds.T
        mid = (start+goal)/2
        for k in range(VIA_ATTEMPTS):
            scale = .15 + .85*k/VIA_ATTEMPTS
            via = np.clip(mid + scale*self.rng.uniform(-1, 1, len(lo))*(hi-lo)/2, lo, hi)
            if self.b.arm_world_collision(self.arm, via) is None and self._segment_free(start, via) \
                    and self._segment_free(via, goal):
                return {'status': 'Success', 'position': np.vstack([via, goal])}
        return {'status': 'Fail'}

    def plan_path(self, start, target_pose, real_robot_pose=None):
        ik = self.solve_ik_to_joint(start, target_pose)
        if ik['status'] != 'Success':
            return {'status': ik['status']}
        for q in sorted(ik['joint_candidates'], key=lambda q: float(np.linalg.norm(q-start))):
            result = self.plan_joint(start, q)
            if result['status'] == 'Success':
                return result
        return {'status': 'Fail'}


class NativeMujocoBackend:
    def __init__(self, model, data, max_control_steps, control_dt=.04, cameras=None, image_size=(640, 480),
                 post_planning_hold_steps=50, expose_object_ground_truth=False, instruction='', rig=None,
                 planner='mujoco', curobo_options=None):
        """planner: 'mujoco' (native stand-in) or 'curobo' (upstream CuroboPlanner on the pod; curobo_options are
        passed to adapters.pod_curobo.start_backend_planners, e.g. gpu, job, log_path)."""
        if type(max_control_steps) is not int or max_control_steps <= 0:
            raise ValueError('An explicit positive task control-step limit is required')
        from build_bimanual_scene import settings  # project module (scripts/ on sys.path)
        from q0_harness.adapters.model_loading import require_unit_polygon_normals
        require_unit_polygon_normals(model)  # garbage multi-contact points otherwise (see model_loading.py)
        self.model, self.data = model, data
        self.scratch = mujoco.MjData(model)
        self.control_dt = float(control_dt)
        self.substeps = round(self.control_dt/model.opt.timestep)
        if self.substeps < 1 or not np.isclose(self.substeps*model.opt.timestep, self.control_dt):
            raise ValueError('control_dt must be an integer multiple of the physics timestep')
        self.max_control_steps, self.steps, self.actions_sent = max_control_steps, 0, 0
        self.step_phase = 'initial'
        self.on_step_acknowledged = lambda: None
        self.instruction = instruction
        self.expose_objects = bool(expose_object_ground_truth)
        self.post_hold_limit, self.post_hold = int(post_planning_hold_steps), 0
        self._cancelled, self._error = False, None
        self.physics_events = []
        self.after_step_hooks = []
        self.sim_invalid = False
        self.compliance = None  # optional Cartesian impedance on one arm (see step_compliant)
        self.group_compliance = None  # set for the duration of a plan group that requests compliance
        self.after_group_hooks = []   # environment side, called with the step count when an action group has ended
        self.station_status = None    # callable -> dict shown to the policy as observation['station'] (fastening signal)
        self._jacp, self._jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        self.record = []  # per acknowledged control step: commands, phase and full native state
        self.observation_index = 0
        self.servo = settings('physics')['servo']
        self.arm, self.joint_limits = {}, {}
        for a, s in PREFIX.items():
            joints = [model.joint(f'{s}_joint{i}') for i in range(1, 7)]
            kinds = ['revolute' if model.jnt_limited[j.id] else 'continuous' for j in joints]
            bounds = [tuple(model.jnt_range[j.id]) if model.jnt_limited[j.id] else (-math.inf, math.inf)
                      for j in joints]
            self.joint_limits[a] = JointLimits([j.name for j in joints], kinds, bounds)
            bodies = {i for i in range(model.nbody) if model.body(i).name.startswith(s+'_')}
            geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] in bodies
                     and (model.geom_contype[g] or model.geom_conaffinity[g])]
            self.arm[a] = dict(prefix=s, qids=np.array([j.qposadr[0] for j in joints]),
                               vids=np.array([j.dofadr[0] for j in joints]),
                               act=np.array([model.actuator(f'{s}_joint{i}_torque').id for i in range(1, 7)]),
                               grip=model.actuator(f'{s}_gripper').id, site=model.site(f'{s}_tcp').id,
                               finger_q=(model.joint(f'{s}_gripper_L_joint').qposadr[0],
                                         model.joint(f'{s}_gripper_R_joint').qposadr[0]),
                               bodies=bodies, geoms=geoms, base=model.body(f'{s}_base_link').id)
        self.table = model.geom('lab_table').id
        mujoco.mj_forward(model, data)
        normal = data.geom_xmat[self.table].reshape(3, 3)[:, 2]
        self.table_z = float((data.geom_xpos[self.table] + model.geom_size[self.table, 2]*normal)[2])
        rg.TABLE_SURFACE_Z = rg.TOOL_TABLE_GUARD_Z = self.table_z
        self.ctrl_max = {a: float(model.actuator_ctrlrange[v['grip'], 1]) for a, v in self.arm.items()}
        self.cmd_g = {a: 1-float(data.ctrl[v['grip']])/self.ctrl_max[a] for a, v in self.arm.items()}
        self.targets = self.joints()
        self.held_width_mm = {a: None for a in ARMS}
        self._collision_ranges = None
        self.collision_context, self.last_collision = {}, None
        self.robots = {a: SimpleNamespace(entity_origin_pose=np.r_[data.xpos[v['base']], data.xquat[v['base']]])
                       for a, v in self.arm.items()}
        if planner not in ('mujoco', 'curobo'):
            raise ValueError(f'unknown planner {planner!r}')
        self.planner_kind, self.curobo_service = planner, None
        if planner == 'curobo':
            from .pod_curobo import start_backend_planners
            self.curobo_service, self.planners = start_backend_planners(self, **(curobo_options or {}))
        else:
            self.planners = {a: MujocoPlanner(self, a) for a in ARMS}
        self.path_diagnostics = {}
        self.origin_poses = {a: self.pose(a) for a in ARMS}
        self.rig = rig if rig is not None else CameraRig(model, cameras, *image_size)

    # --- state ------------------------------------------------------------------------------------
    def joints(self):
        return {a: self.data.qpos[v['qids']].copy() for a, v in self.arm.items()}

    def grippers(self):
        return dict(self.cmd_g)

    def gripper_commands(self):
        return dict(self.cmd_g)

    def pose(self, a):
        s = self.data.site(self.arm[a]['site'])
        return s.xpos.copy(), s.xmat.reshape(3, 3).copy()

    def fk(self, a, q):
        v = self.arm[a]
        self.scratch.qpos[:] = self.data.qpos
        self.scratch.qpos[v['qids']] = q
        mujoco.mj_kinematics(self.model, self.scratch)
        s = self.scratch.site(v['site'])
        return s.xpos.copy(), s.xmat.reshape(3, 3).copy()

    def sim_time(self):
        return self.steps*self.control_dt

    def remaining_steps(self):
        return max(0, self.max_control_steps-self.steps)

    def budget_state(self):
        return {'control_steps_used': self.steps, 'control_steps_sent': self.actions_sent,
                'unacknowledged_actions': self.actions_sent-self.steps,
                'control_step_limit': self.max_control_steps,
                'control_steps_remaining': self.remaining_steps(),
                'control_time_remaining_s': self.remaining_steps()*self.control_dt,
                'control_step_limit_source': 'audited static task definition',
                'sim_time_s': self.sim_time(), 'control_dt_s': self.control_dt}

    def ended(self):
        return self._cancelled

    # --- collision -----------------------------------------------------------------------------
    def _pose_scratch(self, q_arms, gaps_mm=None):
        self.scratch.qpos[:] = self.data.qpos
        for a, q in q_arms.items():
            v = self.arm[a]
            self.scratch.qpos[v['qids']] = q
            if gaps_mm is not None and a in gaps_mm:
                closure = float(np.clip((rg.FULL_OPEN_GAP_MM-gaps_mm[a])/2000., 0, self.ctrl_max[a]))
                self.scratch.qpos[v['finger_q'][0]] = -closure
                self.scratch.qpos[v['finger_q'][1]] = closure
        mujoco.mj_fwdPosition(self.model, self.scratch)

    def arm_world_collision(self, a, q):
        """CuRobo-equivalent world for one arm: the table and the arm itself (no scene objects)."""
        self._pose_scratch({a: q})
        v = self.arm[a]
        for c in self.scratch.contact[:self.scratch.ncon]:
            if c.dist >= 0:
                continue
            g1, g2 = int(c.geom[0]), int(c.geom[1])
            b1, b2 = self.model.geom_bodyid[g1], self.model.geom_bodyid[g2]
            in1, in2 = b1 in v['bodies'], b2 in v['bodies']
            if not (in1 or in2):
                continue
            other = g2 if in1 else g1
            mine = b1 if in1 else b2
            if other == self.table and mine != v['base']:
                return ('table', self.model.body(mine).name, float(c.dist))
            if in1 and in2:
                return ('self', self.model.body(b1).name, self.model.body(b2).name, float(c.dist))
        return None

    def gripper_estimates(self, group, when, previous=None):
        widths = dict(self.held_width_mm if previous is None else previous)
        for a in ARMS:
            command = group['grippers'][a]
            if command['when'] == when:
                if command['command'] == 'open':
                    widths[a] = None
                elif command.get('held_width_mm') is not None:
                    widths[a] = float(command['held_width_mm'])
        return widths

    @staticmethod
    def collision_gap_range(command, held_width):
        # Same rule as upstream, with the Panthera empty-closed gap instead of the X5 0.8 mm.
        closed = rg.EMPTY_CLOSED_GAP_MM
        gap = rg.jaw_gap_mm(command)
        if held_width is None and gap <= closed+1e-6:
            return (closed, rg.jaw_gap_mm(1.))
        gap = max(closed, gap, held_width or 0.)
        return (gap, gap)

    def set_collision_grippers(self, before, after, held_before=None, held_after=None):
        held_before = self.held_width_mm if held_before is None else held_before
        held_after = held_before if held_after is None else held_after
        self._collision_ranges = {}
        for a in ARMS:
            lo = self.collision_gap_range(before[a], held_before[a])
            hi = self.collision_gap_range(after[a], held_after[a])
            self._collision_ranges[a] = (min(lo[0], hi[0]), max(lo[1], hi[1]))

    def _inter_arm_clearance(self):
        m, d = self.model, self.scratch
        best, pair, fromto = math.inf, None, np.zeros(6)
        for g1 in self.arm['left']['geoms']:
            for g2 in self.arm['right']['geoms']:
                centre = np.linalg.norm(d.geom_xpos[g1]-d.geom_xpos[g2])
                if centre - m.geom_rbound[g1] - m.geom_rbound[g2] > INTER_ARM_CLEARANCE_M:
                    continue
                dist = mujoco.mj_geomDistance(m, d, g1, g2, INTER_ARM_CLEARANCE_M, fromto)
                if dist < best:
                    best, pair = dist, (m.body(m.geom_bodyid[g1]).name, m.body(m.geom_bodyid[g2]).name, fromto.copy())
        return best, pair

    def validate_pair(self, q):
        for a in ARMS:
            self.joint_limits[a].validate(q[a])
            p, r = self.fk(a, q[a])
            if min(p[2], (p+rg.FINGERTIP_REACH_M*r[:, 0])[2]) < rg.TOOL_TABLE_GUARD_Z:
                raise Replan(f'{a} tool would penetrate the table')
        ranges = self._collision_ranges or {a: self.collision_gap_range(self.cmd_g[a], self.held_width_mm[a])
                                            for a in ARMS}
        worst = (math.inf, None)
        for k in (0, 1):  # both ends of the commanded/held gap range, as upstream inflates spheres
            self._pose_scratch(q, {a: ranges[a][k] for a in ARMS})
            dist, pair = self._inter_arm_clearance()
            if dist < worst[0]:
                worst = (dist, pair)
        distance, pair = worst
        if distance < INTER_ARM_CLEARANCE_M:
            self.last_collision = {'clearance_mm': distance*1000, 'required_mm': 2.,
                                   'links': list(pair[:2]), 'points': pair[2].reshape(2, 3).tolist(),
                                   **self.collision_context, 'gap_ranges_mm': copy.deepcopy(ranges),
                                   'geometry_source': 'command and held-width estimate; not measured opening'}
            error = Replan(f'inter-arm clearance {distance*1000:.1f} mm at {list(pair[:2])}; required 2.0 mm; '
                           'see collision feedback')
            error.collision = copy.deepcopy(self.last_collision)
            raise error

    # --- planning ------------------------------------------------------------------------------
    def planner_target_pose(self, a, seed, p, r):
        if self.planner_kind == 'curobo':
            # Upstream: flange (link6) pose [x,y,z,qw,qx,qy,qz] in world, quaternion sign anchored to the seed FK.
            reference = rg.matrix_to_quat_wxyz(self.fk(a, seed)[1])
            return rg.flange_pose(p, r, reference_quaternion=reference)
        return (np.asarray(p, float), np.asarray(r, float))  # native IK targets the TCP site directly

    def ik(self, a, seed, p, r):
        return self.ik_candidates(a, seed, p, r)[0]

    def ik_candidates(self, a, seed, p, r):
        result = self.planners[a].solve_ik_to_joint(seed, self.planner_target_pose(a, seed, p, r),
                                                    copy.deepcopy(self.robots[a].entity_origin_pose))
        if result.get('status') != 'Success':
            raise Replan(f'{a} Cartesian segment is not reachable; add/change explicit keypoints')
        candidates = []
        for raw in result.get('joint_candidates', [result.get('joint_value')]):
            try:
                q = self.nearest_joint_equivalent(a, raw, seed)
            except Replan:
                continue
            if not any(np.max(abs(q-other)) < 1e-3 for other in candidates):
                candidates.append(q)
        if not candidates:
            raise Replan(f'{a} has no IK candidate within configured joint limits')
        return sorted(candidates, key=lambda q: float(np.linalg.norm(q-seed)))

    def nearest_joint_equivalent(self, arm, q, seed):
        return self.joint_limits[arm].nearest(q, seed)

    def plan_free(self, a, start, p, r):
        from ..controller.motion import plan_path
        return plan_path(self, a, start, p, r)

    def alternative_paths(self, a, start, p, r, check_deadline):
        from ..controller.motion import path_alternatives
        return path_alternatives(self, a, start, p, r, check_deadline)

    # --- actuation -----------------------------------------------------------------------------
    def step(self, q, g):
        if self._cancelled:
            raise Ended()
        if self.remaining_steps() == 0:
            raise StepBudgetExhausted('task_control_step_limit')
        if any(not np.isfinite(g[a]) or not 0 <= g[a] <= 1 for a in ARMS):
            raise ValueError('Invalid normalized gripper command')
        for a in ARMS:
            self.joint_limits[a].validate(q[a])
        self.actions_sent += 1
        self.cmd_g = dict(g)
        self.targets = {a: np.asarray(q[a], float).copy() for a in ARMS}
        m, d = self.model, self.data
        for a, v in self.arm.items():
            d.ctrl[v['grip']] = (1-g[a])*self.ctrl_max[a]
        kp, kd = np.asarray(self.servo['arm_kp'], float), np.asarray(self.servo['arm_kd'], float)
        warn0 = d.warning.number.copy()
        gc = self.group_compliance
        if gc is not None:   # the commanded joint target of that arm becomes a Cartesian reference followed softly
            p_d, R_d = self.fk(gc['arm'], self.targets[gc['arm']])
            self.compliance = {**gc, 'p_d': p_d, 'R_d': R_d}
        comp = self.compliance
        for _ in range(self.substeps):
            for a, v in self.arm.items():
                if comp is not None and comp['arm'] == a:
                    torque = self._impedance_torque(a, comp)
                else:
                    torque = d.qfrc_bias[v['vids']] + kp*(self.targets[a]-d.qpos[v['qids']]) - kd*d.qvel[v['vids']]
                lim = m.actuator_ctrlrange[v['act']]
                d.ctrl[v['act']] = np.clip(torque, lim[:, 0], lim[:, 1])
            mujoco.mj_step(m, d)
        if np.any(d.warning.number != warn0) or not np.isfinite(d.qpos).all():
            dw = (d.warning.number-warn0).tolist()
            # BADQPOS/BADQVEL/BADQACC: MuJoCo has already reset MjData to the model's initial state. Continuing would
            # silently show the policy a restored scene, so the episode ends here and is marked invalid.
            unstable = any(dw[int(w)] for w in (mujoco.mjtWarning.mjWARN_BADQPOS, mujoco.mjtWarning.mjWARN_BADQVEL,
                                                 mujoco.mjtWarning.mjWARN_BADQACC)) or not np.isfinite(d.qpos).all()
            self.physics_events.append({'step': self.steps, 'warnings': dw, 'finite': bool(np.isfinite(d.qpos).all()),
                                        'simulation_reset': bool(unstable)})
            if unstable:
                self.sim_invalid = True
                self._cancelled = True
                raise Ended()
        self.steps += 1
        self.record.append({'step': self.steps, 'phase': self.step_phase, 'sim_time_s': float(d.time),
                            'q_left': self.targets['left'].copy(), 'q_right': self.targets['right'].copy(),
                            'g_left': float(g['left']), 'g_right': float(g['right']),
                            'qpos': d.qpos.copy(), 'qvel': d.qvel.copy(), 'ctrl': d.ctrl.copy()})
        for hook in self.after_step_hooks:  # environment side (e.g. fastening station); not policy-visible
            hook(self.steps)
        self.on_step_acknowledged()
        if self.step_phase == 'post_planning_hold':
            self.post_hold += 1
            if self.post_hold >= self.post_hold_limit:
                self._cancelled = True  # the evaluator side closes the episode
                raise Ended()

    def save_record(self, path):
        """trajectory.npz: one row per acknowledged control step (after the step)."""
        r = self.record
        if not r:
            return
        np.savez_compressed(path, step=np.array([x['step'] for x in r]), phase=np.array([x['phase'] for x in r]),
                            sim_time_s=np.array([x['sim_time_s'] for x in r]),
                            q_target=np.stack([np.r_[x['q_left'], x['q_right']] for x in r]),
                            gripper_cmd=np.array([[x['g_left'], x['g_right']] for x in r]),
                            qpos=np.stack([x['qpos'] for x in r]), qvel=np.stack([x['qvel'] for x in r]),
                            ctrl=np.stack([x['ctrl'] for x in r]), control_dt=self.control_dt,
                            joint_names=np.array([self.model.joint(i).name for i in range(self.model.njnt)]))

    # Default gains: soft lateral, force-limited axial, firm rotation (results/compliant_probe/*).
    IMPEDANCE = dict(K_a=1500., D_a=60., F_max=5., K_l=250., D_l=25., K_r=150., D_r=1.1)

    # Group compliance (plan contract): stiff along the press direction so the push saturates at max_force_n within a
    # fraction of a millimetre of lag (with the soft K_a above the push never exceeded ~5 N whatever the limit:
    # results/cap_press_probe/grid.json); lateral and rotational gains stay those of IMPEDANCE.
    GROUP_COMPLIANCE_AXIAL = dict(K_a=20000., D_a=200.)

    def _impedance_torque(self, a, c):
        """tau = J^T [F; M] + bias - joint damping + null-space posture. Axial force clamped to F_max along c['axis'];
        no force sensing (commanded torque and joint state only)."""
        m, d, v = self.model, self.data, self.arm[a]
        site = v['site']
        mujoco.mj_jacSite(m, d, self._jacp, self._jacr, site)
        J = np.vstack([self._jacp[:, v['vids']], self._jacr[:, v['vids']]])
        vel = J @ d.qvel[v['vids']]
        p, R = d.site_xpos[site], d.site_xmat[site].reshape(3, 3)
        g, ax = c['gains'], c['axis']
        e = c['p_d'] - p
        e_a, v_a = float(e @ ax), float(vel[:3] @ ax)
        F_a = float(np.clip(g['K_a']*e_a - g['D_a']*v_a, -g['F_max'], g['F_max']))
        F = F_a*ax + g['K_l']*(e - e_a*ax) - g['D_l']*(vel[:3] - v_a*ax)
        M = g['K_r']*Rotation.from_matrix(c['R_d'] @ R.T).as_rotvec() - g['D_r']*vel[3:]
        Jp = J.T @ np.linalg.inv(J @ J.T + 1e-4*np.eye(6))
        return (J.T @ np.r_[F, M] + d.qfrc_bias[v['vids']] - 1.0*d.qvel[v['vids']]
                + (np.eye(6) - Jp @ J) @ (5.*(c['q_null'] - d.qpos[v['qids']])))

    def step_compliant(self, arm, p_d, R_d, axis, g=None, gains=None):
        """One control step with `arm` under force-limited Cartesian impedance toward (p_d, R_d); the other arm keeps
        its joint targets with the native PD. Same bookkeeping, hooks and instability handling as step()."""
        if self.compliance is None or self.compliance['arm'] != arm:
            self.compliance = {'arm': arm, 'q_null': self.joints()[arm].copy()}
        self.compliance.update(p_d=np.asarray(p_d, float), R_d=np.asarray(R_d, float),
                               axis=np.asarray(axis, float)/np.linalg.norm(axis), gains=dict(self.IMPEDANCE, **(gains or {})))
        q = {a: (self.joints()[a] if a == arm else self.targets[a]) for a in ARMS}
        try:
            self.step(q, self.gripper_commands() if g is None else g)
        finally:
            pass

    def end_compliance(self):
        self.compliance = None
        self.targets = self.joints()

    # --- group compliance (station extension of the plan contract: group['compliance']) ---------------------------
    def attach_executor(self, executor):
        """Make `executor` honour group['compliance'] and run after_group_hooks, without changing the upstream
        Executor: its _execute_group is wrapped so the backend knows which arm is compliant for the duration of
        that group and when the group has ended."""
        inner = executor._execute_group

        def execute_group(group):
            spec = group.get('compliance')
            if spec is not None:
                self.group_compliance = {'arm': spec['arm'], 'q_null': self.joints()[spec['arm']].copy(),
                                         'axis': np.asarray(spec['press_direction'], float)/np.linalg.norm(spec['press_direction']),
                                         'gains': dict(self.IMPEDANCE, **self.GROUP_COMPLIANCE_AXIAL,
                                                       F_max=float(spec['max_force_n']))}
            try:
                return inner(group)
            finally:
                if self.group_compliance is not None:
                    # Leave the arm where contact left it: the stiff hold that follows must not push on.
                    self.group_compliance = None
                    self.compliance = None
                if not self.ended():
                    for hook in self.after_group_hooks:   # e.g. the fastening station looks at the part now
                        hook(self.steps)
        executor._execute_group = execute_group
        return executor

    def hold_current(self):
        pass  # No hidden action or physics step (upstream).

    # --- observation ---------------------------------------------------------------------------
    def observation(self):
        self.observation_index += 1
        vision = self.rig.capture(self.data)
        calibration = {name: {'K': cam['intrinsic_matrix'].tolist(),
                              'camera_to_world': cam['extrinsic_matrix'].tolist(),
                              'convention': 'USD/OpenGL: +X right, +Y up, -Z forward'}
                       for name, cam in vision.items()}
        fx, fy, fz = rg.FRONT_TIP_OFFSET_LOCAL_M
        px, py, pz = rg.DISTAL_PAD_OFFSET_LOCAL_M
        states = {}
        for a in ARMS:
            p, r = self.pose(a)
            home_delta = p - self.origin_poses[a][0]
            home_angle = float(np.degrees(angle(r, self.origin_poses[a][1])))
            states[a] = {'grasp_point': p.tolist(), 'tool_axis': r[:, 0].tolist(), 'jaw_axis': r[:, 1].tolist(),
                         'gripper_pose_points': [(p-.02*r[:, 1]).tolist(), (p+.02*r[:, 1]).tolist()],
                         'gripper_pose_finger_links': list(rg.POSE_FINGER_LINKS),
                         'commanded_gripper': self.cmd_g[a],
                         'flange_point': (p-rg.GRASP_POINT_OFFSET_M*r[:, 0]).tolist(),
                         'fingertip_center': (p+fx*r[:, 0]+fy*r[:, 1]+fz*r[:, 2]).tolist(),
                         'distal_pad_bbox_center': (p+px*r[:, 0]+py*r[:, 1]+pz*r[:, 2]).tolist(),
                         'commanded_gap_mm': rg.jaw_gap_mm(self.cmd_g[a]),
                         'collision_gap_range_mm': list(self.collision_gap_range(self.cmd_g[a], self.held_width_mm[a])),
                         'held_width_estimate_mm': self.held_width_mm[a],
                         'collision_geometry_source': 'command and held-width estimate; not measured opening',
                         'gripper_state_semantics': 'command echo only; actual opening/contact/possession unavailable',
                         'home_position_error_m': float(np.linalg.norm(home_delta)),
                         'home_orientation_error_deg': home_angle,
                         'near_home': bool(np.max(np.abs(home_delta)) <= .15 and home_angle <= 20),
                         'home_grasp_point': self.origin_poses[a][0].tolist(),
                         'home_tool_axis': self.origin_poses[a][1][:, 0].tolist(),
                         'home_jaw_axis': self.origin_poses[a][1][:, 1].tolist()}
        obs = {'source_observation_index': self.observation_index, 'instruction': str(self.instruction),
               'state': states, 'calibration': calibration,
               'home_check': {'position_per_axis_m': .15, 'orientation_deg': 20,
                              'note': 'Observed initial-pose check using benchmark default tolerances; task-specific completion and official result remain authoritative.'},
               **self.budget_state(), 'execution_limits': execution_limits(),
               'sim_time_source': 'relative time from acknowledged action steps and supplied control frequency'}
        if self.station_status is not None:
            obs['station'] = self.station_status()
        depth_dir = getattr(self, 'depth_dir', None)
        if depth_dir is not None and all('depth' in cam for cam in vision.values()):
            # Depth images (a sensor the wrist and head cameras have): saved into the agent workspace, referenced here.
            depth_dir.mkdir(parents=True, exist_ok=True)
            files = {}
            for name, cam in vision.items():
                f = depth_dir/f'obs{self.observation_index:05d}_{name}.depth.npy'
                np.save(f, cam['depth'].astype(np.float32), allow_pickle=False)
                files[name] = '/work/depth/' + f.name
            obs['depth'] = {'files': files, 'units': 'm', 'shape_hw': list(next(iter(vision.values()))['depth'].shape),
                            'meaning': 'distance along each camera\'s viewing axis (OpenCV z) for every pixel; pixels with '
                                       'nothing in view hold the far-plane value (about 12 m). Back-project with the K of '
                                       'that camera (OpenCV) and convert to world with camera_to_world (OpenGL axes: flip '
                                       'y and z first). /work/tools/pose_tools.py does this and fits the CAD to it.'}
        if self.expose_objects:
            obs['objects_ground_truth'] = {
                name: {'position': self.data.body(name).xpos.tolist(), 'quaternion_wxyz': self.data.body(name).xquat.tolist()}
                for name in ('base', 'motor')}
            obs['objects_ground_truth_notice'] = 'Simulator ground truth, explicitly enabled for this run.'
        return obs, vision

    # --- lifecycle -----------------------------------------------------------------------------
    def cancel(self):
        self._cancelled = True

    def close(self):
        self.cancel()
        self.rig.close()
        if self.curobo_service is not None:
            self.curobo_service.close()

    def fail(self, error):
        self._error = error
