"""Render the Q0 cameras on the GPU pod (NVIDIA EGL) for a simulation running on the devbox.

One devbox-initiated ssh connection runs `python scripts/q0_render_server.py <scene>` on the pod. Per capture the
devbox sends the current qpos; the pod sets it on its own copy of the same scene, runs mj_forward and renders the
three cameras with the same CameraRig (housing hiding included), and returns lossless uint8 RGB. Intrinsics and
camera-to-world matrices are computed on the devbox from its own MjData, i.e. for exactly the physics state that
was rendered. No port is opened; nothing goes through HDFS.
"""
import hashlib
import json
import os
import subprocess
from pathlib import Path

import numpy as np

from .mujoco_cameras import DEFAULT_CAMERAS, camera_to_world, intrinsics

# Remote GPU host settings (simulation only; not used on real hardware). Set these in the environment.
POD_ROOT = os.environ.get('POD_ROOT', '')              # repo path on the GPU host (same absolute path)
ENDPOINTS = Path(os.environ.get('POD_ENDPOINTS', ''))  # dir with one '<host> <port>' file per job
POD_USER = os.environ.get('POD_USER', '')
POD_SSH_KEY = os.environ.get('POD_SSH_KEY', str(Path.home()/'.ssh/id_rsa'))
POD_KNOWN_HOSTS = os.environ.get('POD_KNOWN_HOSTS', str(Path.home()/'.ssh/known_hosts'))
EGL_ENV_SCRIPT = os.environ.get('EGL_ENV_SCRIPT', '')  # script that sets up NVIDIA EGL for a GPU index


def ssh_command(job):
    host, port = (ENDPOINTS/job).read_text().split()[:2]
    return ['ssh', '-6', '-p', port, '-i', POD_SSH_KEY, '-o', 'BatchMode=yes',
            '-o', 'IdentitiesOnly=yes', '-o', f'UserKnownHostsFile={POD_KNOWN_HOSTS}',
            '-o', 'ServerAliveInterval=15', f'{POD_USER}@{host}', '--']


class PodCameraRig:
    def __init__(self, model, scene, job=None, cameras=None, width=640, height=480, gpu=0, depth=None):
        self.model, self.width, self.height = model, int(width), int(height)
        self.cameras = dict(DEFAULT_CAMERAS if cameras is None else cameras)
        self.ids = {k: model.camera(v).id for k, v in self.cameras.items()}
        scene = Path(scene).resolve()
        self.scene_sha256 = hashlib.sha256(scene.read_bytes()).hexdigest()
        job = job or os.environ['POD_JOB']
        gpu = int(os.environ.get('Q0_RENDER_GPU', 7 if gpu == 0 else gpu))   # the pod GPU that renders (the others may carry keepers)
        self.gpu = gpu
        self.depth = bool(int(os.environ.get('Q0_RENDER_DEPTH', '1'))) if depth is None else bool(depth)
        remote = (f'cd {POD_ROOT} && source {EGL_ENV_SCRIPT} {gpu} >/dev/null 2>&1 && '
                  f'export PYTHONPATH=/tmp/so101_pydeps/pydeps:{POD_ROOT}:{POD_ROOT}/scripts LD_LIBRARY_PATH=/tmp/so101_pydeps/libs:$LD_LIBRARY_PATH && '
                  f'exec python -u scripts/q0_render_server.py {scene} {self.width} {self.height} '
                  f"'{json.dumps(self.cameras)}'" + (' depth' if self.depth else ''))
        self.proc = subprocess.Popen(ssh_command(job)+[remote], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        hello = json.loads(self.proc.stdout.readline())
        if hello.get('scene_sha256') != self.scene_sha256:
            raise RuntimeError(f'pod scene differs from devbox scene: {hello}')
        if 'NVIDIA' not in hello.get('gl_renderer', ''):
            raise RuntimeError(f'pod renderer is not NVIDIA: {hello}')
        self.info = hello
        self.backend = 'pod:' + hello['gl_renderer']

    def capture(self, data):
        q = np.ascontiguousarray(data.qpos, dtype=np.float64).tobytes()
        self.proc.stdin.write(len(q).to_bytes(8, 'little') + q)
        self.proc.stdin.flush()
        vision = {}
        n = self.width*self.height*3
        for name, i in self.ids.items():
            buf = bytearray()
            while len(buf) < n:
                chunk = self.proc.stdout.read(n-len(buf))
                if not chunk:
                    raise RuntimeError('pod render stream closed')
                buf += chunk
            rgb = np.frombuffer(bytes(buf), np.uint8).reshape(self.height, self.width, 3).copy()
            vision[name] = {'color': rgb, 'intrinsic_matrix': intrinsics(self.model, i, self.width, self.height),
                            'extrinsic_matrix': camera_to_world(data, i)}
            if self.depth:
                m = self.width*self.height*4
                dbuf = bytearray()
                while len(dbuf) < m:
                    chunk = self.proc.stdout.read(m-len(dbuf))
                    if not chunk:
                        raise RuntimeError('pod render stream closed')
                    dbuf += chunk
                vision[name]['depth'] = np.frombuffer(bytes(dbuf), np.float32).reshape(self.height, self.width).copy()
        return vision

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=30)
        except Exception:
            self.proc.kill()
