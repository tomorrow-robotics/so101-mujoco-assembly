"""CameraIO for the three Intel RealSense cameras of the lab (pyrealsense2).

Serial numbers default to the cameras the twin was calibrated with (head 261822072968, left wrist D405
262422271868, right wrist D405 262422271416); pass others with --camera-args '{"serials": {...}}'.
Colour 640x480 RGB, depth aligned to colour, metres, 0 = no return. K is read from the device for the colour
stream. Extrinsics come from the twin (see real/interface.py), unless you add 'camera_to_world' yourself.
"""
from __future__ import annotations

import numpy as np

from .interface import CameraIO

SERIALS = {'cam_head': '261822072968', 'cam_left_wrist': '262422271868', 'cam_right_wrist': '262422271416'}


class RealSenseCameras(CameraIO):
    def __init__(self, serials=None, width=640, height=480, fps=30, warmup_frames=15, timeout_ms=5000):
        import pyrealsense2 as rs
        self.rs, self.timeout_ms = rs, int(timeout_ms)
        self.serials = dict(SERIALS if serials is None else serials)
        connected = {d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()}
        missing = {k: s for k, s in self.serials.items() if s not in connected}
        if missing:
            raise RuntimeError(f'RealSense cameras not connected: {missing}; connected: {sorted(connected)}')
        self.pipes, self.align, self.scale, self.K = {}, rs.align(rs.stream.color), {}, {}
        for name, serial in self.serials.items():
            cfg = rs.config()
            cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
            cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            pipe = rs.pipeline()
            prof = pipe.start(cfg)
            self.scale[name] = prof.get_device().first_depth_sensor().get_depth_scale()
            i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.K[name] = np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1.]])
            self.pipes[name] = pipe
        for _ in range(int(warmup_frames)):      # auto exposure settles
            for pipe in self.pipes.values():
                pipe.wait_for_frames(self.timeout_ms)

    def capture(self):
        out = {}
        for name, pipe in self.pipes.items():
            frames = self.align.process(pipe.wait_for_frames(self.timeout_ms))
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            if not color or not depth:
                raise RuntimeError(f'{name}: no frame')
            out[name] = {'color': np.asanyarray(color.get_data()).copy(),
                         'depth': np.asanyarray(depth.get_data()).astype(np.float32)*self.scale[name],
                         'K': self.K[name].copy()}
        return out

    def close(self):
        for pipe in self.pipes.values():
            try:
                pipe.stop()
            except Exception:  # noqa: BLE001
                pass
