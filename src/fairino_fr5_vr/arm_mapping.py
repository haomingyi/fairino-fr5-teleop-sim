"""Anchored wrist mapping with workspace and per-frame limits."""
from __future__ import annotations
from dataclasses import dataclass
import math
from .protocol import HandFrame

@dataclass(frozen=True)
class ArmTarget:
    pose_mm_deg: tuple[float, float, float, float, float, float]
    anchored: bool
    clamped: bool


def _quat_multiply(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def _relative_rotvec_deg(anchor, current):
    def unit(q):
        n = math.sqrt(sum(float(v) * float(v) for v in q))
        return tuple(float(v) / n for v in q) if n > 1e-9 else (0.0, 0.0, 0.0, 1.0)
    a = unit(anchor); c = unit(current)
    q = _quat_multiply((-a[0], -a[1], -a[2], a[3]), c)
    if q[3] < 0.0: q = tuple(-v for v in q)
    nv = math.sqrt(sum(v * v for v in q[:3]))
    if nv < 1e-9: return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(nv, max(min(q[3], 1.0), -1.0))
    scale = math.degrees(angle) / nv
    return tuple(v * scale for v in q[:3])

class WristArmMapper:
    def __init__(self, cfg: dict) -> None:
        self.sign = tuple(float(x) for x in cfg.get("quest_sign", (1,-1,1)))
        self.scale = tuple(float(x) for x in cfg["translation_scale_mm_per_m"])
        self.matrix = tuple(tuple(float(x) for x in row) for row in cfg["axis_matrix"])
        self.max_step, self.workspace = float(cfg["max_translation_step_mm"]), cfg["workspace_mm"]
        self.orientation_enabled = bool(cfg.get("orientation_enabled", True))
        self.orientation_scale = tuple(float(x) for x in cfg.get("orientation_scale_deg_per_rad", (57.2958,) * 3))
        self.max_rotation_step = float(cfg.get("max_rotation_step_deg", 2.0))
        self._wrist_anchor = self._robot_anchor = self._previous = None
        self._wrist_quat_anchor = None
    def anchor(self, frame: HandFrame, robot_pose_mm_deg: tuple[float, ...]) -> None:
        if len(robot_pose_mm_deg) != 6: raise ValueError("robot pose must contain six values")
        self._wrist_anchor, self._robot_anchor = tuple(frame.wrist_position), tuple(float(x) for x in robot_pose_mm_deg)
        self._wrist_quat_anchor = tuple(frame.wrist_quaternion_xyzw)
        self._previous = self._robot_anchor
    def map(self, frame: HandFrame) -> ArmTarget | None:
        if self._wrist_anchor is None or self._robot_anchor is None: return None
        delta = tuple((now-zero)*sign for now,zero,sign in zip(frame.wrist_position,self._wrist_anchor,self.sign))
        scaled = tuple(delta[i]*self.scale[i] for i in range(3))
        robot_delta = tuple(sum(self.matrix[row][col]*scaled[col] for col in range(3)) for row in range(3))
        desired, previous = list(self._robot_anchor), self._previous or self._robot_anchor
        for i in range(3):
            desired[i] += robot_delta[i]
            desired[i] = max(previous[i]-self.max_step, min(previous[i]+self.max_step, desired[i]))
        clamped = False
        for i, axis in enumerate(("x","y","z")):
            lo, hi = (float(x) for x in self.workspace[axis]); bounded = max(lo, min(hi, desired[i]))
            clamped = clamped or bounded != desired[i]; desired[i] = bounded
        if self.orientation_enabled and self._wrist_quat_anchor is not None:
            wrist_rot = _relative_rotvec_deg(self._wrist_quat_anchor, frame.wrist_quaternion_xyzw)
            scaled_rot = tuple(wrist_rot[i] * self.sign[i] * self.orientation_scale[i] / 57.2958 for i in range(3))
            robot_rot = tuple(sum(self.matrix[row][col] * scaled_rot[col] for col in range(3)) for row in range(3))
            for i in range(3):
                desired[i + 3] += robot_rot[i]
                desired[i + 3] = max(previous[i + 3] - self.max_rotation_step,
                                      min(previous[i + 3] + self.max_rotation_step, desired[i + 3]))
        self._previous = tuple(desired)
        return ArmTarget(tuple(desired), True, clamped)
