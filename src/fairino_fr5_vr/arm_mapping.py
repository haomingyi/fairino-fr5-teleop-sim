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
    # Root poses are expressed in the Quest tracking space. Compose the
    # current orientation with the inverse anchor on the right to obtain a
    # world/tracking-frame delta; inverse(anchor) * current would express the
    # delta in the initially rotated hand-local frame.
    q = _quat_multiply(c, (-a[0], -a[1], -a[2], a[3]))
    if q[3] < 0.0: q = tuple(-v for v in q)
    nv = math.sqrt(sum(v * v for v in q[:3]))
    if nv < 1e-9: return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(nv, max(min(q[3], 1.0), -1.0))
    scale = math.degrees(angle) / nv
    return tuple(v * scale for v in q[:3])


def _determinant3(matrix) -> float:
    a, b, c = matrix
    return (a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0]))

class WristArmMapper:
    def __init__(self, cfg: dict) -> None:
        self.sign = tuple(float(x) for x in cfg.get("quest_sign", (1,-1,1)))
        self.scale = tuple(float(x) for x in cfg["translation_scale_mm_per_m"])
        self.matrix = tuple(tuple(float(x) for x in row) for row in cfg["axis_matrix"])
        position_matrix = tuple(
            tuple(self.matrix[row][col] * self.sign[col] for col in range(3))
            for row in range(3)
        )
        # A rotation vector is an axial vector. Under a reflected coordinate
        # transform it gains an extra determinant sign compared with position.
        self.rotation_handedness = -1.0 if _determinant3(position_matrix) < 0.0 else 1.0
        self.orientation_axis_sign = tuple(float(x) for x in cfg.get(
            "orientation_axis_sign", (1.0, 1.0, 1.0)))
        self.max_step, self.workspace = float(cfg["max_translation_step_mm"]), cfg["workspace_mm"]
        self.max_speed = float(cfg.get("max_translation_speed_mm_s", math.inf))
        self.orientation_enabled = bool(cfg.get("orientation_enabled", True))
        self.orientation_scale = tuple(float(x) for x in cfg.get("orientation_scale_deg_per_rad", (57.2958,) * 3))
        self.max_rotation_step = float(cfg.get("max_rotation_step_deg", 2.0))
        self.max_rotation_speed = float(cfg.get("max_rotation_speed_deg_s", math.inf))
        rotation_range = cfg.get("rotation_range_deg", (45.0, 45.0, 45.0))
        self.rotation_range = tuple(abs(float(value)) for value in rotation_range)
        self._wrist_anchor = self._robot_anchor = self._previous = None
        self._wrist_quat_anchor = None
        self._previous_timestamp = None
    def anchor(self, frame: HandFrame, robot_pose_mm_deg: tuple[float, ...]) -> None:
        if len(robot_pose_mm_deg) != 6: raise ValueError("robot pose must contain six values")
        self._wrist_anchor, self._robot_anchor = tuple(frame.wrist_position), tuple(float(x) for x in robot_pose_mm_deg)
        self._wrist_quat_anchor = tuple(frame.wrist_quaternion_xyzw)
        self._previous = self._robot_anchor
        self._previous_timestamp = float(frame.timestamp_s)
    def map(self, frame: HandFrame) -> ArmTarget | None:
        if self._wrist_anchor is None or self._robot_anchor is None: return None
        delta = tuple((now-zero)*sign for now,zero,sign in zip(frame.wrist_position,self._wrist_anchor,self.sign))
        scaled = tuple(delta[i]*self.scale[i] for i in range(3))
        robot_delta = tuple(sum(self.matrix[row][col]*scaled[col] for col in range(3)) for row in range(3))
        # The Quest delta is measured from the clutch anchor, so construct an
        # absolute target from the anchor on every frame.  Accumulating that
        # already-absolute delta into ``desired`` causes drift/floating and
        # exaggerated wrist rotation after a few frames.
        desired, previous = list(self._robot_anchor), self._previous or self._robot_anchor
        now = float(frame.timestamp_s)
        dt = max(1.0 / 120.0, min(0.1, now - self._previous_timestamp)) if self._previous_timestamp is not None else 1.0 / 30.0
        translation_step = min(self.max_step, self.max_speed * dt)
        for i in range(3):
            target_value = self._robot_anchor[i] + robot_delta[i]
            desired[i] = max(previous[i] - translation_step,
                             min(previous[i] + translation_step, target_value))
        clamped = False
        for i, axis in enumerate(("x","y","z")):
            lo, hi = (float(x) for x in self.workspace[axis]); bounded = max(lo, min(hi, desired[i]))
            clamped = clamped or bounded != desired[i]; desired[i] = bounded
        if self.orientation_enabled and self._wrist_quat_anchor is not None:
            wrist_rot = _relative_rotvec_deg(self._wrist_quat_anchor, frame.wrist_quaternion_xyzw)
            scaled_rot = tuple(wrist_rot[i] * self.sign[i] * self.orientation_scale[i] / 57.2958 for i in range(3))
            robot_rot = tuple(
                self.rotation_handedness * self.orientation_axis_sign[row] *
                sum(self.matrix[row][col] * scaled_rot[col] for col in range(3))
                for row in range(3)
            )
            rotation_step = min(self.max_rotation_step, self.max_rotation_speed * dt)
            for i in range(3):
                target_value = self._robot_anchor[i + 3] + robot_rot[i]
                desired[i + 3] = max(previous[i + 3] - rotation_step,
                                      min(previous[i + 3] + rotation_step, target_value))
                # Keep wrist roll/pitch/yaw within a bounded envelope around
                # the operator's initial orientation; this removes large
                # Euler excursions caused by brief quaternion flips.
                anchor_value = self._robot_anchor[i + 3]
                desired[i + 3] = max(anchor_value - self.rotation_range[i],
                                      min(anchor_value + self.rotation_range[i], desired[i + 3]))
        self._previous = tuple(desired)
        self._previous_timestamp = now
        return ArmTarget(tuple(desired), True, clamped)
