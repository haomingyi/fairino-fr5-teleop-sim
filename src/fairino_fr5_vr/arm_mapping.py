"""Anchored wrist mapping with workspace and per-frame limits."""
from __future__ import annotations
from dataclasses import dataclass
from .protocol import HandFrame

@dataclass(frozen=True)
class ArmTarget:
    pose_mm_deg: tuple[float, float, float, float, float, float]
    anchored: bool
    clamped: bool

class WristArmMapper:
    def __init__(self, cfg: dict) -> None:
        self.sign = tuple(float(x) for x in cfg.get("quest_sign", (1,-1,1)))
        self.scale = tuple(float(x) for x in cfg["translation_scale_mm_per_m"])
        self.matrix = tuple(tuple(float(x) for x in row) for row in cfg["axis_matrix"])
        self.max_step, self.workspace = float(cfg["max_translation_step_mm"]), cfg["workspace_mm"]
        self._wrist_anchor = self._robot_anchor = self._previous = None
    def anchor(self, frame: HandFrame, robot_pose_mm_deg: tuple[float, ...]) -> None:
        if len(robot_pose_mm_deg) != 6: raise ValueError("robot pose must contain six values")
        self._wrist_anchor, self._robot_anchor = tuple(frame.wrist_position), tuple(float(x) for x in robot_pose_mm_deg)
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
        self._previous = tuple(desired)
        return ArmTarget(tuple(desired), True, clamped)
