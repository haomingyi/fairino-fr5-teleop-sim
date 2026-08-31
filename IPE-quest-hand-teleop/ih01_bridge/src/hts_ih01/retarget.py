"""Geometry baseline mapping HTS landmarks to the six active IH01 channels.

Author: haoming
"""

from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import json
import math
from pathlib import Path

from .protocol import HandFrame, Point

SEMANTICS = (
    "pinky_flex",
    "ring_flex",
    "middle_flex",
    "index_flex",
    "thumb_flex",
    "thumb_opposition",
)
STEP_MAX = (1700, 1700, 1700, 1700, 1700, 1300)
# Quest's tracked PIP/DIP angles rarely reach the theoretical 210 degree
# span used by the old mapper.  Use a human-fist span and saturate the last
# part so a clearly closed finger reaches the IH01 mechanical end point.
CURL_FULL_BEND_RAD = math.radians(180.0)
CURL_SATURATION = 0.88


def _sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm(value: Point) -> float:
    return math.sqrt(sum(component * component for component in value))


def _distance(a: Point, b: Point) -> float:
    return _norm(_sub(a, b))


def _angle(a: Point, b: Point, c: Point) -> float:
    left, right = _sub(a, b), _sub(c, b)
    denominator = _norm(left) * _norm(right)
    if denominator < 1e-9:
        return math.pi
    cosine = sum(x * y for x, y in zip(left, right, strict=True)) / denominator
    return math.acos(max(-1.0, min(1.0, cosine)))


def _clip01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _curl(points: tuple[Point, ...], indices: tuple[int, int, int, int]) -> float:
    a, b, c, d = (points[index] for index in indices)
    bend = (math.pi - _angle(a, b, c)) + (math.pi - _angle(b, c, d))
    normalized = _clip01(bend / CURL_FULL_BEND_RAD)
    if normalized <= 0.025:
        return 0.0
    if normalized >= CURL_SATURATION:
        return 1.0
    return normalized / CURL_SATURATION


def _endpoint(value: float, *, closed: float = 0.88) -> float:
    """Keep an open-hand deadband and make an unambiguous close reach 1.0."""
    value = _clip01(value)
    if value <= 0.025:
        return 0.0
    if value >= closed:
        return 1.0
    return value / closed


def _finger_flex(points: tuple[Point, ...], indices: tuple[int, int, int, int]) -> float:
    """Fuse joint bend with fingertip contraction for occlusion-tolerant flex."""
    a, b, c, d = (points[index] for index in indices)
    joint_flex = _curl(points, indices)
    path = _distance(a, b) + _distance(b, c) + _distance(c, d)
    if path < 1e-9:
        return joint_flex
    # A straight finger has reach/path close to one.  A curled fingertip moves
    # back toward its base, so the ratio falls without depending on hand size.
    reach_ratio = _distance(a, d) / path
    tip_contraction = _clip01((0.96 - reach_ratio) / 0.54)
    return _endpoint(0.68 * joint_flex + 0.32 * tip_contraction, closed=0.84)


def _thumb_flex(points: tuple[Point, ...], palm_width: float) -> float:
    """Include thumb-to-palm closure, which dominates a natural fist."""
    joint_flex = _curl(points, (1, 2, 3, 4))
    if palm_width < 1e-9:
        return joint_flex
    palm_center = tuple(
        sum(points[index][axis] for index in (5, 9, 13, 17)) / 4.0
        for axis in range(3)
    )
    tip_to_palm = _distance(points[4], palm_center) / palm_width
    palm_closure = _clip01((1.15 - tip_to_palm) / 0.65)
    return _endpoint(0.35 * joint_flex + 0.65 * palm_closure, closed=0.74)


@dataclass(frozen=True)
class RetargetResult:
    normalized: tuple[float, float, float, float, float, float]
    steps: tuple[int, int, int, int, int, int]
    quality: float


class InitialRetargeter:
    """Restore the prior HTS geometry mapping without touching V1/V2."""

    def __init__(self, calibration_path: Path | None = None) -> None:
        self._history: dict[str, list[deque[float]]] = {}
        self._last_frame_s: dict[str, float] = {}
        self._recent_raw: dict[str, deque[tuple[float, ...]]] = {}
        self._calibration_path = calibration_path
        self._calibration: dict[str, dict[str, tuple[float, ...]]] = {}
        self._load_calibration()

    def _load_calibration(self) -> None:
        if self._calibration_path is None or not self._calibration_path.exists():
            return
        try:
            payload = json.loads(self._calibration_path.read_text(encoding="utf-8"))
            if payload.get("format") != "ih01_quest_personal_calibration_v1":
                return
            for side in ("left", "right"):
                entry = payload.get("hands", {}).get(side, {})
                poses = {
                    kind: tuple(float(value) for value in entry[kind])
                    for kind in ("open", "fist") if len(entry.get(kind, [])) == 6
                }
                if poses:
                    self._calibration[side] = poses
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            self._calibration = {}

    def _save_calibration(self) -> None:
        if self._calibration_path is None:
            return
        self._calibration_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "format": "ih01_quest_personal_calibration_v1",
            "hands": {
                side: {kind: list(values) for kind, values in poses.items()}
                for side, poses in self._calibration.items()
            },
        }
        self._calibration_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    def capture_pose(self, kind: str, sides: tuple[str, ...]) -> str:
        """Capture a robust recent median for an open hand or natural fist."""
        if kind not in ("open", "fist"):
            raise ValueError("calibration kind must be open or fist")
        captured: list[str] = []
        for side in sides:
            samples = list(self._recent_raw.get(side, ()))
            if len(samples) < 5:
                continue
            medians = tuple(
                sorted(sample[channel] for sample in samples)[len(samples) // 2]
                for channel in range(6)
            )
            self._calibration.setdefault(side, {})[kind] = medians
            self._history.pop(side, None)
            captured.append(side)
        if not captured:
            return "calibration failed: hold a tracked hand steady first"
        self._save_calibration()
        if kind == "fist":
            counts = ",".join(
                f"{side}={self._valid_channel_count(side)}/6" for side in captured
            )
            return f"captured fist: {counts} calibrated channels"
        return f"captured open: {','.join(captured)}"

    def clear_calibration(self, sides: tuple[str, ...]) -> str:
        cleared = [side for side in sides if self._calibration.pop(side, None) is not None]
        for side in sides:
            self._history.pop(side, None)
        self._save_calibration()
        return f"cleared calibration: {','.join(cleared)}" if cleared else "no calibration to clear"

    def calibration_status(self, side: str) -> str:
        sides = ("left", "right") if side == "both" else (side,)
        labels = []
        for name in sides:
            poses = self._calibration.get(name, {})
            count = self._valid_channel_count(name)
            labels.append(f"{name[0].upper()}:{count}/6" if count else f"{name[0].upper()}:FIXED")
        return "calibration " + " ".join(labels)

    def _valid_channel_count(self, side: str) -> int:
        poses = self._calibration.get(side, {})
        if "open" not in poses or "fist" not in poses:
            return 0
        return sum(
            closed - opened >= 0.08
            for opened, closed in zip(poses["open"], poses["fist"], strict=True)
        )

    def _apply_calibration(self, side: str, values: tuple[float, ...]) -> tuple[float, ...]:
        poses = self._calibration.get(side, {})
        if "open" not in poses or "fist" not in poses:
            return values
        result = []
        for value, opened, closed in zip(values, poses["open"], poses["fist"], strict=True):
            span = closed - opened
            if span < 0.08:
                result.append(value)
                continue
            normalized = _clip01((value - opened) / span)
            result.append(0.0 if normalized <= 0.03 else 1.0 if normalized >= 0.95 else normalized)
        return tuple(result)

    def _stabilize(self, side: str, values: tuple[float, ...], timestamp_s: float) -> tuple[float, ...]:
        """Reject a one-frame landmark spike with only one frame of latency."""
        if timestamp_s - self._last_frame_s.get(side, timestamp_s) > 0.20:
            self._history.pop(side, None)
        self._last_frame_s[side] = timestamp_s
        histories = self._history.setdefault(
            side, [deque(maxlen=3) for _ in values]
        )
        result: list[float] = []
        for history, value in zip(histories, values, strict=True):
            history.append(float(value))
            result.append(
                float(sorted(history)[len(history) // 2])
                if len(history) == 3 else float(value)
            )
        return tuple(result)

    def map(self, frame: HandFrame) -> RetargetResult:
        points = frame.landmarks
        if len(points) != 21:
            raise ValueError(f"expected 21 landmarks, got {len(points)}")
        index = _finger_flex(points, (5, 6, 7, 8))
        middle = _finger_flex(points, (9, 10, 11, 12))
        ring = _finger_flex(points, (13, 14, 15, 16))
        pinky = _finger_flex(points, (17, 18, 19, 20))
        palm_width = _distance(points[5], points[17])
        thumb_flex = _thumb_flex(points, palm_width)
        if palm_width < 1e-6:
            opposition, quality = 0.0, 0.0
        else:
            opposition = _clip01((1.45 - _distance(points[4], points[5]) / palm_width) / 1.10)
            if opposition <= 0.025:
                opposition = 0.0
            elif opposition >= 0.88:
                opposition = 1.0
            lengths = tuple(
                _distance(points[start], points[end])
                for start, end in ((5, 8), (9, 12), (13, 16), (17, 20))
            )
            quality = _clip01(min(lengths) / max(max(lengths), 1e-6) / 0.55)
        raw = (pinky, ring, middle, index, thumb_flex, opposition)
        self._recent_raw.setdefault(frame.side, deque(maxlen=20)).append(raw)
        normalized = self._stabilize(
            frame.side,
            self._apply_calibration(frame.side, raw),
            frame.timestamp_s,
        )
        steps = tuple(round(value * maximum) for value, maximum in zip(normalized, STEP_MAX, strict=True))
        return RetargetResult(normalized=normalized, steps=steps, quality=quality)
