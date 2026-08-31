"""Geometry baseline mapping HTS landmarks to the six active IH01 channels.

Author: haoming
"""

from __future__ import annotations

from dataclasses import dataclass
import math

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


@dataclass(frozen=True)
class RetargetResult:
    normalized: tuple[float, float, float, float, float, float]
    steps: tuple[int, int, int, int, int, int]
    quality: float


class InitialRetargeter:
    """Restore the prior HTS geometry mapping without touching V1/V2."""

    def map(self, frame: HandFrame) -> RetargetResult:
        points = frame.landmarks
        if len(points) != 21:
            raise ValueError(f"expected 21 landmarks, got {len(points)}")
        index = _curl(points, (5, 6, 7, 8))
        middle = _curl(points, (9, 10, 11, 12))
        ring = _curl(points, (13, 14, 15, 16))
        pinky = _curl(points, (17, 18, 19, 20))
        thumb_flex = _curl(points, (1, 2, 3, 4))
        palm_width = _distance(points[5], points[17])
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
        normalized = (pinky, ring, middle, index, thumb_flex, opposition)
        steps = tuple(round(value * maximum) for value, maximum in zip(normalized, STEP_MAX, strict=True))
        return RetargetResult(normalized=normalized, steps=steps, quality=quality)
