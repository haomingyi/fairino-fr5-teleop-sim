"""Geometry baseline from 21 OpenXR landmarks to six IH01 channels."""
from __future__ import annotations
from dataclasses import dataclass
import math
from .protocol import HandFrame, Point

SEMANTICS = ("pinky_flex", "ring_flex", "middle_flex", "index_flex", "thumb_flex", "thumb_opposition")
def _sub(a: Point, b: Point) -> Point: return tuple(x - y for x, y in zip(a, b))
def _norm(v: Point) -> float: return math.sqrt(sum(x * x for x in v))
def _distance(a: Point, b: Point) -> float: return _norm(_sub(a, b))
def _angle(a: Point, b: Point, c: Point) -> float:
    left, right = _sub(a, b), _sub(c, b); denominator = _norm(left) * _norm(right)
    if denominator < 1e-9: return math.pi
    cosine = sum(x * y for x, y in zip(left, right)) / denominator
    return math.acos(max(-1.0, min(1.0, cosine)))
def _curl(points: tuple[Point, ...], indices: tuple[int, int, int, int]) -> float:
    a, b, c, d = (points[i] for i in indices)
    bend = (math.pi - _angle(a, b, c)) + (math.pi - _angle(b, c, d))
    return max(0.0, min(1.0, bend / math.radians(210.0)))

@dataclass(frozen=True)
class HandTarget:
    normalized: tuple[float, ...]
    steps: tuple[int, ...]
    quality: float

class IH01Mapper:
    def __init__(self, channel_max: list[int], max_delta: int = 81) -> None:
        if len(channel_max) != 6: raise ValueError("IH01 needs six channel maxima")
        self.channel_max, self.max_delta = tuple(int(x) for x in channel_max), max(1, int(max_delta))
        self._previous: tuple[int, ...] | None = None
    def map(self, frame: HandFrame) -> HandTarget:
        p = frame.landmarks
        if len(p) != 21: raise ValueError(f"expected 21 landmarks, got {len(p)}")
        values = (_curl(p, (17,18,19,20)), _curl(p, (13,14,15,16)), _curl(p, (9,10,11,12)), _curl(p, (5,6,7,8)), _curl(p, (1,2,3,4)))
        palm_width = _distance(p[5], p[17])
        opposition = 0.0 if palm_width < 1e-6 else max(0.0, min(1.0, (1.45 - _distance(p[4], p[5]) / palm_width) / 1.10))
        lengths = tuple(_distance(p[a], p[b]) for a, b in ((5,8),(9,12),(13,16),(17,20)))
        quality = 0.0 if max(lengths, default=0.0) < 1e-6 else max(0.0, min(1.0, min(lengths) / max(lengths) / 0.55))
        normalized = values + (opposition,)
        desired = tuple(round(value * maximum) for value, maximum in zip(normalized, self.channel_max))
        if self._previous is None: limited = desired
        else: limited = tuple(max(old-self.max_delta, min(old+self.max_delta, new)) for old,new in zip(self._previous,desired))
        self._previous = limited
        return HandTarget(normalized, limited, quality)
