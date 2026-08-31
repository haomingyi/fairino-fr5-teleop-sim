"""Parser for Hand Tracking Streamer wrist and landmark CSV lines."""
from __future__ import annotations
from dataclasses import asdict, dataclass
import time

Point = tuple[float, float, float]

@dataclass(frozen=True)
class HandFrame:
    side: str
    wrist_position: Point
    wrist_quaternion_xyzw: tuple[float, float, float, float]
    landmarks: tuple[Point, ...]
    timestamp_s: float
    def to_dict(self) -> dict:
        return asdict(self)
    @classmethod
    def from_dict(cls, value: dict) -> "HandFrame":
        return cls(str(value["side"]).lower(), tuple(float(x) for x in value["wrist_position"]), tuple(float(x) for x in value["wrist_quaternion_xyzw"]), tuple(tuple(float(x) for x in p) for p in value["landmarks"]), float(value["timestamp_s"]))

@dataclass(frozen=True)
class ParsedLine:
    side: str
    kind: str
    values: tuple[float, ...]

def parse_hts_line(line: str) -> ParsedLine | None:
    parts = [part.strip() for part in line.strip().split(",")]
    if not parts or not parts[0]: return None
    label = parts[0].lower()
    kind = "wrist" if "wrist" in label else "landmarks" if "landmarks" in label else ""
    side = "left" if "left" in label else "right" if "right" in label else ""
    if not kind or not side: return None
    try: values = tuple(float(value) for value in parts[1:] if value)
    except ValueError as exc: raise ValueError(f"invalid numeric value in HTS {kind} line") from exc
    expected = 7 if kind == "wrist" else 63
    if len(values) != expected: raise ValueError(f"HTS {side} {kind} needs {expected} floats, got {len(values)}")
    return ParsedLine(side, kind, values)

class HandStreamAssembler:
    def __init__(self, pair_timeout_s: float = 0.2) -> None:
        self.pair_timeout_s = float(pair_timeout_s)
        self._wrists: dict[str, tuple[tuple[float, ...], float]] = {}
    def feed(self, line: str, *, now: float | None = None) -> HandFrame | None:
        parsed = parse_hts_line(line)
        if parsed is None: return None
        stamp = time.monotonic() if now is None else float(now)
        if parsed.kind == "wrist":
            self._wrists[parsed.side] = (parsed.values, stamp); return None
        pair = self._wrists.get(parsed.side)
        if pair is None or stamp - pair[1] > self.pair_timeout_s: return None
        wrist = pair[0]
        points = tuple(tuple(parsed.values[i:i + 3]) for i in range(0, 63, 3))
        return HandFrame(parsed.side, tuple(wrist[:3]), tuple(wrist[3:7]), points, stamp)
