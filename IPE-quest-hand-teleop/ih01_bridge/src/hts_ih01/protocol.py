"""Parser for the official Hand Tracking Streamer CSV protocol.

Author: haoming
"""

from __future__ import annotations

from dataclasses import dataclass
import time

Point = tuple[float, float, float]


@dataclass(frozen=True)
class ParsedLine:
    side: str
    kind: str
    values: tuple[float, ...]


@dataclass(frozen=True)
class HandFrame:
    side: str
    wrist_position: Point
    wrist_quaternion_xyzw: tuple[float, float, float, float]
    landmarks: tuple[Point, ...]
    timestamp_s: float


def parse_hts_line(line: str) -> ParsedLine | None:
    """Parse one HTS `Left/Right wrist/landmarks` CSV line."""
    parts = [part.strip() for part in line.strip().split(",")]
    if not parts or not parts[0]:
        return None
    label = parts[0].lower()
    if "wrist" not in label and "landmarks" not in label:
        return None
    side = "left" if "left" in label else "right" if "right" in label else ""
    if not side:
        return None
    kind = "wrist" if "wrist" in label else "landmarks"
    try:
        values = tuple(float(value) for value in parts[1:] if value)
    except ValueError as exc:
        raise ValueError(f"invalid numeric value in HTS {kind} line") from exc
    expected = 7 if kind == "wrist" else 63
    if len(values) != expected:
        raise ValueError(f"HTS {side} {kind} needs {expected} floats, got {len(values)}")
    return ParsedLine(side=side, kind=kind, values=values)


class HandStreamAssembler:
    """Combine separate wrist and landmark lines into complete hand frames."""

    def __init__(self) -> None:
        self._wrists: dict[str, tuple[float, ...]] = {}

    def feed(self, line: str, *, now: float | None = None) -> HandFrame | None:
        parsed = parse_hts_line(line)
        if parsed is None:
            return None
        if parsed.kind == "wrist":
            self._wrists[parsed.side] = parsed.values
            return None
        wrist = self._wrists.get(parsed.side)
        if wrist is None:
            return None
        points = tuple(
            (parsed.values[index], parsed.values[index + 1], parsed.values[index + 2])
            for index in range(0, 63, 3)
        )
        return HandFrame(
            side=parsed.side,
            wrist_position=(wrist[0], wrist[1], wrist[2]),
            wrist_quaternion_xyzw=(wrist[3], wrist[4], wrist[5], wrist[6]),
            landmarks=points,
            timestamp_s=time.monotonic() if now is None else now,
        )
