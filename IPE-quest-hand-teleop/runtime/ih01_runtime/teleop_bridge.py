"""Stateful bridge from visual hand targets to the persistent IH01 backend.

Author: haoming
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np

from .hardware_control import CHANNEL_LIMITS, ConsoleState

VISION_TO_CHANNEL = (
    "pinky_flex",
    "ring_flex",
    "middle_flex",
    "index_flex",
    "thumb_flex",
    "thumb_opposition",
)


class CommandSink(Protocol):
    def send(self, command: str) -> None: ...


def set_hardware_speed(backend: CommandSink, value: int) -> int:
    """Stage a vendor speed update and return the validated steps/s value."""
    speed = int(np.clip(int(value), 1, 2000))
    backend.send(f"SPEED {speed}")
    return speed


@dataclass
class TeleopConfig:
    command_rate_hz: float = 50.0
    loss_timeout_s: float = 0.35
    max_delta_steps: int = 24
    speed_steps_s: int = 2000
    mapping: dict[str, Any] | None = None
    open_steps: dict[str, tuple[int, ...]] = field(
        default_factory=lambda: {
            "left": (0, 0, 0, 0, 0, 0),
            "right": (0, 0, 0, 0, 0, 0),
        }
    )
    closed_steps: dict[str, tuple[int, ...]] = field(
        default_factory=lambda: {
            "left": CHANNEL_LIMITS,
            "right": CHANNEL_LIMITS,
        }
    )

    @classmethod
    def from_mapping(cls, payload: dict[str, Any]) -> TeleopConfig:
        defaults = cls()
        return cls(
            command_rate_hz=float(payload.get("command_rate_hz", defaults.command_rate_hz)),
            loss_timeout_s=float(payload.get("loss_timeout_s", defaults.loss_timeout_s)),
            max_delta_steps=int(
                payload.get("max_delta_steps_per_command", defaults.max_delta_steps)
            ),
            speed_steps_s=int(payload.get("speed_steps_s", defaults.speed_steps_s)),
            mapping=payload.get("mapping"),
            open_steps={
                side: tuple(int(value) for value in payload.get(side, {}).get(
                    "open_steps", defaults.open_steps[side]
                ))
                for side in ("left", "right")
            },
            closed_steps={
                side: tuple(int(value) for value in payload.get(side, {}).get(
                    "closed_steps", defaults.closed_steps[side]
                ))
                for side in ("left", "right")
            },
        )


def active_targets_to_steps(
    active: dict[str, float],
    open_steps: tuple[int, ...],
    closed_steps: tuple[int, ...],
) -> list[int]:
    """Map six normalized visual targets into the hardware channel order."""
    if len(open_steps) != 6 or len(closed_steps) != 6:
        raise ValueError("open_steps and closed_steps must contain six values")
    result = []
    for index, semantic in enumerate(VISION_TO_CHANNEL):
        normalized = float(np.clip(float(active[semantic]), 0.0, 1.0))
        value = round(open_steps[index] + normalized * (closed_steps[index] - open_steps[index]))
        result.append(int(np.clip(value, 0, CHANNEL_LIMITS[index])))
    return result


def mapped_targets_to_steps(
    active: dict[str, float],
    mapping: dict[str, Any] | None,
    open_steps: tuple[int, ...],
    closed_steps: tuple[int, ...],
    *,
    gesture: str | None = None,
    pinch_contact_strength: float | None = None,
) -> list[int]:
    """Map visual targets, blending the dedicated pinch pose by contact strength."""
    base = active_targets_to_steps(active, open_steps, closed_steps)
    if mapping and gesture == "pinch" and mapping.get("pinch_target"):
        target = mapping["pinch_target"].get("position_steps_median")
        if isinstance(target, list) and len(target) == 6:
            strength = (
                1.0
                if pinch_contact_strength is None
                else float(np.clip(pinch_contact_strength, 0.0, 1.0))
            )
            return [
                int(
                    np.clip(
                        round(base[index] + strength * (float(value) - base[index])),
                        0,
                        CHANNEL_LIMITS[index],
                    )
                )
                for index, value in enumerate(target)
            ]
    if not mapping or not mapping.get("mappings"):
        return base
    result = list(base)
    for item in mapping["mappings"]:
        channel = int(item["channel"]) - 1
        feature = str(item["feature"])
        value = float(active.get(feature, 0.0))
        predicted = float(item["intercept_steps"]) + float(
            item["slope_steps_per_normalized"]
        ) * value
        result[channel] = int(np.clip(round(predicted), 0, CHANNEL_LIMITS[channel]))
    return result


class HardwareTeleopBridge:
    """Arm, rate-limit and stop commands without owning the EtherCAT process."""

    def __init__(
        self,
        backend: CommandSink,
        state: ConsoleState,
        connected_sides: set[str],
        config: TeleopConfig | None = None,
    ) -> None:
        self.backend = backend
        self.state = state
        self.connected_sides = set(connected_sides)
        self.config = config or TeleopConfig()
        self.armed = False
        self.message = "DISARMED - press E to enable"
        self.last_seen_s: dict[str, float] = {}
        self.last_sent_s = 0.0
        self.command_steps: dict[str, list[int]] = {}

    def bus_ready(self) -> bool:
        return (
            self.state.state == "OP"
            and self.state.expected_wkc > 0
            and self.state.wkc >= self.state.expected_wkc
            and all(self.state.hands[side].connected for side in self.connected_sides)
            and not any(
                any(self.state.hands[side].fault) for side in self.connected_sides
            )
        )

    def bus_not_ready_reason(self) -> str:
        """Return the actual EtherCAT gate state shown after an E request."""
        side_state = []
        for side in sorted(self.connected_sides):
            hand = self.state.hands[side]
            faults = ",".join(str(value) for value in hand.fault if value)
            detail = f"{side}:connected={hand.connected}"
            if faults:
                detail += f",fault={faults}"
            side_state.append(detail)
        suffix = "; ".join(side_state)
        return (
            f"state={self.state.state} WKC={self.state.wkc}/{self.state.expected_wkc} "
            f"{suffix}"
        )

    def arm(self, states: list[dict[str, Any]]) -> bool:
        visible = {str(item["handedness"]).lower() for item in states}
        missing = self.connected_sides - visible
        if not self.bus_ready():
            self.message = "CANNOT ARM - " + self.bus_not_ready_reason()
            return False
        if missing:
            self.message = "CANNOT ARM - show " + ", ".join(sorted(missing)) + " hand"
            return False
        self.command_steps = {
            side: [
                int(np.clip(value, 0, CHANNEL_LIMITS[index]))
                for index, value in enumerate(self.state.hands[side].position)
            ]
            for side in self.connected_sides
        }
        now = time.monotonic()
        self.last_seen_s = {side: now for side in self.connected_sides}
        self.last_sent_s = 0.0
        self.armed = True
        # Establish the command path at the measured position first.  This is
        # a no-motion SETALL, then the next visual frame supplies retargeted
        # positions.  It prevents a first visual command being discarded while
        # the interactive backend is still staging its control word.
        for side, values in self.command_steps.items():
            self.backend.send(f"SETALL {side[0].upper()} {' '.join(str(value) for value in values)}")
        self.message = "ARMED - visual targets control hardware"
        return True

    def disarm(self, reason: str = "operator pause") -> None:
        for side in self.connected_sides:
            self.backend.send(f"STOP {side[0].upper()}")
        self.armed = False
        self.message = f"DISARMED - {reason}"

    def toggle(self, states: list[dict[str, Any]]) -> bool:
        if self.armed:
            self.disarm()
            return False
        return self.arm(states)

    def update(self, states: list[dict[str, Any]], now: float | None = None) -> None:
        if not self.armed:
            return
        current_time = time.monotonic() if now is None else now
        by_side = {str(item["handedness"]).lower(): item for item in states}
        for side in self.connected_sides:
            if side in by_side and not by_side[side].get("tracking_hold", False):
                self.last_seen_s[side] = current_time
            elif current_time - self.last_seen_s.get(side, 0.0) > self.config.loss_timeout_s:
                self.disarm(f"{side} tracking lost")
                return
        if not self.bus_ready():
            self.disarm("EtherCAT feedback fault")
            return
        interval = 1.0 / max(self.config.command_rate_hz, 1.0)
        if current_time - self.last_sent_s < interval:
            return
        visible_sides = self.connected_sides.intersection(by_side)
        if not visible_sides:
            # A single MediaPipe frame can miss an otherwise visible hand.
            # Retain the previously sent target during the configured grace
            # period; the loss-timeout branch above remains responsible for
            # sending STOP if tracking does not recover.
            return
        for side in sorted(visible_sides):
            active = by_side[side]["active_dof_targets"]
            desired = mapped_targets_to_steps(
                active,
                self.config.mapping,
                self.config.open_steps[side],
                self.config.closed_steps[side],
                gesture=by_side[side].get("gesture"),
                pinch_contact_strength=by_side[side].get("pinch_contact_strength"),
            )
            previous = self.command_steps[side]
            limited = [
                int(
                    np.clip(
                        target,
                        current - self.config.max_delta_steps,
                        current + self.config.max_delta_steps,
                    )
                )
                for current, target in zip(previous, desired, strict=True)
            ]
            self.command_steps[side] = limited
            values = " ".join(str(value) for value in limited)
            self.backend.send(f"SETALL {side[0].upper()} {values}")
        self.last_sent_s = current_time
