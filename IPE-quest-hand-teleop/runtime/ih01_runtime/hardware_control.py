"""Dual-hand OpenCV control and feedback console for IH01 EtherCAT hands.

Author: haoming
"""

from __future__ import annotations

import argparse
import json
import queue
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TextIO

import cv2
import numpy as np

DASHBOARD_WINDOW = "IH01 Dual-Hand Hardware"
CONTROL_WINDOW = "IH01 Controls"
CHANNEL_NAMES = ("Pinky", "Ring", "Middle", "Index", "Thumb Flex", "Thumb Rotate")
CHANNEL_LIMITS = (1700, 1700, 1700, 1700, 1700, 1300)
SIDES = ("left", "right")
# Fixed dashboard coordinates (the dashboard is rendered at 1600x900).  The
# teleoperation window uses these coordinates for a mouse-click reset button.
FAULT_RESET_BUTTON = (1320, 815, 1565, 865)


@dataclass
class HandView:
    connected: bool = False
    slave: int = 0
    command_active: bool = False
    target: list[int] = field(default_factory=lambda: [0] * 6)
    position: list[int] = field(default_factory=lambda: [0] * 6)
    current: list[int] = field(default_factory=lambda: [0] * 6)
    force: list[int] = field(default_factory=lambda: [0] * 6)
    temperature: list[int] = field(default_factory=lambda: [0] * 6)
    fault: list[int] = field(default_factory=lambda: [0] * 6)
    status_word: int = 0
    contact_hold: list[bool] = field(default_factory=lambda: [False] * 6)
    grasp_hold: bool = False
    fault_latched: bool = False


@dataclass
class ConsoleState:
    hands: dict[str, HandView] = field(
        default_factory=lambda: {side: HandView() for side in SIDES}
    )
    state: str = "STARTING"
    wkc: int = 0
    expected_wkc: int = 0
    cycle: int = 0
    error: str = ""

    def update(self, payload: dict[str, Any]) -> None:
        if payload.get("format") != "ih01_dual_hand_control_v1":
            raise ValueError(f"unsupported backend format: {payload.get('format')}")
        self.state = str(payload.get("state", "UNKNOWN"))
        self.wkc = int(payload.get("wkc", 0))
        self.expected_wkc = int(payload.get("expected_wkc", 0))
        self.cycle = int(payload.get("cycle", 0))
        for item in payload.get("hands", []):
            side = str(item.get("side"))
            if side not in self.hands:
                continue
            view = self.hands[side]
            view.connected = True
            view.slave = int(item["slave"])
            view.command_active = bool(item["command_active"])
            view.target = _six_ints(item, "target_steps")
            view.position = _six_ints(item, "position_steps")
            view.current = _six_ints(item, "current_ma")
            view.force = _six_ints(item, "force_raw")
            view.temperature = _six_ints(item, "temperature_c")
            view.fault = _six_ints(item, "fault_code")
            view.status_word = int(item["status_word"])
            view.contact_hold = [bool(value) for value in item.get("contact_hold", [False] * 6)]
            view.grasp_hold = bool(item.get("grasp_hold", False))
            view.fault_latched = bool(item.get("fault_latched", False))


def _six_ints(payload: dict[str, Any], key: str) -> list[int]:
    values = payload.get(key)
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError(f"{key} must contain six values")
    return [int(value) for value in values]


def _put(
    canvas: np.ndarray,
    text: str,
    xy: tuple[int, int],
    *,
    color: tuple[int, int, int] = (225, 225, 225),
    scale: float = 0.55,
    thickness: int = 1,
) -> None:
    cv2.putText(
        canvas,
        text,
        xy,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def render_dashboard(
    state: ConsoleState,
    control_values: dict[str, list[int]] | None = None,
    follow_speed_steps: int | None = None,
    hardware_speed_steps: int | None = None,
) -> np.ndarray:
    canvas = np.full((900, 1600, 3), (23, 27, 32), dtype=np.uint8)
    connected = state.state == "OP" and state.wkc >= state.expected_wkc > 0
    status_color = (80, 220, 120) if connected else (60, 170, 255)
    _put(
        canvas,
        "IH01 DEXHANDS  |  DUAL-HAND HARDWARE CONSOLE",
        (30, 48),
        color=(80, 220, 255),
        scale=0.92,
        thickness=2,
    )
    _put(
        canvas,
        f"Bus {state.state}  WKC {state.wkc}/{state.expected_wkc}  cycle {state.cycle}",
        (30, 82),
        color=status_color,
        scale=0.68,
        thickness=2,
    )
    if state.error:
        _put(canvas, state.error[-90:], (640, 82), color=(70, 70, 255), scale=0.52)
    if follow_speed_steps is not None:
        _put(
            canvas,
            f"FOLLOW CMD {int(follow_speed_steps)} steps/frame",
            (1050, 82),
            color=(100, 190, 255),
            scale=0.52,
            thickness=1,
        )
    if hardware_speed_steps is not None:
        _put(
            canvas,
            f"MOTOR {int(hardware_speed_steps)} steps/s",
            (1050, 112),
            color=(255, 190, 100),
            scale=0.52,
        )

    for column, side in enumerate(SIDES):
        x0 = 25 + column * 790
        view = state.hands[side]
        title = f"{side.upper()} HAND"
        if view.connected:
            title += f"  slave {view.slave}  {'MOVING/HOLD' if view.command_active else 'CW0'}"
            color = (
                (90, 220, 120)
                if not any(view.fault) and view.status_word == 0
                else (70, 70, 255)
            )
        else:
            title += "  DISCONNECTED"
            color = (130, 140, 150)
        _put(canvas, title, (x0 + 10, 130), color=color, scale=0.72, thickness=2)
        headers = ("CH", "TARGET", "POSITION", "CURRENT", "FORCE", "TEMP", "FAULT")
        widths = (10, 160, 275, 390, 500, 600, 700)
        for header, offset in zip(headers, widths, strict=True):
            _put(canvas, header, (x0 + offset, 170), color=(150, 175, 195), scale=0.48)
        cv2.line(canvas, (x0 + 5, 185), (x0 + 670, 185), (75, 85, 95), 1)
        for channel, name in enumerate(CHANNEL_NAMES):
            y = 220 + channel * 58
            _put(canvas, f"{channel + 1} {name}", (x0 + 10, y), color=(255, 205, 100), scale=0.56)
            if not view.connected:
                continue
            fault = view.fault[channel]
            row_ok = fault == 0 and view.status_word == 0
            row_color = (90, 220, 120) if row_ok else (70, 70, 255)
            _put(canvas, str(view.target[channel]), (x0 + 160, y), scale=0.62)
            _put(canvas, str(view.position[channel]), (x0 + 275, y), scale=0.62)
            _put(canvas, f"{view.current[channel]} mA", (x0 + 390, y), scale=0.55)
            _put(canvas, str(view.force[channel]), (x0 + 500, y), scale=0.62)
            _put(canvas, f"{view.temperature[channel]} C", (x0 + 600, y), scale=0.55)
            _put(canvas, str(fault), (x0 + 700, y), color=row_color, scale=0.68, thickness=2)
        if view.connected:
            held = [str(index + 1) for index, active in enumerate(view.contact_hold) if active]
            protection = (
                f"CONTACT {'/'.join(held) if held else '-'}  "
                f"GRASP {'HOLD' if view.grasp_hold else 'FREE'}"
            )
            _put(canvas, protection, (x0 + 10, 590),
                 color=(80, 220, 255) if held else (155, 185, 205),
                 scale=0.45, thickness=1)

    cv2.line(canvas, (25, 770), (1575, 770), (75, 85, 95), 1)
    footer = (
        "FOLLOW 1..100 steps/frame | MOTOR 1..2000 steps/s"
        if follow_speed_steps is not None
        else "Controls: LEFT window | RIGHT window | SYNC window"
    )
    _put(canvas, footer, (30, 800), color=(190, 205, 220), scale=0.68)
    _put(
        canvas,
        "o: open both  c: close 1700  r: clear fault (CW5 20ms)  "
        "SPACE: CW0 hold  q/Esc: disconnect",
        (30, 850),
        color=(100, 190, 255),
        scale=0.68,
        thickness=2,
    )
    bx1, by1, bx2, by2 = FAULT_RESET_BUTTON
    cv2.rectangle(canvas, (bx1, by1), (bx2, by2), (80, 55, 45), -1)
    cv2.rectangle(canvas, (bx1, by1), (bx2, by2), (120, 150, 190), 2)
    _put(
        canvas,
        "CLEAR FAULT",
        (bx1 + 28, by1 + 32),
        color=(255, 220, 150),
        scale=0.58,
        thickness=2,
    )
    _put(canvas, "CW5 -> CW0", (bx1 + 42, by1 + 48), color=(200, 210, 220), scale=0.38)
    return canvas


class Backend:
    def __init__(
        self,
        command: list[str],
        state: ConsoleState,
        log_stream: TextIO,
    ) -> None:
        self.state = state
        self.log_stream = log_stream
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.payloads: queue.Queue[dict[str, Any]] = queue.Queue()
        self.stderr_lines: queue.Queue[str] = queue.Queue()
        threading.Thread(target=self._read_stdout, daemon=True).start()
        threading.Thread(target=self._read_stderr, daemon=True).start()

    def _read_stdout(self) -> None:
        assert self.process.stdout is not None
        for line in self.process.stdout:
            self.log_stream.write(line)
            self.log_stream.flush()
            try:
                self.payloads.put(json.loads(line))
            except json.JSONDecodeError:
                self.stderr_lines.put(f"Invalid backend JSON: {line.strip()}")

    def _read_stderr(self) -> None:
        assert self.process.stderr is not None
        for line in self.process.stderr:
            self.stderr_lines.put(line.strip())

    def send(self, command: str) -> None:
        if self.process.poll() is not None or self.process.stdin is None:
            self.state.error = (
                "EtherCAT backend is not running "
                f"(exit={self.process.returncode}); command not sent: {command}"
            )
            return
        try:
            self.process.stdin.write(command.rstrip() + "\n")
            self.process.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            self.state.error = f"EtherCAT command pipe failed: {exc}"

    def poll(self) -> None:
        while True:
            try:
                self.state.update(self.payloads.get_nowait())
            except queue.Empty:
                break
            except (KeyError, TypeError, ValueError) as exc:
                self.state.error = f"Feedback error: {exc}"
        while True:
            try:
                line = self.stderr_lines.get_nowait()
            except queue.Empty:
                break
            if line:
                self.state.error = line

    def close(self) -> int:
        if self.process.poll() is None:
            self.send("QUIT")
            try:
                return self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    return self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
        return int(self.process.wait())


class TrackbarControls:
    def __init__(self, backend: Backend, state: ConsoleState) -> None:
        self.backend = backend
        self.state = state
        self.ready = {side: False for side in SIDES}
        self.suppress = False
        self.initialized = {side: False for side in SIDES}
        for side in SIDES:
            window = CONTROL_WINDOW
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window, 620, 620)
            for channel, (name, limit) in enumerate(
                zip(CHANNEL_NAMES, CHANNEL_LIMITS, strict=True), start=1
            ):
                cv2.createTrackbar(
                    f"{side[0].upper()} CH{channel} {name}", window, 0, limit,
                    self._channel_callback(side, channel),
                )
            cv2.createTrackbar(
                f"{side[0].upper()} ALL FLEX", window, 0, 1700,
                self._flex_callback(side),
            )
        cv2.createTrackbar("BOTH FLEX", CONTROL_WINDOW, 0, 1700, self._both_callback())
        cv2.createTrackbar("MOTOR SPEED (steps/s)", CONTROL_WINDOW, 1000, 2000,
                           self._speed_callback())

    def _channel_callback(self, side: str, channel: int):
        def callback(value: int) -> None:
            if not self.suppress and self.ready[side]:
                self.backend.send(f"SET {side[0].upper()} {channel} {value}")
        return callback

    def _flex_callback(self, side: str):
        def callback(value: int) -> None:
            if self.suppress or not self.ready[side]:
                return
            self.suppress = True
            try:
                for channel in range(5):
                    cv2.setTrackbarPos(
                        f"{side[0].upper()} CH{channel + 1} {CHANNEL_NAMES[channel]}",
                        CONTROL_WINDOW, min(value, CHANNEL_LIMITS[channel])
                    )
            finally:
                self.suppress = False
            self.backend.send(f"FLEX {side[0].upper()} {value}")
        return callback

    def _both_callback(self):
        def callback(value: int) -> None:
            if self.suppress:
                return
            for side in SIDES:
                if self.ready[side]:
                    self.backend.send(f"FLEX {side[0].upper()} {value}")
        return callback

    def _speed_callback(self):
        def callback(value: int) -> None:
            if value > 0:
                self.backend.send(f"SPEED {value}")
        return callback

    def synchronize(self) -> None:
        for side in SIDES:
            view = self.state.hands[side]
            if not view.connected or self.initialized[side]:
                continue
            self.suppress = True
            try:
                for channel, target in enumerate(view.target):
                    cv2.setTrackbarPos(
                        f"{side[0].upper()} CH{channel + 1} {CHANNEL_NAMES[channel]}",
                        CONTROL_WINDOW, max(0, min(target, CHANNEL_LIMITS[channel]))
                    )
                cv2.setTrackbarPos(
                    f"{side[0].upper()} ALL FLEX", CONTROL_WINDOW,
                    int(round(sum(view.target[:5]) / 5))
                )
            finally:
                self.suppress = False
            self.initialized[side] = True
            self.ready[side] = True

    def set_both_visual(self, value: int) -> None:
        cv2.setTrackbarPos("BOTH FLEX", CONTROL_WINDOW, value)
        for side in SIDES:
            if self.ready[side]:
                cv2.setTrackbarPos(f"{side[0].upper()} ALL FLEX", CONTROL_WINDOW, value)


def run_console(args: argparse.Namespace) -> int:
    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "sudo",
        "-n",
        args.backend,
        args.interface,
        "--left-slave",
        str(args.left_slave),
        "--right-slave",
        str(args.right_slave),
        "--interactive",
        "--contact-hold-mode",
        args.contact_hold_mode,
        "--contact-hold-delay-ms",
        str(args.contact_hold_delay_ms),
    ]
    if args.enable_thumb_index_soft_limit:
        command.append("--enable-thumb-index-soft-limit")
    state = ConsoleState()
    return_code = 1
    with log_path.open("w", encoding="utf-8") as log_stream:
        backend = Backend(command, state, log_stream)
        cv2.namedWindow(DASHBOARD_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(DASHBOARD_WINDOW, 1200, 675)
        reset_requested = [False]

        def _reset_mouse(event: int, x: int, y: int, flags: int, userdata: object) -> None:
            del flags, userdata
            if event != cv2.EVENT_LBUTTONUP:
                return
            width = max(cv2.getWindowImageRect(DASHBOARD_WINDOW)[2], 1)
            height = max(cv2.getWindowImageRect(DASHBOARD_WINDOW)[3], 1)
            px, py = int(x * 1600 / width), int(y * 900 / height)
            bx1, by1, bx2, by2 = FAULT_RESET_BUTTON
            if bx1 <= px <= bx2 and by1 <= py <= by2:
                reset_requested[0] = True

        cv2.setMouseCallback(DASHBOARD_WINDOW, _reset_mouse)
        controls = TrackbarControls(backend, state)
        backend.send("SPEED 1000")
        try:
            while True:
                backend.poll()
                controls.synchronize()
                cv2.imshow(DASHBOARD_WINDOW, render_dashboard(state))
                key = cv2.waitKey(20) & 0xFF
                if reset_requested[0]:
                    reset_requested[0] = False
                    backend.send("RESET")
                if key in (ord("q"), 27):
                    break
                if key == ord(" "):
                    backend.send("STOP L")
                    backend.send("STOP R")
                elif key == ord("o"):
                    controls.set_both_visual(0)
                    backend.send("BOTH 0")
                elif key == ord("c"):
                    controls.set_both_visual(1700)
                    backend.send("BOTH 1700")
                elif key == ord("r"):
                    backend.send("RESET")
                if backend.process.poll() is not None:
                    backend.poll()
                    if not state.error:
                        state.error = f"Backend exited with code {backend.process.returncode}"
                    cv2.imshow(DASHBOARD_WINDOW, render_dashboard(state))
                    cv2.waitKey(1500)
                    break
        finally:
            return_code = backend.close()
            cv2.destroyAllWindows()
    print(f"Session log: {log_path}")
    return return_code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="enp130s0")
    parser.add_argument("--left-slave", type=int, default=0)
    parser.add_argument("--right-slave", type=int, default=1)
    parser.add_argument("--contact-hold-mode", choices=("visual", "quest"), default="visual")
    parser.add_argument("--contact-hold-delay-ms", type=int, default=200)
    parser.add_argument("--enable-thumb-index-soft-limit", action="store_true")
    parser.add_argument(
        "--backend",
        default=str(
            Path(__file__).resolve().parents[2]
            / "build"
            / "hardware"
            / "ih01_hand_control_backend"
        ),
    )
    parser.add_argument(
        "--log",
        default=f"artifacts/ih01-hand-control-{time.strftime('%Y%m%d-%H%M%S')}.jsonl",
    )
    args = parser.parse_args()
    if args.left_slave < 0 or args.right_slave < 0:
        parser.error("slave indices cannot be negative; 0 means disconnected")
    if args.left_slave == args.right_slave != 0:
        parser.error("left and right hands cannot use the same slave")
    if args.left_slave == args.right_slave == 0:
        parser.error("at least one hand must be assigned")
    if not 20 <= args.contact_hold_delay_ms <= 60000:
        parser.error("contact hold delay must be between 20 and 60000 ms")
    if not Path(args.backend).is_file():
        parser.error(f"backend binary not found: {args.backend}")
    return run_console(args)


if __name__ == "__main__":
    raise SystemExit(main())
