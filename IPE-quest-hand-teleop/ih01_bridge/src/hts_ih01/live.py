"""Receive official HTS TCP data and optionally drive read-only IH01 MuJoCo.

Author: haoming
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import socket
import time
from typing import Any

import cv2
import numpy as np

from .protocol import HandFrame, HandStreamAssembler
from .retarget import InitialRetargeter, RetargetResult, SEMANTICS


PANEL_WIDTH = 1100
PANEL_HEIGHT = 780
SLIDER_LEFT = 390
SLIDER_RIGHT = 1065
FOLLOW_SLIDER_Y = 715
MOTOR_SLIDER_Y = 755


def _draw_slider(canvas: np.ndarray, *, label: str, value: int,
                 maximum: int, y: int, digits: int) -> None:
    """Draw a theme-independent slider with a permanently visible value."""
    value = max(1, min(maximum, int(value)))
    text = f"{label} ({value:0{digits}d}/{maximum})"
    cv2.putText(canvas, text, (18, y + 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, (25, 25, 25), 1, cv2.LINE_AA)
    cv2.rectangle(canvas, (SLIDER_LEFT, y - 5), (SLIDER_RIGHT, y + 5),
                  (185, 185, 185), -1)
    knob_x = SLIDER_LEFT + round(
        (SLIDER_RIGHT - SLIDER_LEFT) * value / maximum
    )
    cv2.rectangle(canvas, (SLIDER_LEFT, y - 5), (knob_x, y + 5),
                  (210, 125, 45), -1)
    cv2.circle(canvas, (knob_x, y), 9, (250, 250, 250), -1, cv2.LINE_AA)
    cv2.circle(canvas, (knob_x, y), 9, (125, 125, 125), 1, cv2.LINE_AA)


def _slider_value(x: int, maximum: int) -> int:
    ratio = (int(x) - SLIDER_LEFT) / max(SLIDER_RIGHT - SLIDER_LEFT, 1)
    return max(1, min(maximum, round(ratio * maximum)))


class TcpSource:
    def __init__(self, host: str, port: int) -> None:
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen(2)
        self.server.setblocking(False)
        self.connections: dict[socket.socket, str] = {}
        self.assembler = HandStreamAssembler()

    def poll(self) -> list[HandFrame]:
        while len(self.connections) < 2:
            try:
                connection, address = self.server.accept()
            except BlockingIOError:
                break
            connection.setblocking(False)
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.connections[connection] = ""
            print(f"hts_connected={address[0]}:{address[1]}", flush=True)
        frames: list[HandFrame] = []
        closed: list[socket.socket] = []
        for connection, buffer in list(self.connections.items()):
            while True:
                try:
                    payload = connection.recv(65536)
                except BlockingIOError:
                    break
                except ConnectionError:
                    payload = b""
                if not payload:
                    closed.append(connection)
                    break
                buffer += payload.decode("utf-8", errors="replace")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    try:
                        frame = self.assembler.feed(line)
                    except ValueError as exc:
                        print(f"hts_protocol_warning={exc}", flush=True)
                        continue
                    if frame is not None:
                        frames.append(frame)
            if connection in self.connections:
                self.connections[connection] = buffer
        for connection in closed:
            self.connections.pop(connection, None)
            connection.close()
            print("hts_disconnected=true waiting_for_reconnect=true", flush=True)
        return frames

    def close(self) -> None:
        for connection in self.connections:
            connection.close()
        self.connections.clear()
        self.server.close()


def _state(frame: HandFrame, result: RetargetResult) -> dict[str, Any]:
    return {
        "handedness": frame.side.title(),
        # Preserve metric wrist pose for the simulation.  The hardware bridge
        # ignores these fields; only the MuJoCo wrist retargeting consumes them.
        "wrist_position_m": list(frame.wrist_position),
        "palm_quaternion_wxyz": [frame.wrist_quaternion_xyzw[3],
                                  frame.wrist_quaternion_xyzw[0],
                                  frame.wrist_quaternion_xyzw[1],
                                  frame.wrist_quaternion_xyzw[2]],
        "active_dof_targets": dict(zip(SEMANTICS, result.normalized, strict=True)),
        "gesture": "hand_tracking_streamer",
        "tracking_hold": False,
    }


def _panel(*, source: TcpSource, side: str,
           results: dict[str, RetargetResult], bridge: Any,
           hardware_state: Any, follow_speed: int, speed: int,
           fps: float) -> np.ndarray:
    """Restore the previous QUEST 3 -> IH01 dashboard appearance."""
    canvas = np.full((PANEL_HEIGHT, PANEL_WIDTH, 3), (25, 29, 36), np.uint8)
    connected = source.connections != {}
    armed = bool(bridge is not None and bridge.armed)
    cv2.putText(canvas, "QUEST 3 -> IH01 LIVE RETARGETING", (28, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.88, (80, 220, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, f"Quest: {'CONNECTED' if connected else 'WAITING'}  side={side.upper()}  {fps:.1f} FPS",
                (28, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                (80, 225, 120) if connected else (80, 170, 255), 2, cv2.LINE_AA)
    mode = "HARDWARE" if bridge is not None else "SIMULATION"
    cv2.putText(canvas, f"Mode: {mode}  {'ARMED' if armed else 'DISARMED'}  motor={speed} steps/s",
                (28, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                (80, 225, 120) if armed else (120, 180, 255), 2, cv2.LINE_AA)
    if bridge is not None and getattr(bridge, "backend", None) is not None:
        process = getattr(bridge.backend, "process", None)
        if process is not None and process.poll() is not None:
            msg = f"BACKEND EXITED code={process.returncode}"
        else:
            msg = bridge.message
    else:
        msg = "SIMULATION ONLY - no hardware command"
    cv2.putText(canvas, msg[-95:], (28, 158), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (210, 215, 225), 1, cv2.LINE_AA)
    cv2.putText(canvas, "HTS geometric mapping | per-channel target output",
                (28, 184), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (170, 205, 180), 1, cv2.LINE_AA)
    def draw_result(result_side: str, result: RetargetResult | None, x: int, width: int) -> None:
        heading_color = (100, 230, 150) if result is not None else (135, 145, 160)
        cv2.putText(canvas, f"{result_side.upper()} HAND", (x, 218),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, heading_color, 1, cv2.LINE_AA)
        if result is None:
            cv2.putText(canvas, "waiting for HTS landmarks", (x, 252),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.43, (160, 165, 175), 1, cv2.LINE_AA)
            return
        for index, (name, normalized, steps) in enumerate(
            zip(SEMANTICS, result.normalized, result.steps, strict=True)
        ):
            y = 252 + index * 60
            cv2.putText(canvas, f"CH{index + 1} {name}  norm={normalized:0.2f}  target={steps:4d}",
                        (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.40 if side == "both" else 0.52,
                        (130, 215, 255), 1, cv2.LINE_AA)
            # Keep the bar on its own line so it cannot cover target values.
            bar_y = y + 10
            bar_width = width - 12
            cv2.rectangle(canvas, (x, bar_y - 5), (x + bar_width, bar_y + 5), (60, 65, 75), -1)
            cv2.rectangle(canvas, (x, bar_y - 5),
                          (x + round(bar_width * normalized), bar_y + 5), (85, 190, 245), -1)
            if hardware_state is not None and result_side in hardware_state.hands:
                view = hardware_state.hands[result_side]
                if view.connected:
                    feedback = (
                        f"fb pos={view.position[index]:4d}  I={view.current[index]:4d}mA "
                        f"F={view.force[index]:5d}  T={view.temperature[index]:2d}C "
                        f"fault={view.fault[index]}"
                    )
                    cv2.putText(canvas, feedback, (x, y + 31),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.34,
                                (175, 185, 195) if view.fault[index] == 0 else (80, 80, 255),
                                1, cv2.LINE_AA)
        cv2.putText(canvas, f"quality={result.quality:.2f}", (x, 635),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 225, 150), 1, cv2.LINE_AA)

    if side == "both":
        draw_result("left", results.get("left"), 28, 515)
        draw_result("right", results.get("right"), 560, 515)
    else:
        draw_result(side, results.get(side), 40, 1000)
    if hardware_state is not None:
        cv2.putText(canvas, f"EtherCAT {hardware_state.state} WKC {hardware_state.wkc}/{hardware_state.expected_wkc}",
                    (600, 158), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (100, 225, 140) if hardware_state.state == "OP" else (80, 170, 255), 1, cv2.LINE_AA)
    controls = ("E arm/disarm | SPACE pause | R clear fault | Q quit"
                if bridge is not None else
                "L/R/B switch simulated hand | Q quit")
    cv2.putText(canvas, controls, (28, 675), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (120, 200, 255), 1, cv2.LINE_AA)
    # Native OpenCV/Qt trackbar captions become white-on-white with some Linux
    # desktop themes.  Draw the two controls ourselves so their current/max
    # values remain readable, matching the main visual-teleop console.
    cv2.rectangle(canvas, (0, 690), (PANEL_WIDTH, PANEL_HEIGHT),
                  (245, 245, 245), -1)
    _draw_slider(canvas, label="FOLLOW SPEED (steps/frame)",
                 value=follow_speed, maximum=100,
                 y=FOLLOW_SLIDER_Y, digits=3)
    _draw_slider(canvas, label="MOTOR SPEED (steps/s)",
                 value=speed, maximum=2000,
                 y=MOTOR_SLIDER_Y, digits=4)
    return canvas


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--side", choices=("left", "right", "both"), default="left")
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--hardware", action="store_true", help="Enable IH01 EtherCAT output (starts DISARMED)")
    # Kept as a backwards-compatible no-op for older launchers.  The user-facing
    # YES gate was removed; hardware still requires pressing E in the window.
    parser.add_argument("--confirm-hardware", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--interface", default="enp130s0")
    parser.add_argument("--slave", type=int, default=1)
    parser.add_argument("--speed", type=int, default=1000)
    parser.add_argument("--backend", default="build/hardware/ih01_hand_control_backend")
    args = parser.parse_args()
    if args.hardware and args.side == "both":
        raise SystemExit("Hardware mode supports one hand at a time; use --side left or --side right.")
    # The hardware workflow always keeps a read-only MuJoCo mirror open.
    args.simulate = bool(args.simulate or args.hardware)
    source = TcpSource(args.host, args.port)
    retargeter = InitialRetargeter()
    latest_states: dict[str, dict[str, Any]] = {}
    latest_results: dict[str, RetargetResult] = {}
    latest_time: dict[str, float] = {}
    last_print = 0.0
    frame_count = 0
    started = time.monotonic()
    simulator = None
    viewer = None
    backend = None
    bridge = None
    hardware_state = None
    log_stream = None
    current_speed = [max(1, min(2000, int(args.speed)))]
    current_follow_speed = [81]
    side_index = {"left": 0, "right": 1, "both": 2}
    side_names = ("left", "right", "both")
    selected_side = [args.side]
    window = "Quest 3 -> IH01 retargeting"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    # Match the restored dashboard aspect ratio; the shorter legacy size
    # compressed the channel table and hid the speed control.
    cv2.resizeWindow(window, PANEL_WIDTH, PANEL_HEIGHT)
    print(
        f"hts_adapter=ready protocol=tcp host={args.host} port={args.port} "
        f"side={args.side} simulation={args.simulate} "
        f"hardware_commands={bool(args.hardware)}",
        flush=True,
    )
    try:
        with ExitStack() as stack:
            if args.simulate:
                import mujoco.viewer
                from ih01_runtime.simulation import IH01Simulation

                simulator = IH01Simulation(render=False)
                viewer = stack.enter_context(mujoco.viewer.launch_passive(simulator.model, simulator.data))
                viewer.cam.azimuth = 90.0
                viewer.cam.elevation = -12.0
                viewer.cam.distance = 0.55
                viewer.cam.lookat[:] = (0.0, 0.05, 0.12)
            if args.hardware:
                from ih01_runtime.hardware_control import Backend, ConsoleState
                from ih01_runtime.teleop_bridge import HardwareTeleopBridge, TeleopConfig
                from pathlib import Path
                hardware_state = ConsoleState()
                command = [
                    "sudo", "-n", str(Path(args.backend).resolve()), args.interface,
                    "--left-slave", str(args.slave if args.side == "left" else 0),
                    "--right-slave", str(args.slave if args.side == "right" else 0),
                    "--interactive", "--speed-steps-s", str(max(1, min(2000, args.speed))),
                    "--contact-hold-mode", "visual",
                ]
                log_path = Path("artifacts/hts-ih01-hardware.jsonl")
                log_path.parent.mkdir(parents=True, exist_ok=True)
                log_stream = log_path.open("a", encoding="utf-8")
                backend = Backend(command, hardware_state, log_stream)
                bridge = HardwareTeleopBridge(
                    backend, hardware_state, {args.side},
                    TeleopConfig(command_rate_hz=50.0, loss_timeout_s=2.0,
                                 max_delta_steps=81, speed_steps_s=args.speed),
                )
                backend.send(f"SPEED {max(1, min(2000, args.speed))}")

            def _speed_changed(value: int) -> None:
                value = max(1, min(2000, int(value)))
                current_speed[0] = value
                if backend is not None:
                    backend.send(f"SPEED {value}")

            def _follow_speed_changed(value: int) -> None:
                value = max(1, min(100, int(value)))
                current_follow_speed[0] = value
                if bridge is not None:
                    bridge.config.max_delta_steps = value

            active_slider: list[str | None] = [None]

            def _set_slider_from_x(name: str, x: int) -> None:
                if name == "follow":
                    _follow_speed_changed(_slider_value(x, 100))
                else:
                    _speed_changed(_slider_value(x, 2000))

            def _mouse(event: int, x: int, y: int, flags: int,
                       _userdata: Any) -> None:
                if event == cv2.EVENT_LBUTTONDOWN:
                    if abs(y - FOLLOW_SLIDER_Y) <= 16:
                        active_slider[0] = "follow"
                    elif abs(y - MOTOR_SLIDER_Y) <= 16:
                        active_slider[0] = "motor"
                    if active_slider[0] is not None:
                        _set_slider_from_x(active_slider[0], x)
                elif event == cv2.EVENT_MOUSEMOVE and active_slider[0] is not None:
                    if flags & cv2.EVENT_FLAG_LBUTTON:
                        _set_slider_from_x(active_slider[0], x)
                elif event == cv2.EVENT_LBUTTONUP:
                    if active_slider[0] is not None:
                        _set_slider_from_x(active_slider[0], x)
                    active_slider[0] = None

            cv2.setMouseCallback(window, _mouse)
            def _side_changed(value: int) -> None:
                selected_side[0] = side_names[max(0, min(2, int(value)))]

            cv2.createTrackbar(
                "HAND MODE 0=LEFT 1=RIGHT 2=BOTH", window,
                side_index[args.side], 2, _side_changed,
            )
            while viewer is None or viewer.is_running():
                now = time.monotonic()
                for frame in source.poll():
                    result = retargeter.map(frame)
                    latest_results[frame.side] = result
                    latest_states[frame.side] = _state(frame, result)
                    latest_time[frame.side] = now
                    frame_count += 1
                    if now - last_print >= 0.10:
                        print(
                            f"side={frame.side} quality={result.quality:.2f} "
                            f"steps={list(result.steps)}",
                            flush=True,
                        )
                        last_print = now
                active_sides = ("left", "right") if selected_side[0] == "both" else (selected_side[0],)
                states = [
                    latest_states[side] for side in active_sides
                    if side in latest_states and now - latest_time.get(side, 0.0) <= 0.75
                ]
                # The Quest streamer reports the *operator* hand.  In a
                # single-hand bench setup it is common to connect the left
                # physical IH01 while the operator presents the right hand,
                # or vice versa.  The old adapter accepted that workflow.
                # When exactly one tracked source hand is available, retag it
                # as the selected physical hand before the EtherCAT bridge.
                # If both source hands are available we keep the exact
                # left/right association and never guess.
                if args.hardware and not states and selected_side[0] != "both":
                    fresh_sources = [
                        source_side for source_side in ("left", "right")
                        if source_side in latest_states
                        and now - latest_time.get(source_side, 0.0) <= 0.75
                    ]
                    if len(fresh_sources) == 1:
                        forwarded = dict(latest_states[fresh_sources[0]])
                        forwarded["handedness"] = selected_side[0].title()
                        states = [forwarded]
                if simulator is not None:
                    simulator.update_hands(states)
                    viewer.sync()
                if backend is not None and bridge is not None and hardware_state is not None:
                    backend.poll()
                    bridge.update(states, now)
                fps = frame_count / max(now - started, 1e-6)
                cv2.imshow(window, _panel(source=source, side=selected_side[0],
                                          results=latest_results,
                                          bridge=bridge, hardware_state=hardware_state,
                                          follow_speed=current_follow_speed[0],
                                          speed=current_speed[0], fps=fps))
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                if key in (ord("l"), ord("r"), ord("b")):
                    selected_side[0] = {ord("l"): "left", ord("r"): "right", ord("b"): "both"}[key]
                    cv2.setTrackbarPos("HAND MODE 0=LEFT 1=RIGHT 2=BOTH", window,
                                       side_index[selected_side[0]])
                if bridge is not None and key == ord("e"):
                    armed = bridge.toggle(states)
                    print(
                        f"hts_arm_request side={args.side} result={armed} "
                        f"states={[item.get('handedness') for item in states]} "
                        f"message={bridge.message}",
                        flush=True,
                    )
                elif bridge is not None and key == ord(" "):
                    bridge.disarm("operator pause")
                elif backend is not None and key == ord("r"):
                    backend.send("RESET")
                time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        if simulator is not None:
            simulator.close()
        if bridge is not None and bridge.armed:
            bridge.disarm("shutdown")
        if backend is not None:
            backend.close()
        if log_stream is not None:
            log_stream.close()
        source.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
