"""MuJoCo FR5 + IH01-X1-R validation and Quest teleoperation simulator."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import socket
import time

from .protocol import HandStreamAssembler
from .runtime import Pipeline

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "simulation" / "fr5_ih01_right.xml"
CONFIG = ROOT / "simulation" / "teleop_resolved.json"
ARM_HOME = (0.0, -1.35, 1.50, -1.55, -1.20, 0.0)
HAND_ACTUATORS = (
    "rh_pinky_flex", "rh_ring_flex", "rh_middle_flex",
    "rh_index_flex", "rh_thumb_flex", "rh_thumb_opposition",
)
HAND_MAX = (1.396263, 1.396263, 1.396263, 1.396263, 0.959931, 0.7)
HAND_STEPS = (1700, 1700, 1700, 1700, 1700, 1300)


class CombinedSimulation:
    def __init__(self, model_path: Path = MODEL) -> None:
        import mujoco
        import numpy as np
        self.mujoco, self.np = mujoco, np
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.hand_body = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "rh_hand"
        )
        if self.hand_body < 0:
            raise RuntimeError("combined model has no attached rh_hand")
        self.data.qpos[:6] = ARM_HOME
        self.arm_target = list(ARM_HOME)
        self.hand_target_steps = [0] * 6
        mujoco.mj_forward(self.model, self.data)
        # Keep the manual viewer at the authored home pose.  In static mode
        # the MuJoCo control sliders can then drive the position actuators.
        for index, value in enumerate(self.arm_target):
            self.data.ctrl[self.actuator_id(f"fr5_j{index + 1}")] = value
        self.anchor_mm = tuple(float(value) * 1000.0 for value in self.data.xpos[self.hand_body]) + (0.0, 0.0, 0.0)

    def actuator_id(self, name: str) -> int:
        value = self.mujoco.mj_name2id(
            self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, name
        )
        if value < 0:
            raise RuntimeError(f"missing actuator {name}")
        return value

    def solve_position_ik(self, target_xyz_m, iterations: int = 35) -> float:
        np, mujoco = self.np, self.mujoco
        target = np.asarray(target_xyz_m, dtype=float)
        jacp = np.zeros((3, self.model.nv)); jacr = np.zeros((3, self.model.nv))
        for _ in range(iterations):
            mujoco.mj_forward(self.model, self.data)
            error = target - self.data.xpos[self.hand_body]
            if float(np.linalg.norm(error)) < 0.001:
                break
            mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.hand_body)
            jacobian = jacp[:, :6]
            damping = 2.5e-3
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(3), error
            )
            delta = np.clip(delta, -0.06, 0.06)
            self.data.qpos[:6] += delta
            for index in range(6):
                lo, hi = self.model.jnt_range[index]
                self.data.qpos[index] = np.clip(self.data.qpos[index], lo, hi)
        mujoco.mj_forward(self.model, self.data)
        self.arm_target = [float(value) for value in self.data.qpos[:6]]
        return float(np.linalg.norm(target - self.data.xpos[self.hand_body]))

    def apply_targets(self, target: dict) -> float:
        pose = target.get("arm_pose_mm_deg")
        if pose is None:
            return math.inf
        error = self.solve_position_ik([float(value) / 1000.0 for value in pose[:3]])
        self.hand_target_steps = [int(value) for value in target["ih01_steps"]]
        return error

    def hand_position_mm(self) -> tuple[float, float, float]:
        """Return the simulated IH01 base position in the same units as Quest targets."""
        return tuple(float(value) * 1000.0 for value in self.data.xpos[self.hand_body])

    def step(self) -> None:
        for index, value in enumerate(self.arm_target):
            self.data.ctrl[self.actuator_id(f"fr5_j{index + 1}")] = value
        for name, maximum, steps, step_max in zip(
            HAND_ACTUATORS, HAND_MAX, self.hand_target_steps, HAND_STEPS
        ):
            self.data.ctrl[self.actuator_id(name)] = maximum * steps / step_max
        self.mujoco.mj_step(self.model, self.data)


class QuestTcpSource:
    def __init__(self, host: str, port: int) -> None:
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port)); self.server.listen(1); self.server.setblocking(False)
        self.connection = None; self.buffer = ""; self.assembler = HandStreamAssembler()
        print(f"SIM LISTEN tcp://{host}:{port}", flush=True)

    def poll(self):
        if self.connection is None:
            try:
                self.connection, address = self.server.accept()
                self.connection.setblocking(False)
                print(f"SIM CONNECTED source={address}", flush=True)
            except BlockingIOError:
                return []
        try:
            payload = self.connection.recv(65536)
        except BlockingIOError:
            return []
        if not payload:
            self.connection.close(); self.connection = None; self.buffer = ""
            return []
        self.buffer += payload.decode("utf-8", errors="replace")
        lines = self.buffer.split("\n"); self.buffer = lines.pop(); frames = []
        for line in lines:
            frame = self.assembler.feed(line)
            if frame is not None: frames.append(frame)
        return frames

    def close(self) -> None:
        if self.connection is not None: self.connection.close()
        self.server.close()


def validate(model_path: Path = MODEL, steps: int = 50) -> dict:
    sim = CombinedSimulation(model_path)
    initial = sim.np.asarray(sim.anchor_mm[:3]) / 1000.0
    ik_error = sim.solve_position_ik(initial + sim.np.asarray([0.015, 0.0, 0.0]))
    sim.hand_target_steps = [850, 850, 850, 850, 850, 650]
    for _ in range(steps): sim.step()
    if not math.isfinite(ik_error) or ik_error > 0.006:
        raise RuntimeError(f"combined IK validation failed: {ik_error:.6f} m")
    return {
        "model": model_path.name, "nq": sim.model.nq, "nu": sim.model.nu,
        "bodies": sim.model.nbody, "arm_dof": 6, "hand": "IH01-X1-R",
        "ik_error_m": round(ik_error, 6),
        "hand_world_xyz": [round(float(value), 4) for value in sim.data.xpos[sim.hand_body]],
    }


def run_headless_teleop(model_path: Path, config_path: Path, host: str, port: int,
                        side: str, duration_s: float) -> dict:
    """Run the Quest-to-MuJoCo path without a GUI and report tracking error.

    This is intentionally a dry-run: it never imports the FAIRINO SDK and only
    reports the simulated wrist target versus the simulated IH01 base position.
    """
    sim = CombinedSimulation(model_path)
    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    cfg["quest"]["mapping_mode"] = side
    pipeline = Pipeline(cfg, robot_anchor=sim.anchor_mm)
    source = QuestTcpSource(host, port)
    started = time.monotonic(); frames = 0; errors = []; last_target = None
    first_frame_at = None; last_frame_at = None
    try:
        while time.monotonic() - started < duration_s:
            for frame in source.poll():
                target = pipeline.process(frame)
                if target is None:
                    continue
                ik_error = sim.apply_targets(target)
                target_xyz = tuple(float(value) for value in target["arm_pose_mm_deg"][:3])
                last_target = target_xyz
                errors.append(ik_error * 1000.0)
                frames += 1
                now = time.monotonic()
                first_frame_at = now if first_frame_at is None else first_frame_at
                last_frame_at = now
            sim.step()
            time.sleep(float(sim.model.opt.timestep))
    finally:
        source.close()
    actual = sim.hand_position_mm()
    return {
        "frames": frames,
        "duration_s": round(time.monotonic() - started, 3),
        "input_hz": round((frames - 1) / max(1e-9, last_frame_at - first_frame_at), 1) if frames > 1 else 0.0,
        "ik_error_mm_max": round(max(errors, default=0.0), 3),
        "ik_error_mm_rms": round(math.sqrt(sum(e * e for e in errors) / len(errors)), 3) if errors else 0.0,
        "last_target_xyz_mm": list(last_target) if last_target else None,
        "last_sim_ih01_xyz_mm": [round(value, 3) for value in actual],
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=MODEL)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--listen", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--side", choices=("left", "right", "both"), default="right")
    parser.add_argument("--steps", type=int, default=50)
    parser.add_argument("--duration", type=float, default=10.0,
                        help="seconds for --headless --listen Quest test")
    parser.add_argument("--auto-hand", action="store_true",
                        help="run the scripted hand animation in the standalone viewer")
    args = parser.parse_args(argv)
    result = validate(args.model, args.steps)
    print("PASS combined-sim", json.dumps(result, separators=(",", ":")))
    if args.headless:
        if args.listen:
            telemetry = run_headless_teleop(args.model, args.config, args.host,
                                             args.port, args.side, args.duration)
            print("PASS quest-sim", json.dumps(telemetry, separators=(",", ":")))
        return 0
    import mujoco.viewer
    sim = CombinedSimulation(args.model)
    pipeline = None
    source = None
    if args.listen:
        cfg = json.loads(args.config.read_text(encoding="utf-8"))
        cfg["quest"]["mapping_mode"] = args.side
        pipeline = Pipeline(cfg, robot_anchor=sim.anchor_mm)
        source = QuestTcpSource(args.host, args.port)
    def key_callback(key: int) -> None:
        # MuJoCo passes GLFW key codes; printable keys retain their ASCII value.
        try:
            char = chr(key).lower()
        except (ValueError, OverflowError):
            char = ""
        if char == " ":
            sim.paused = not sim.paused
            print(f"SIM {'PAUSED' if sim.paused else 'RUNNING'}", flush=True)
            return
        if char in "123456":
            sim.selected_joint = int(char) - 1
            print(f"SIM SELECT J{sim.selected_joint + 1}", flush=True)
            return
        if char in "qaw sedrftgyh".replace(" ", ""):
            pairs = (("q", "a"), ("w", "s"), ("e", "d"), ("r", "f"), ("t", "g"), ("y", "h"))
            for index, (positive, negative) in enumerate(pairs):
                if char == positive: sim.selected_joint, delta = index, 0.08
                elif char == negative: sim.selected_joint, delta = index, -0.08
                else: continue
                sim.arm_target[index] = float(max(sim.model.jnt_range[index, 0], min(sim.model.jnt_range[index, 1], sim.arm_target[index] + delta)))
                sim.data.qpos[index] = sim.arm_target[index]
                sim.data.ctrl[sim.actuator_id(f"fr5_j{index + 1}")] = sim.arm_target[index]
                sim.mujoco.mj_forward(sim.model, sim.data)
                print(f"SIM JOINT J{index + 1} target={sim.arm_target[index]:.3f} rad", flush=True)
                return
        if char == "0":
            sim.arm_target = list(ARM_HOME)
            sim.data.qpos[:6] = ARM_HOME
            for index, value in enumerate(ARM_HOME):
                sim.data.ctrl[sim.actuator_id(f"fr5_j{index + 1}")] = value
            sim.mujoco.mj_forward(sim.model, sim.data)
            print("SIM RESET ARM", flush=True)
        elif char == "o":
            sim.hand_target_steps = [0] * 6
        elif char == "c":
            sim.hand_target_steps = list(HAND_STEPS)

    sim.selected_joint = 0
    sim.paused = False
    print("SIM CONTROLS: q/a J1, w/s J2, e/d J3, r/f J4, t/g J5, y/h J6; 0 reset; o open; c close; SPACE pause", flush=True)
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=key_callback) as viewer:
        viewer.cam.distance=1.45; viewer.cam.azimuth=135; viewer.cam.elevation=-18; viewer.cam.lookat[:]=(0,0,0.35)
        started=time.monotonic(); last_target = None; last_ik_error = None; last_report = 0.0; last_state = 0.0
        try:
            while viewer.is_running():
                if sim.paused:
                    sim.mujoco.mj_forward(sim.model, sim.data)
                elif source is None and args.auto_hand:
                    closure=int((0.5+0.45*math.sin(time.monotonic()-started))*1300)
                    sim.hand_target_steps=[closure]*6
                    sim.step()
                elif source is not None:
                    for frame in source.poll():
                        target=pipeline.process(frame)
                        if target:
                            last_target = tuple(float(value) for value in target["arm_pose_mm_deg"][:3])
                            last_ik_error = sim.apply_targets(target) * 1000.0
                    sim.step()
                else:
                    # Manual inspection mode: position controls are treated
                    # as direct joint commands so the right-side MuJoCo
                    # sliders move immediately. Reject a requested pose that
                    # creates a contact instead of letting links interpenetrate.
                    previous_qpos = sim.data.qpos.copy()
                    sim.data.qpos[:6] = sim.data.ctrl[:6]
                    sim.mujoco.mj_forward(sim.model, sim.data)
                    if sim.data.ncon:
                        sim.data.qpos[:] = previous_qpos
                        sim.mujoco.mj_forward(sim.model, sim.data)
                now = time.monotonic()
                if last_target is not None and now - last_report >= 0.2:
                    actual = sim.hand_position_mm()
                    position_error = math.dist(last_target, actual)
                    print("SIM TELEMETRY", json.dumps({
                        "type": "sim_telemetry",
                        "target_xyz_mm": [round(value, 2) for value in last_target],
                        "sim_ih01_xyz_mm": [round(value, 2) for value in actual],
                        "position_error_mm": round(position_error, 3),
                        "ik_error_mm": round(last_ik_error or 0.0, 3),
                    }, separators=(",", ":")), flush=True)
                    last_report = now
                if source is None and now - last_state >= 0.5:
                    sim.mujoco.mj_forward(sim.model, sim.data)
                    joints = [round(float(value), 3) for value in sim.data.qpos[:6]]
                    print("SIM STATE", json.dumps({"joints_rad": joints,
                          "ih01_xyz_mm": [round(value, 1) for value in sim.hand_position_mm()]}, separators=(",", ":")), flush=True)
                    last_state = now
                viewer.sync(); time.sleep(sim.model.opt.timestep)
        finally:
            if source is not None: source.close()
    return 0


if __name__ == "__main__": raise SystemExit(main())
