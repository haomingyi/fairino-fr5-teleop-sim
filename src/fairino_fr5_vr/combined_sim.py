"""MuJoCo FR5 + IH01-X1-R validation and Quest teleoperation simulator."""
from __future__ import annotations

import argparse
import json
import math
import numpy as np
from pathlib import Path
import socket
import time

from .protocol import HandStreamAssembler
from .runtime import Pipeline

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "simulation" / "fr5_ih01_right.xml"
CONFIG = ROOT / "simulation" / "teleop_resolved.json"
# Neutral upright working pose: the arm and sideways-mounted IH01 match the
# operator's requested screenshot posture used for wrist anchoring.
# Values are radians and remain inside the official FR5 joint limits.
ARM_HOME = (-0.0305, -1.14, -1.92, 0.0174, 1.56, 0.0)
HAND_ACTUATORS = (
    "rh_pinky_flex", "rh_ring_flex", "rh_middle_flex",
    "rh_index_flex", "rh_thumb_flex", "rh_thumb_opposition",
)
HAND_MAX = (1.396263, 1.396263, 1.396263, 1.396263, 0.959931, 0.7)
HAND_STEPS = (1700, 1700, 1700, 1700, 1700, 1300)


def _rpy_deg_to_matrix(np, rpy_deg):
    """FR5-style roll/pitch/yaw degrees to a ZYX rotation matrix."""
    roll, pitch, yaw = (math.radians(float(value)) for value in rpy_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray((
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    ), dtype=float)


def _matrix_to_rpy_deg(np, matrix):
    matrix = np.asarray(matrix, dtype=float).reshape(3, 3)
    pitch = math.asin(max(-1.0, min(1.0, -float(matrix[2, 0]))))
    if abs(math.cos(pitch)) > 1e-7:
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    else:
        roll = math.atan2(-float(matrix[1, 2]), float(matrix[1, 1]))
        yaw = 0.0
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def _rotation_error(np, target, current):
    """World-frame rotation vector from current orientation to target."""
    relative = np.asarray(target) @ np.asarray(current).T
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) * 0.5))
    angle = math.acos(cosine)
    vee = np.asarray((relative[2, 1] - relative[1, 2],
                      relative[0, 2] - relative[2, 0],
                      relative[1, 0] - relative[0, 1]), dtype=float) * 0.5
    if angle < 1e-7:
        return vee
    sine = math.sin(angle)
    if abs(sine) < 1e-7:
        return vee
    return vee * (angle / sine)


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
        self.collision_blocked = False
        arm_names = {"base_link", "shoulder_link", "upperarm_link", "forearm_link",
                     "wrist1_link", "wrist2_link", "wrist3_link"}
        hand_names = {"rh_hand", "r_thumb1_Link", "r_thumb2_Link", "r_thumb3_Link", "r_thumb4_Link",
                      "r_index1_Link", "r_index2_Link", "r_middle1_Link", "r_middle2_Link",
                      "r_ring1_Link", "r_ring2_Link", "r_little1_Link", "r_little2_Link"}
        object_names = {"grasp_bottle", "grasp_bottle_green", "grasp_box_yellow"}
        self.arm_body_ids = {mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                             for name in arm_names}
        self.hand_body_ids = {mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                              for name in hand_names}
        self.object_body_ids = {mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                                for name in object_names}
        self.prop_qpos = {}
        for name in object_names:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            joint_id = int(self.model.body_jntadr[body_id])
            qadr = int(self.model.jnt_qposadr[joint_id])
            self.prop_qpos[name] = (qadr, self.data.qpos[qadr:qadr + 7].copy())
        self.arm_body_ids.discard(-1); self.hand_body_ids.discard(-1); self.object_body_ids.discard(-1)
        mujoco.mj_forward(self.model, self.data)
        # Keep the manual viewer at the authored home pose.  In static mode
        # the MuJoCo control sliders can then drive the position actuators.
        for index, value in enumerate(self.arm_target):
            self.data.ctrl[self.actuator_id(f"fr5_j{index + 1}")] = value
        # The mounted IH01 palm reference is the simulated tool center point
        # (TCP). Quest wrist targets are anchored against this pose, not a
        # raw FR5 joint or flange pose.
        self.anchor_mm = self.tcp_pose_mm_deg()

    def set_grasp_scene(self, enabled: bool) -> None:
        """Show and enable grasp props only in teleoperation scenes."""
        for name in ("grasp_bottle_body_geom", "grasp_bottle_neck_geom", "grasp_bottle_cap_geom",
                     "grasp_bottle_green_body_geom", "grasp_bottle_green_neck_geom", "grasp_bottle_green_cap_geom",
                     "grasp_box_yellow_geom"):
            geom_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_GEOM, name)
            if geom_id < 0 or enabled:
                continue
            self.model.geom_rgba[geom_id, 3] = 0.0
            self.model.geom_contype[geom_id] = 0
            self.model.geom_conaffinity[geom_id] = 0

    def reset_ready(self) -> None:
        """Return the simulated arm and hand to the authored ready pose."""
        self.data.qpos[:6] = ARM_HOME
        self.data.qvel[:6] = 0.0
        self.arm_target = list(ARM_HOME)
        self.hand_target_steps = [0] * 6
        self.collision_blocked = False
        # Ready also restores the grasp scene so repeated trials start from a
        # known upright bottle and box without a separate UI button.
        for qadr, initial in self.prop_qpos.values():
            self.data.qpos[qadr:qadr + 7] = initial
            self.data.qvel[qadr:qadr + 6] = 0.0
        for index, value in enumerate(ARM_HOME):
            self.data.ctrl[self.actuator_id(f"fr5_j{index + 1}")] = value
        self.mujoco.mj_forward(self.model, self.data)

    def move_prop(self, name: str, axis: int, delta: float) -> None:
        """Nudge a free grasp prop in world XYZ for manual scene setup."""
        qadr, _ = self.prop_qpos[name]
        self.data.qpos[qadr + axis] += float(delta)
        self.data.qvel[qadr:qadr + 6] = 0.0
        self.mujoco.mj_forward(self.model, self.data)

    def actuator_id(self, name: str) -> int:
        value = self.mujoco.mj_name2id(
            self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, name
        )
        if value < 0:
            raise RuntimeError(f"missing actuator {name}")
        return value

    def tcp_pose_mm_deg(self) -> tuple[float, ...]:
        """Return the mounted IH01 palm/TCP pose in mm and FR5 RPY degrees."""
        self.mujoco.mj_forward(self.model, self.data)
        xyz = tuple(float(value) * 1000.0 for value in self.data.xpos[self.hand_body])
        rpy = _matrix_to_rpy_deg(self.np, self.data.xmat[self.hand_body])
        return xyz + rpy

    def current_pose_mm_deg(self) -> tuple[float, ...]:
        """Backward-compatible alias for the mounted tool TCP pose."""
        return self.tcp_pose_mm_deg()

    def solve_pose_ik(self, target_xyz_m, target_rpy_deg, iterations: int = 50) -> tuple[float, float]:
        np, mujoco = self.np, self.mujoco
        target = np.asarray(target_xyz_m, dtype=float)
        target_rotation = _rpy_deg_to_matrix(np, target_rpy_deg)
        self.collision_blocked = False
        jacp = np.zeros((3, self.model.nv)); jacr = np.zeros((3, self.model.nv))
        for _ in range(iterations):
            mujoco.mj_forward(self.model, self.data)
            position_error = target - self.data.xpos[self.hand_body]
            rotation_error = _rotation_error(np, target_rotation,
                                             self.data.xmat[self.hand_body].reshape(3, 3))
            if float(np.linalg.norm(position_error)) < 0.0008 and float(np.linalg.norm(rotation_error)) < math.radians(0.5):
                break
            mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.hand_body)
            # A 0.15 m characteristic length balances millimetre translation
            # accuracy against wrist orientation without letting either term
            # dominate the six-axis solve.
            orientation_weight = 0.15
            jacobian = np.vstack((jacp[:, :6], orientation_weight * jacr[:, :6]))
            error = np.concatenate((position_error, orientation_weight * rotation_error))
            damping = 3.0e-3
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(6), error
            )
            delta = np.clip(delta, -0.06, 0.06)
            previous_qpos = self.data.qpos.copy()
            self.data.qpos[:6] += delta
            for index in range(6):
                lo, hi = self.model.jnt_range[index]
                self.data.qpos[index] = np.clip(self.data.qpos[index], lo, hi)
            mujoco.mj_forward(self.model, self.data)
            if self.has_forbidden_penetration():
                # Do not latch at the previous target. Find the largest safe
                # fraction of this IK increment so motion glides up to the
                # collision boundary and can continue tangentially next frame.
                accepted = False
                for fraction in (0.5, 0.25, 0.125, 0.0625, 0.03125):
                    self.data.qpos[:] = previous_qpos
                    self.data.qpos[:6] = previous_qpos[:6] + fraction * delta
                    for index in range(6):
                        lo, hi = self.model.jnt_range[index]
                        self.data.qpos[index] = np.clip(self.data.qpos[index], lo, hi)
                    mujoco.mj_forward(self.model, self.data)
                    if not self.has_forbidden_penetration():
                        accepted = True
                        break
                if not accepted:
                    self.data.qpos[:] = previous_qpos
                    mujoco.mj_forward(self.model, self.data)
                # A partial safe step is normal boundary following, not a
                # hard stop. Report protection only when no safe increment
                # could be applied at all.
                self.collision_blocked = not accepted
                break
        mujoco.mj_forward(self.model, self.data)
        self.arm_target = [float(value) for value in self.data.qpos[:6]]
        position_error = float(np.linalg.norm(target - self.data.xpos[self.hand_body]))
        rotation_error = float(np.linalg.norm(_rotation_error(
            np, target_rotation, self.data.xmat[self.hand_body].reshape(3, 3))))
        return position_error, rotation_error

    def has_forbidden_penetration(self) -> bool:
        """Reject only meaningful arm/hand/prop interpenetration.

        MuJoCo reports small negative distances while resolving mesh/capsule
        contacts. Those are expected; a 4 mm threshold filters solver and
        collision-mesh tolerance while still rejecting clear deep penetration.
        """
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            if contact.dist >= -0.004:
                continue
            body_a = int(self.model.geom_bodyid[contact.geom1])
            body_b = int(self.model.geom_bodyid[contact.geom2])
            kind_a = "arm" if body_a in self.arm_body_ids else "hand" if body_a in self.hand_body_ids else "object" if body_a in self.object_body_ids else "other"
            kind_b = "arm" if body_b in self.arm_body_ids else "hand" if body_b in self.hand_body_ids else "object" if body_b in self.object_body_ids else "other"
            # Hand/object contact is expected during grasping and is resolved
            # by MuJoCo contact forces. Only reject penetration involving the
            # FR5 arm itself; a light finger-to-bottle bump must not disarm
            # teleoperation.
            if {kind_a, kind_b} <= {"arm", "hand", "object"} and "arm" in {kind_a, kind_b}:
                return True
        return False

    def solve_position_ik(self, target_xyz_m, iterations: int = 35) -> float:
        current_rpy = _matrix_to_rpy_deg(self.np, self.data.xmat[self.hand_body])
        error, _ = self.solve_pose_ik(target_xyz_m, current_rpy, iterations)
        return error

    def apply_targets(self, target: dict) -> float:
        pose = target.get("arm_pose_mm_deg")
        if pose is None:
            return math.inf
        # Online targets are already slew-limited and close to the previous
        # pose. A bounded iteration budget avoids starving the viewer when a
        # burst of network frames arrives; the full budget remains available
        # to the offline validation path.
        # Online IK runs every viewer tick. Keep the iteration budget small so
        # rendering and Quest polling remain responsive; offline validation
        # still uses the full solver budget.
        # IK is a target planner here. Preserve the physical state and let the
        # position actuators move the arm continuously toward the solution.
        # Directly leaving IK's qpos in the live state teleported the fingers
        # every Quest frame and ejected/slipped grasped objects.
        physical_qpos = self.data.qpos[:6].copy()
        physical_qvel = self.data.qvel[:6].copy()
        error, self.last_rotation_error_rad = self.solve_pose_ik(
            [float(value) / 1000.0 for value in pose[:3]], pose[3:6], iterations=10)
        planned_target = list(self.arm_target)
        self.data.qpos[:6] = physical_qpos
        self.data.qvel[:6] = physical_qvel
        self.arm_target = planned_target
        self.mujoco.mj_forward(self.model, self.data)
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
        # The physical FR5 drive holds position with gravity compensation.
        # A bare MuJoCo position spring produces zero torque at zero error and
        # therefore sags under gravity until an error develops. Compensating
        # the six arm bias forces models the enabled servo hold while leaving
        # contacts, the hand, floor and free grasp objects fully dynamic.
        self.mujoco.mj_forward(self.model, self.data)
        self.data.qfrc_applied[:6] = self.data.qfrc_bias[:6]
        self.mujoco.mj_step(self.model, self.data)


class QuestTcpSource:
    def __init__(self, host: str, port: int) -> None:
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port)); self.server.listen(1); self.server.setblocking(False)
        self.connection = None; self.buffer = ""; self.assembler = HandStreamAssembler()
        self.last_frame_at = 0.0
        print(f"SIM LISTEN tcp://{host}:{port}", flush=True)

    def poll(self):
        if self.connection is None:
            try:
                self.connection, address = self.server.accept()
                self.connection.setblocking(False)
                print(f"SIM CONNECTED source={address}", flush=True)
            except BlockingIOError:
                return []
        chunks = []; closed = False
        try:
            for _ in range(8):
                payload = self.connection.recv(65536)
                if not payload:
                    closed = True
                    break
                chunks.append(payload)
        except BlockingIOError:
            pass
        if not chunks:
            if closed:
                self.connection.close(); self.connection = None; self.buffer = ""
            return []
        self.buffer += b"".join(chunks).decode("utf-8", errors="replace")
        lines = self.buffer.split("\n"); self.buffer = lines.pop(); latest = {}
        for line in lines:
            frame = self.assembler.feed(line)
            if frame is not None:
                # If IK or rendering falls behind the Quest stream, replaying
                # every buffered frame creates visible catch-up jumps. Keep
                # only the newest complete frame per hand for this tick.
                latest[frame.side] = frame; self.last_frame_at = time.monotonic()
        if closed:
            self.connection.close(); self.connection = None; self.buffer = ""
        return list(latest.values())

    def close(self) -> None:
        if self.connection is not None: self.connection.close()
        self.server.close()


class PalmCameraPip:
    """Always-on eye-in-hand preview embedded in the viewer bottom-right."""

    def __init__(self, mujoco, model, width: int = 400, height: int = 260,
                 fps: float = 15.0) -> None:
        self.np = np
        self.mujoco = mujoco
        self.width = width
        self.height = height
        self.period = 1.0 / fps
        self.last_frame = 0.0
        self.renderer = None
        try:
            self.renderer = mujoco.Renderer(model, height=height, width=width)
            print("SIM CAMERA PIP ready: embedded IH01 palm view", flush=True)
        except Exception as exc:
            print(f"WARN palm camera PIP disabled: {exc}", flush=True)
            self.close()

    def update(self, viewer, data, now: float) -> None:
        if self.renderer is None or now - self.last_frame < self.period:
            return
        try:
            self.renderer.update_scene(data, camera="ih01_palm_camera")
            rgb = self.renderer.render()
            # Match the operator's preferred upright orientation for the
            # palm-camera inset without changing the physical camera pose.
            rgb = self.np.rot90(rgb, 2).copy()
            viewport = viewer.viewport
            margin = 8
            rect = self.mujoco.MjrRect(
                max(0, viewport.width - self.width - margin), margin,
                self.width, self.height)
            viewer.set_images((rect, rgb))
            self.last_frame = now
        except Exception as exc:
            print(f"WARN palm camera PIP stopped: {exc}", flush=True)
            self.close()

    def close(self) -> None:
        if self.renderer is not None:
            try: self.renderer.close()
            except Exception: pass
            self.renderer = None


class ArmTeleopControl:
    """Fail-closed, file-backed control shared with the local Tk panel."""
    def __init__(self, path: Path) -> None:
        self.path = path
        self.status_path = Path(f"{path}.status")

    def read(self) -> dict:
        try:
            state = json.loads(self.path.read_text(encoding="utf-8"))
            heartbeat = float(state.get("heartbeat", 0.0))
            state["panel_fresh"] = time.time() - heartbeat <= 0.25
            return state
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return {"armed": False, "paused": False, "estop": True,
                    "panel_fresh": False}

    def write_status(self, payload: dict) -> None:
        temporary = Path(f"{self.status_path}.tmp")
        try:
            temporary.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
            temporary.replace(self.status_path)
        except OSError:
            pass


def _apply_panel_tuning(cfg: dict, state: dict) -> tuple:
    arm = cfg["arm"]; hand = cfg["hand"]
    mapping_mode = str(state.get("mapping_mode", cfg["quest"].get("mapping_mode", "right")))
    if mapping_mode not in {"left", "right", "both"}:
        mapping_mode = "right"
    cfg["quest"]["mapping_mode"] = mapping_mode
    scale = max(0.1, min(2.0, float(state.get("arm_scale", 1.6))))
    arm["translation_scale_mm_per_m"] = [1000.0 * scale] * 3
    arm["max_translation_speed_mm_s"] = max(10.0, min(300.0, float(state.get("arm_speed_mm_s", 220.0))))
    arm["max_rotation_speed_deg_s"] = max(5.0, min(180.0, float(state.get("rotation_speed_deg_s", 100.0))))
    rotation_gain = max(0.1, min(1.5, float(state.get("rotation_gain", 0.80))))
    arm["orientation_scale_deg_per_rad"] = [57.2958 * rotation_gain] * 3
    hand["mapping_gain"] = max(0.5, min(1.5, float(state.get("hand_gain", 1.1))))
    hand_speed = max(100.0, min(2000.0, float(state.get("hand_speed_steps_s", 1500.0))))
    hand["max_step_delta_per_frame"] = max(1, round(hand_speed / 30.0))
    return (mapping_mode, scale, arm["max_translation_speed_mm_s"], arm["max_rotation_speed_deg_s"],
            rotation_gain,
            hand["mapping_gain"], hand_speed)


def validate(model_path: Path = MODEL, steps: int = 50) -> dict:
    sim = CombinedSimulation(model_path)
    initial = sim.np.asarray(sim.anchor_mm[:3]) / 1000.0
    ik_error = sim.solve_position_ik(initial + sim.np.asarray([0.015, 0.0, 0.0]))
    pose = list(sim.tcp_pose_mm_deg()); pose[3] += 5.0
    pose_error, rotation_error = sim.solve_pose_ik(
        [float(value) / 1000.0 for value in pose[:3]], pose[3:])
    sim.hand_target_steps = [850, 850, 850, 850, 850, 650]
    # Run long enough for the hand actuator transient to settle before
    # evaluating whether the arm servo actually supports the pose.
    for _ in range(max(steps, 250)): sim.step()
    hold_error = max(abs(float(actual) - target)
                     for actual, target in zip(sim.data.qpos[:6], sim.arm_target))
    if not math.isfinite(ik_error) or ik_error > 0.006:
        raise RuntimeError(f"combined IK validation failed: {ik_error:.6f} m")
    if pose_error > 0.006 or rotation_error > math.radians(2.0):
        raise RuntimeError("combined 6D IK validation failed: "
                           f"{pose_error:.6f} m, {math.degrees(rotation_error):.3f} deg")
    if hold_error > 0.002:
        raise RuntimeError(f"FR5 gravity hold validation failed: {hold_error:.6f} rad")
    return {
        "model": model_path.name, "nq": sim.model.nq, "nu": sim.model.nu,
        "bodies": sim.model.nbody, "arm_dof": 6, "hand": "IH01-X1-R",
        "ik_error_m": round(ik_error, 6),
        "orientation_error_deg": round(math.degrees(rotation_error), 3),
        "gravity_hold_error_rad": round(hold_error, 7),
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
    parser.add_argument("--control-file", type=Path,
                        help="fail-closed arm teleop panel state file")
    parser.add_argument("--no-grasp-props", action="store_true",
                        help="hide and disable the grasp-test props")
    args = parser.parse_args(argv)
    if args.headless:
        result = validate(args.model, args.steps)
        print("PASS combined-sim", json.dumps(result, separators=(",", ":")))
        if args.listen:
            telemetry = run_headless_teleop(args.model, args.config, args.host,
                                             args.port, args.side, args.duration)
            print("PASS quest-sim", json.dumps(telemetry, separators=(",", ":")))
        return 0
    import mujoco.viewer
    sim = CombinedSimulation(args.model)
    sim.set_grasp_scene(not args.no_grasp_props)
    pipeline = None
    source = None
    cfg = None
    control = ArmTeleopControl(args.control_file) if args.control_file else None
    if args.listen:
        cfg = json.loads(args.config.read_text(encoding="utf-8"))
        cfg["quest"]["mapping_mode"] = args.side
        if control is None:
            pipeline = Pipeline(cfg, robot_anchor=sim.tcp_pose_mm_deg())
        source = QuestTcpSource(args.host, args.port)
    def key_callback(key: int) -> None:
        nonlocal selected_prop
        # MuJoCo passes GLFW key codes; printable keys retain their ASCII value.
        try:
            char = chr(key).lower()
        except (ValueError, OverflowError):
            char = ""
        # Scene setup remains available in arm-teleop mode. Previously the
        # control-panel guard returned before these keys were handled.
        if char in {"v", "b", "n"}:
            selected_prop = {"v": "grasp_bottle", "b": "grasp_bottle_green",
                             "n": "grasp_box_yellow"}[char]
            print(f"SIM PROP SELECT {selected_prop}", flush=True)
            return
        if char in {"i", "k", "j", "l", "u", "m"}:
            axis_delta = {"i": (0, 0.02), "k": (0, -0.02),
                          "j": (1, 0.02), "l": (1, -0.02),
                          "u": (2, 0.02), "m": (2, -0.02)}[char]
            sim.move_prop(selected_prop, *axis_delta)
            qadr, _ = sim.prop_qpos[selected_prop]
            xyz = sim.data.qpos[qadr:qadr + 3]
            print(f"SIM PROP {selected_prop} xyz={list(xyz)}", flush=True)
            return
        if control is not None:
            if char:
                print("SIM CONTROL: use the teleop panel for E, Space pause and E-stop",
                      flush=True)
            return
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
            sim.reset_ready()
            print("SIM RESET READY + PROPS", flush=True)
        elif char == "o":
            sim.hand_target_steps = [0] * 6
        elif char == "c":
            sim.hand_target_steps = list(HAND_STEPS)

    sim.selected_joint = 0
    selected_prop = "grasp_bottle"
    sim.paused = False
    print("SIM CONTROLS: q/a J1, w/s J2, e/d J3, r/f J4, t/g J5, y/h J6; "
          "v/b/n select blue/green/box, i/k X, j/l Y, u/m Z (20 mm); "
          "0 Ready+reset props; o open; c close; SPACE pause", flush=True)
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=key_callback) as viewer:
        viewer.cam.distance=1.45; viewer.cam.azimuth=135; viewer.cam.elevation=-18; viewer.cam.lookat[:]=(0,0,0.35)
        started=time.monotonic(); last_target = None; last_target_pose = None; last_ik_error = None; last_report = 0.0; last_state = 0.0
        last_control_status = 0.0
        control_active = control is None
        tuning_signature = None
        ready_sequence = -1
        prop_move_sequence = None
        palm_pip = PalmCameraPip(mujoco, sim.model) if args.listen else None
        try:
            while viewer.is_running():
                panel_state = control.read() if control is not None else {}
                now = time.monotonic()
                quest_fresh = bool(source is not None and source.connection is not None and
                                   now - source.last_frame_at <= float(cfg["quest"].get("frame_timeout_s", 0.35)))
                requested_active = bool(control is None or (
                    panel_state.get("panel_fresh") and panel_state.get("armed") and
                    not panel_state.get("paused") and not panel_state.get("estop") and quest_fresh))
                if control is not None:
                    requested_ready = int(panel_state.get("ready_sequence", 0))
                    if requested_ready != ready_sequence:
                        sim.reset_ready()
                        pipeline = None
                        control_active = False
                        ready_sequence = requested_ready
                    requested_prop_move = int(panel_state.get("prop_move_sequence", 0))
                    if prop_move_sequence is None:
                        prop_move_sequence = requested_prop_move
                    elif requested_prop_move != prop_move_sequence:
                        prop_name = str(panel_state.get("prop_name", "grasp_bottle"))
                        axis = int(panel_state.get("prop_move_axis", 0))
                        delta = float(panel_state.get("prop_move_delta_m", 0.0))
                        if prop_name in sim.prop_qpos and axis in (0, 1, 2) and abs(delta) <= 0.05:
                            sim.move_prop(prop_name, axis, delta)
                            print(f"SIM PROP MOVE {prop_name} axis={axis} delta={delta:.3f}m",
                                  flush=True)
                        prop_move_sequence = requested_prop_move
                    signature = _apply_panel_tuning(cfg, panel_state)
                    if requested_active and (not control_active or signature != tuning_signature):
                        # Clutch engagement and tuning changes always establish
                        # a fresh wrist/robot anchor, preventing target jumps.
                        pipeline = Pipeline(cfg, robot_anchor=sim.tcp_pose_mm_deg())
                    elif not requested_active:
                        pipeline = None
                        if control_active:
                            # Disarming must stop at the current physical
                            # pose. Retaining the previous IK target would
                            # make the arm continue toward an old Quest pose
                            # after E/Space or a Quest timeout.
                            sim.arm_target = [float(value) for value in sim.data.qpos[:6]]
                    tuning_signature = signature
                    control_active = requested_active
                if sim.paused:
                    sim.mujoco.mj_forward(sim.model, sim.data)
                elif source is None and args.auto_hand:
                    closure=int((0.5+0.45*math.sin(time.monotonic()-started))*1300)
                    sim.hand_target_steps=[closure]*6
                    sim.step()
                elif source is not None:
                    if control is not None and not control_active:
                        # Keep the FR5 kinematically still while disarmed.
                        # Props may continue to settle, but stale IK targets or
                        # contact impulses must never make the arm jump before
                        # E explicitly enables teleoperation.
                        sim.arm_target = [float(value) for value in sim.data.qpos[:6]]
                        sim.data.qvel[:6] = 0.0
                        for index, value in enumerate(sim.arm_target):
                            sim.data.ctrl[sim.actuator_id(f"fr5_j{index + 1}")] = value
                    for frame in source.poll():
                        target=pipeline.process(frame) if pipeline is not None else None
                        if target:
                            last_target_pose = tuple(float(value) for value in target["arm_pose_mm_deg"])
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
                        "collision_blocked": bool(sim.collision_blocked),
                    }, separators=(",", ":")), flush=True)
                    last_report = now
                if source is None and now - last_state >= 0.5:
                    sim.mujoco.mj_forward(sim.model, sim.data)
                    joints = [round(float(value), 3) for value in sim.data.qpos[:6]]
                    print("SIM STATE", json.dumps({"joints_rad": joints,
                          "ih01_xyz_mm": [round(value, 1) for value in sim.hand_position_mm()]}, separators=(",", ":")), flush=True)
                    last_state = now
                if control is not None and now - last_control_status >= 0.1:
                    control.write_status({
                        "quest_connected": bool(source.connection is not None),
                        "quest_fresh": quest_fresh,
                        "motion_active": control_active,
                        "paused": sim.paused,
                        "real_fr5_output": False,
                        "joints_rad": [round(float(value), 4) for value in sim.data.qpos[:6]],
                        "target_pose_mm_deg": ([round(value, 2) for value in last_target_pose]
                                               if last_target_pose is not None else None),
                        "ik_error_mm": round(last_ik_error or 0.0, 3),
                        "collision_blocked": bool(sim.collision_blocked),
                    })
                    last_control_status = now
                if palm_pip is not None:
                    palm_pip.update(viewer, sim.data, now)
                viewer.sync(); time.sleep(sim.model.opt.timestep)
        finally:
            if palm_pip is not None: palm_pip.close()
            if source is not None: source.close()
    return 0


if __name__ == "__main__": raise SystemExit(main())
