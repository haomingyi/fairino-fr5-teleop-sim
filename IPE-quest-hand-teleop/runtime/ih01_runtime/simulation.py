"""MuJoCo runtime adapter for the dual IH01 dexterous-hand model.

Author: haoming
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Any

import numpy as np

from .retargeting import active_dof_from_state

PROJECT_ROOT = Path(__file__).resolve().parents[2]
IH01_X1_MODEL = PROJECT_ROOT / "simulation" / "ih01_x1_dual.xml"
IH01_X1_CONFIG = PROJECT_ROOT / "config" / "ih01_x1.json"
OPEN_TARGETS = {
    "thumb": {"cmc": 0.0, "mcp": 0.0, "ip": 0.0},
    "index": {"mcp": 0.0, "pip": 0.0, "dip": 0.0},
    "middle": {"mcp": 0.0, "pip": 0.0, "dip": 0.0},
    "ring": {"mcp": 0.0, "pip": 0.0, "dip": 0.0},
    "pinky": {"mcp": 0.0, "pip": 0.0, "dip": 0.0},
}


class IH01Simulation:
    """Offscreen MuJoCo renderer for the URDF-derived IH01-X1-L/R model."""

    def __init__(self, *, render: bool = True) -> None:
        if render:
            os.environ.setdefault("MUJOCO_GL", "egl")
        import mujoco

        self._mujoco = mujoco
        self.ih01_config = json.loads(IH01_X1_CONFIG.read_text(encoding="utf-8"))
        self.model = mujoco.MjModel.from_xml_path(str(IH01_X1_MODEL))
        self.data = mujoco.MjData(self.model)
        self.renderer = (
            mujoco.Renderer(self.model, height=540, width=960) if render else None
        )
        self.camera = mujoco.MjvCamera()
        self.set_camera_view("oblique")
        self._last_update_s = time.monotonic()
        self.last_seen_s = {"Left": 0.0, "Right": 0.0}
        self.last_surface = {"Left": "UNKNOWN", "Right": "UNKNOWN"}
        self.last_active_targets: dict[str, dict[str, float]] = {}
        self._mocap_ids = {}
        self._mocap_base_positions = {}
        self._mocap_base_quaternions = {}
        self._depth_references_m: dict[str, float] = {}
        self._wrist_references_m: dict[str, np.ndarray] = {}
        self.fixed_wrist = bool(
            self.ih01_config.get("simulation_model", {}).get("fixed_wrist", True)
        )
        for handedness, body_name in (("Left", "lh_hand"), ("Right", "rh_hand")):
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body_name
            )
            mocap_id = int(self.model.body_mocapid[body_id])
            self._mocap_ids[handedness] = mocap_id
            self._mocap_base_positions[handedness] = self.data.mocap_pos[
                mocap_id
            ].copy()
            self._mocap_base_quaternions[handedness] = self.data.mocap_quat[
                mocap_id
            ].copy()

    def set_camera_view(self, view: str) -> None:
        views = {
            "top": (90.0, -82.0, 0.52, (0.0, 0.05, 0.12)),
            "oblique": (90.0, -12.0, 0.55, (0.0, 0.05, 0.12)),
            "side": (0.0, -8.0, 0.52, (0.0, 0.05, 0.12)),
        }
        if view not in views:
            raise ValueError(f"Unknown camera view: {view}")
        azimuth, elevation, distance, lookat = views[view]
        self.camera.azimuth = azimuth
        self.camera.elevation = elevation
        self.camera.distance = distance
        self.camera.lookat[:] = lookat

    def _retarget_orientation(
        self, handedness: str, raw_quaternion: np.ndarray
    ) -> np.ndarray:
        """Map the absolute camera palm frame onto the authored model frame.

        The vision quaternion axes are X=across palm, Y=toward fingers and
        Z=palm normal. The authored model at its neutral pose uses X=across,
        Y=palm normal and -Z=toward fingers. Its stored base quaternion is the
        fixed transform between those conventions.
        """
        raw = np.asarray(raw_quaternion, dtype=np.float64)
        raw /= max(float(np.linalg.norm(raw)), 1e-8)
        result = np.empty(4, dtype=np.float64)
        self._mujoco.mju_mulQuat(
            result, raw, self._mocap_base_quaternions[handedness]
        )
        result /= max(float(np.linalg.norm(result)), 1e-8)
        return result

    def _set_actuator(
        self,
        name: str,
        value: float,
        *,
        max_speed_rad_s: float | None = None,
        dt: float | None = None,
    ) -> None:
        actuator_id = self._mujoco.mj_name2id(
            self.model,
            self._mujoco.mjtObj.mjOBJ_ACTUATOR,
            name,
        )
        if actuator_id < 0:
            raise KeyError(f"MuJoCo actuator not found: {name}")
        lower, upper = self.model.actuator_ctrlrange[actuator_id]
        target = float(np.clip(value, lower, upper))
        if max_speed_rad_s is not None and dt is not None:
            current = float(self.data.ctrl[actuator_id])
            maximum_step = max_speed_rad_s * dt
            target = float(np.clip(target, current - maximum_step, current + maximum_step))
        self.data.ctrl[actuator_id] = target

    def _update_hand(self, state: dict[str, Any], dt: float) -> None:
        handedness = str(state["handedness"])
        prefix = "lh" if handedness == "Left" else "rh"
        active_targets = active_dof_from_state(state)
        self.last_active_targets[handedness] = dict(active_targets)
        specification = self.ih01_config["per_hand"]
        visual_speed_scale = float(
            self.ih01_config.get("vision_retargeting", {}).get(
                "simulation_speed_scale", 1.0
            )
        )
        four_finger_speed = math.radians(
            specification["four_finger_speed_deg_s"] * visual_speed_scale
        )
        thumb_speed = math.radians(
            specification["thumb_speed_deg_s"] * visual_speed_scale
        )
        mocap_id = self._mocap_ids[handedness]
        if self.fixed_wrist:
            # The physical IH01 wrist is bolted to its mount. Keep the MuJoCo
            # base at the authored installation pose and retarget fingers only.
            self.data.mocap_pos[mocap_id] = self._mocap_base_positions[handedness]
            self.data.mocap_quat[mocap_id] = self._mocap_base_quaternions[handedness]
        elif "palm_quaternion_wxyz" in state:
            self.data.mocap_quat[mocap_id] = self._retarget_orientation(
                handedness,
                np.asarray(state["palm_quaternion_wxyz"], dtype=np.float64),
            )
        elif "palm_roll_rad" in state:
            roll = float(np.clip(state["palm_roll_rad"], -1.25, 1.25))
            self.data.mocap_quat[mocap_id] = (
                math.cos(roll / 2.0),
                0.0,
                0.0,
                math.sin(roll / 2.0),
            )
        if not self.fixed_wrist and "wrist_position_m" in state:
            wrist = np.asarray(state["wrist_position_m"], dtype=np.float64)
            if wrist.shape != (3,):
                raise ValueError("wrist_position_m must contain three values")
            reference = self._wrist_references_m.setdefault(handedness, wrist.copy())
            # HTS wrist coordinates are already metric. Anchor the first
            # tracked frame to the authored hand mount, then follow Quest
            # translation 1:1 with a conservative visual workspace limit.
            offset = np.clip(wrist - reference, -0.22, 0.22)
            self.data.mocap_pos[mocap_id] = self._mocap_base_positions[handedness] + offset
        elif not self.fixed_wrist and "palm_position_xy" in state:
            image_x, image_y = state["palm_position_xy"]
            base = self._mocap_base_positions[handedness]
            # Preserve the two-hand layout while making camera-plane wrist
            # movement visible. This is display retargeting, not metric pose.
            offset_x = float(np.clip((image_x - 0.5) * 0.18, -0.075, 0.075))
            offset_y = float(np.clip((0.5 - image_y) * 0.16, -0.065, 0.065))
            offset_z = float(self.data.mocap_pos[mocap_id, 2] - base[2])
            if "palm_depth_m" in state:
                depth_m = float(state["palm_depth_m"])
                reference = self._depth_references_m.setdefault(handedness, depth_m)
                # Real depth is metric; the reduced visualization gain keeps
                # both URDF-derived hands inside their side-by-side workspaces.
                offset_z = float(
                    np.clip((reference - depth_m) * 0.25, -0.080, 0.080)
                )
            self.data.mocap_pos[mocap_id] = base + (
                offset_x,
                offset_y,
                offset_z,
            )
        if "palm_surface" in state:
            self.last_surface[handedness] = str(state["palm_surface"])

        for finger in ("index", "middle", "ring", "pinky"):
            self._set_actuator(
                f"{prefix}_{finger}_flex",
                1.396263 * active_targets[f"{finger}_flex"],
                max_speed_rad_s=four_finger_speed,
                dt=dt,
            )

        self._set_actuator(
            f"{prefix}_thumb_opposition",
            # The corrected mirrored yaw axis moves toward the palm as its
            # joint angle increases. The semantic target therefore maps
            # directly from open=0 to inward opposition=0.70 rad.
            0.70 * active_targets["thumb_opposition"],
            max_speed_rad_s=thumb_speed,
            dt=dt,
        )
        self._set_actuator(
            f"{prefix}_thumb_flex",
            0.959931 * active_targets["thumb_flex"],
            max_speed_rad_s=thumb_speed,
            dt=dt,
        )

    def update_hands(self, states: list[dict[str, Any]]) -> np.ndarray | None:
        now = time.monotonic()
        dt = float(np.clip(now - self._last_update_s, 1.0 / 120.0, 0.1))
        self._last_update_s = now
        seen = set()
        for state in states:
            handedness = str(state["handedness"])
            if handedness not in self.last_seen_s:
                continue
            self._update_hand(state, dt)
            self.last_seen_s[handedness] = now
            seen.add(handedness)
        for handedness in ("Left", "Right"):
            last_seen = self.last_seen_s[handedness]
            if handedness not in seen and last_seen > 0.0 and now - last_seen > 0.75:
                self._update_hand(
                    {
                        "handedness": handedness,
                        "joint_flexion": OPEN_TARGETS,
                        "pinch_distance": 1.2,
                    },
                    dt,
                )
        return self.step(simulation_dt=dt)

    def tracking_status(self) -> dict[str, str]:
        now = time.monotonic()
        status = {}
        for handedness, last_seen in self.last_seen_s.items():
            if last_seen == 0.0:
                status[handedness] = "WAITING"
            elif now - last_seen < 0.20:
                status[handedness] = "TRACKING"
            elif now - last_seen <= 0.75:
                status[handedness] = "HOLD"
            else:
                status[handedness] = "RETURNING"
        return status

    def surface_status(self) -> dict[str, str]:
        return dict(self.last_surface)

    def active_target_status(self) -> dict[str, dict[str, float]]:
        return {hand: dict(values) for hand, values in self.last_active_targets.items()}

    def step(self, *, simulation_dt: float | None = None) -> np.ndarray | None:
        # Live vision arrives at about 30 Hz while the model timestep is 5 ms.
        # Advancing a fixed three steps only simulated 15 ms for every 33 ms of
        # wall time, making the hand visibly run at less than half real speed.
        target_dt = 3.0 * self.model.opt.timestep if simulation_dt is None else simulation_dt
        substeps = int(
            np.clip(math.ceil(target_dt / self.model.opt.timestep), 1, 24)
        )
        for _ in range(substeps):
            self._mujoco.mj_step(self.model, self.data)
        if self.renderer is None:
            return None
        self.renderer.update_scene(self.data, camera=self.camera)
        return self.renderer.render()

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
