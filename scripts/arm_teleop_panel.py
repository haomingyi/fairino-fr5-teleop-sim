#!/usr/bin/env python3
"""Fail-closed clutch panel for Quest -> FR5/IH01 combined simulation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk


class ArmTeleopPanel(tk.Tk):
    def __init__(self, control_path: Path, side: str) -> None:
        super().__init__()
        self.control_path = control_path
        self.status_path = Path(f"{control_path}.status")
        self.title("FR5 + IH01 遥操面板")
        self.geometry("620x790")
        self.minsize(580, 740)
        self.armed = False
        self.paused = False
        self.estop = False
        self.ready_sequence = 0
        self.prop_move_sequence = 0
        self.prop_name = tk.StringVar(value="grasp_bottle")
        self.prop_move_axis = 0
        self.prop_move_delta_m = 0.0
        self.arm_scale = tk.DoubleVar(value=1.60)
        self.arm_speed = tk.DoubleVar(value=220.0)
        self.rotation_speed = tk.DoubleVar(value=100.0)
        self.rotation_gain = tk.DoubleVar(value=0.80)
        self.hand_gain = tk.DoubleVar(value=1.10)
        self.hand_speed = tk.DoubleVar(value=1500.0)
        self.real_confirmed = tk.BooleanVar(value=False)
        self.mapping_mode = tk.StringVar(value=side)
        self.keys_down: set[str] = set()
        self.connection = tk.StringVar(value="Quest：等待数据 | FR5 真机：锁定")
        self.motion = tk.StringVar(value="未开启遥操：按 E 开始")
        self.joints = tk.StringVar(value="J1–J6：—")
        self.target_pose = tk.StringVar(value="末端目标 XYZ / RPY：—")
        self._build()
        self.bind_all("<KeyPress>", self._key_press)
        self.bind_all("<KeyRelease>", self._key_release)
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(300, self._claim_keyboard_focus)
        self.after(50, self._tick)

    def _claim_keyboard_focus(self) -> None:
        try:
            self.deiconify(); self.lift(); self.focus_force()
        except tk.TclError:
            pass

    def _build(self) -> None:
        root = ttk.Frame(self, padding=14)
        root.pack(fill=tk.BOTH, expand=True)
        ttk.Label(root, text="Quest 3 → FR5 + IH01", font=("Sans", 17, "bold")).pack(anchor=tk.W)
        ttk.Label(root, textvariable=self.connection).pack(anchor=tk.W, pady=(4, 2))
        ttk.Label(root, textvariable=self.motion, foreground="#185f8d").pack(anchor=tk.W, pady=(0, 12))

        safety = ttk.LabelFrame(root, text="运行许可（默认不运动）", padding=10)
        safety.pack(fill=tk.X)
        row = ttk.Frame(safety); row.pack(fill=tk.X)
        self.arm_button = ttk.Button(row, text="E：开始遥操", command=self._toggle_arm)
        self.arm_button.pack(side=tk.LEFT, padx=(0, 8))
        self.hold_button = tk.Button(row, text="空格：回到 Ready", width=18,
                                     bg="#d7e8f5", relief=tk.RAISED)
        self.hold_button.pack(side=tk.LEFT, padx=8)
        self.hold_button.configure(command=self._request_ready)
        tk.Button(row, text="急停", command=self._emergency_stop, width=9,
                  bg="#b3261e", fg="white", activebackground="#7f1d1d").pack(side=tk.RIGHT)
        ttk.Button(safety, text="复位急停（仍保持未布防）", command=self._reset_estop).pack(anchor=tk.W, pady=(9, 3))
        ttk.Checkbutton(safety, variable=self.real_confirmed,
                        text="我确认真实 FR5、物理急停、TCP 和负载均已检查").pack(anchor=tk.W, pady=(5, 0))
        ttk.Label(safety, text="提示：此版本 arm-teleop 仍是仿真输出；勾选不会解锁真机。",
                  foreground="#8a4b08").pack(anchor=tk.W, pady=(3, 0))

        tuning = ttk.LabelFrame(root, text="遥操参数", padding=10)
        tuning.pack(fill=tk.X, pady=12)
        mapping = ttk.Frame(tuning); mapping.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(mapping, text="映射手", width=27).pack(side=tk.LEFT)
        for value, label in (("right", "右"), ("left", "左"), ("both", "双手")):
            ttk.Radiobutton(mapping, text=label, variable=self.mapping_mode,
                            value=value).pack(side=tk.LEFT, padx=(0, 12))
        self._slider(tuning, "手臂位移比例", self.arm_scale, 0.20, 2.00,
                     "1.60 = 手移动 10 cm，末端目标移动 16 cm；2.00 为 1:2")
        self._slider(tuning, "手臂平移速度 (mm/s)", self.arm_speed, 20, 300, "")
        self._slider(tuning, "腕部旋转速度 (deg/s)", self.rotation_speed, 10, 180, "")
        self._slider(tuning, "腕部旋转比例", self.rotation_gain, 0.20, 1.20,
                     "0.80 = 手腕旋转 10°，末端目标约旋转 8°")
        self._slider(tuning, "灵巧手行程比例", self.hand_gain, 0.60, 1.40,
                     "提高可让握拳更容易到满行程")
        self._slider(tuning, "灵巧手跟随速度 (steps/s)", self.hand_speed, 200, 2000, "")

        scene = ttk.LabelFrame(root, text="仿真物体摆放（每次 20 mm）", padding=10)
        scene.pack(fill=tk.X, pady=(0, 12))
        select = ttk.Frame(scene); select.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(select, text="选择物体", width=12).pack(side=tk.LEFT)
        for value, label in (("grasp_bottle", "蓝瓶"),
                             ("grasp_bottle_green", "绿瓶"),
                             ("grasp_box_yellow", "黄盒")):
            ttk.Radiobutton(select, text=label, variable=self.prop_name,
                            value=value).pack(side=tk.LEFT, padx=(0, 12))
        controls = ttk.Frame(scene); controls.pack(fill=tk.X)
        for text, axis, delta in (("X+", 0, 0.02), ("X−", 0, -0.02),
                                  ("Y+", 1, 0.02), ("Y−", 1, -0.02),
                                  ("Z+", 2, 0.02), ("Z−", 2, -0.02)):
            ttk.Button(controls, text=text, width=6,
                       command=lambda a=axis, d=delta: self._move_prop(a, d)).pack(
                           side=tk.LEFT, padx=(0, 5))
        ttk.Label(scene, text="手心摄像头会始终显示在 MuJoCo 主画面右下角。",
                  foreground="#666666").pack(anchor=tk.W, pady=(8, 0))

        telemetry = ttk.LabelFrame(root, text="状态", padding=10)
        telemetry.pack(fill=tk.X)
        ttk.Label(telemetry, textvariable=self.joints).pack(anchor=tk.W)
        ttk.Label(telemetry, textvariable=self.target_pose).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(telemetry,
                  text="流程：按 E 开始/暂停/继续 → 空格回到 Ready（解除遥操）。\n"
                       "Quest 断流、面板关闭或心跳超时会停止更新。",
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(8, 0))

    def _slider(self, parent, label, variable, lower, upper, hint) -> None:
        row = ttk.Frame(parent); row.pack(fill=tk.X, pady=4)
        ttk.Label(row, text=label, width=27).pack(side=tk.LEFT)
        ttk.Scale(row, variable=variable, from_=lower, to=upper).pack(side=tk.LEFT, fill=tk.X, expand=True)
        value = ttk.Label(row, width=7)
        value.pack(side=tk.RIGHT, padx=(8, 0))
        def refresh(*_args): value.configure(text=f"{variable.get():.2f}")
        variable.trace_add("write", refresh); refresh()
        if hint:
            ttk.Label(parent, text=hint, foreground="#666666").pack(anchor=tk.W, padx=(170, 0))

    def _toggle_arm(self) -> None:
        if self.estop:
            return
        if not self.armed:
            self.armed, self.paused = True, False
        else:
            self.paused = not self.paused
        self._toggle_arm_button()

    def _move_prop(self, axis: int, delta_m: float) -> None:
        self.prop_move_axis = int(axis)
        self.prop_move_delta_m = float(delta_m)
        self.prop_move_sequence += 1

    def _request_ready(self) -> None:
        """Request a local simulation reset while keeping teleop disarmed."""
        self.ready_sequence += 1
        self.armed = False
        self.paused = False
        self._toggle_arm_button()
        self._toggle_pause_button()

    def _toggle_arm_button(self) -> None:
        if not self.armed:
            text = "E：开始遥操"
        elif self.paused:
            text = "E：继续遥操"
        else:
            text = "E：暂停遥操"
        self.arm_button.configure(text=text)

    def _toggle_pause(self) -> None:
        """Compatibility hook: Space is a deterministic Ready action."""
        self._request_ready()

    def _emergency_stop(self) -> None:
        self.estop = True; self.armed = False; self.paused = False
        self._toggle_arm_button()

    def _reset_estop(self) -> None:
        self.estop = False; self.armed = False; self.paused = False
        self._toggle_arm_button()
        self._toggle_pause_button()

    def _toggle_pause_button(self) -> None:
        self.hold_button.configure(text="空格：回到 Ready",
                                   relief=tk.RAISED, bg="#d7e8f5")

    def _key_press(self, event) -> None:
        key = str(event.keysym).lower()
        if key in self.keys_down:
            return
        self.keys_down.add(key)
        if key == "e": self._toggle_arm()
        elif key == "space": self._toggle_pause()
        elif key == "escape": self._emergency_stop()

    def _key_release(self, event) -> None:
        key = str(event.keysym).lower()
        self.keys_down.discard(key)

    def _write_control(self) -> None:
        payload = {
            "format": "fr5_arm_teleop_control_v1", "heartbeat": time.time(),
            "armed": self.armed, "paused": self.paused, "estop": self.estop,
            "ready_sequence": self.ready_sequence,
            "prop_move_sequence": self.prop_move_sequence,
            "prop_name": self.prop_name.get(),
            "prop_move_axis": self.prop_move_axis,
            "prop_move_delta_m": self.prop_move_delta_m,
            "mapping_mode": self.mapping_mode.get(),
            "arm_scale": self.arm_scale.get(), "arm_speed_mm_s": self.arm_speed.get(),
            "rotation_speed_deg_s": self.rotation_speed.get(),
            "rotation_gain": self.rotation_gain.get(),
            "hand_gain": self.hand_gain.get(), "hand_speed_steps_s": self.hand_speed.get(),
            "real_fr5_confirmed": self.real_confirmed.get(),
        }
        temporary = Path(f"{self.control_path}.tmp")
        temporary.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
        temporary.replace(self.control_path)

    def _read_status(self) -> None:
        try:
            status = json.loads(self.status_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError):
            status = {}
        quest = "已连接" if status.get("quest_fresh") else "等待数据"
        self.connection.set(f"Quest：{quest} | FR5 真机：锁定（仿真输出）")
        if self.estop: message = "急停已锁存：仿真输出停止"
        elif self.paused: message = "已暂停：按 E 继续，按空格回到 Ready"
        elif status.get("collision_blocked"): message = "碰撞保护：目标被拒绝，已保持当前位置"
        elif status.get("motion_active"): message = "遥操中：Quest 正在跟随"
        elif self.armed and not status.get("quest_fresh"): message = "已开启：等待 Quest 数据（请打开 TCP 应用）"
        elif self.armed: message = "已开启：正在等待有效目标"
        else: message = "未开启遥操：按 E 开始"
        self.motion.set(message)
        joints = status.get("joints_rad")
        if joints:
            self.joints.set("  ".join(f"J{i + 1}={value:.3f}" for i, value in enumerate(joints)))
        target = status.get("target_pose_mm_deg")
        if target and len(target) == 6:
            self.target_pose.set(
                "末端目标  " + "  ".join(
                    f"{name}={value:.1f}" for name, value in
                    zip(("X", "Y", "Z", "R", "P", "Yaw"), target)
                )
            )

    def _tick(self) -> None:
        try: self._write_control()
        except OSError: pass
        self._read_status()
        self.after(50, self._tick)

    def _close(self) -> None:
        self.estop = True; self.armed = False; self.paused = False
        try: self._write_control()
        except OSError: pass
        self.destroy()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--control-file", type=Path, required=True)
    parser.add_argument("--side", choices=("left", "right", "both"), default="right")
    args = parser.parse_args()
    ArmTeleopPanel(args.control_file, args.side).mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
