#!/usr/bin/env python3
"""Small simulation dashboard for the FR5 + IH01 + Quest workflow.

The dashboard launches simulation-only make targets.  IH01 manual control is
delegated to the preserved IPE project and always asks for explicit consent.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import tkinter as tk
from tkinter import messagebox, ttk

ROOT = Path(__file__).resolve().parents[1]


class TeleopUi(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("FR5 + IH01 Quest 仿真控制台")
        self.geometry("850x590")
        self.process: subprocess.Popen[str] | None = None
        self.lines: queue.Queue[str] = queue.Queue()
        self.state = tk.StringVar(value="待机：仅仿真，不连接 FR5 真机")
        self.target = tk.StringVar(value="—")
        self.actual = tk.StringVar(value="—")
        self.error = tk.StringVar(value="—")
        self.ik_error = tk.StringVar(value="—")
        self.mapping_mode = tk.StringVar(value="right")
        self._build()
        self.after(100, self._drain)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build(self) -> None:
        root = ttk.Frame(self, padding=12)
        root.pack(fill=tk.BOTH, expand=True)
        ttk.Label(root, text="FR5 + IH01-X1-R / Quest 3", font=("Sans", 16, "bold")).pack(anchor=tk.W)
        ttk.Label(root, textvariable=self.state, foreground="#1a5276").pack(anchor=tk.W, pady=(5, 12))

        mapping = ttk.LabelFrame(root, text="Quest 映射", padding=8)
        mapping.pack(fill=tk.X, pady=(0, 10))
        for value, label in (("right", "右手：右腕 + 右手指"),
                             ("left", "左手：左腕 + 左手指"),
                             ("both", "双手：右腕控制 FR5，左手指控制 IH01")):
            ttk.Radiobutton(mapping, text=label, variable=self.mapping_mode, value=value).pack(side=tk.LEFT, padx=(0, 16))

        buttons = ttk.Frame(root)
        buttons.pack(fill=tk.X)
        ttk.Button(buttons, text="启动机械臂仿真", command=lambda: self._start("arm-sim")).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(buttons, text="机械臂 Quest 联动", command=lambda: self._start("arm-teleop")).pack(side=tk.LEFT, padx=6)
        ttk.Button(buttons, text="灵巧手 Quest 遥操", command=lambda: self._start("hand-teleop")).pack(side=tk.LEFT, padx=6)
        ttk.Button(buttons, text="停止", command=self._stop).pack(side=tk.LEFT, padx=6)
        ttk.Button(buttons, text="检查", command=lambda: self._start("check")).pack(side=tk.LEFT, padx=6)
        ttk.Button(buttons, text="灵巧手手动控制", command=self._hand_control).pack(side=tk.RIGHT)

        card = ttk.LabelFrame(root, text="实时仿真数据", padding=10)
        card.pack(fill=tk.X, pady=14)
        for label, value in (("Quest 映射目标 (mm)", self.target), ("IH01 仿真位置 (mm)", self.actual),
                             ("位置误差", self.error), ("数值 IK 误差", self.ik_error)):
            row = ttk.Frame(card); row.pack(fill=tk.X, pady=3)
            ttk.Label(row, text=label, width=25).pack(side=tk.LEFT)
            ttk.Label(row, textvariable=value).pack(side=tk.LEFT)

        ttk.Label(root, text="运行日志").pack(anchor=tk.W)
        self.log = tk.Text(root, height=17, wrap=tk.WORD, state=tk.DISABLED, background="#171717", foreground="#dddddd")
        self.log.pack(fill=tk.BOTH, expand=True)

    def _start(self, target: str) -> None:
        if self.process is not None and self.process.poll() is None:
            messagebox.showinfo("正在运行", "请先停止当前任务。")
            return
        side = self.mapping_mode.get()
        self._append(f"> make {target}  [映射={side}]\n")
        self.state.set("启动中…")
        # Keep the command line short; pass the UI selection through the
        # child environment so the target itself can remain interactive.
        env = dict(os.environ)
        env["SIDE"] = side
        self.process = subprocess.Popen(["make", target], cwd=ROOT, env=env, text=True,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        threading.Thread(target=self._read_output, daemon=True).start()

    def _read_output(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        for line in self.process.stdout:
            self.lines.put(line)
        self.lines.put("[process-exited]\n")

    def _drain(self) -> None:
        while True:
            try: line = self.lines.get_nowait()
            except queue.Empty: break
            self._append(line)
            if line.startswith("SIM TELEMETRY "):
                try:
                    data = json.loads(line.removeprefix("SIM TELEMETRY "))
                    self.target.set(str(data["target_xyz_mm"]))
                    self.actual.set(str(data["sim_ih01_xyz_mm"]))
                    self.error.set(f'{data["position_error_mm"]} mm')
                    self.ik_error.set(f'{data["ik_error_mm"]} mm')
                    self.state.set("Quest 数据正在驱动联合仿真")
                except (json.JSONDecodeError, KeyError):
                    pass
            elif "PASS combined-sim" in line:
                self.state.set("联合仿真已启动")
            elif line == "[process-exited]\n":
                self.state.set("已停止")
        self.after(100, self._drain)

    def _append(self, line: str) -> None:
        self.log.configure(state=tk.NORMAL); self.log.insert(tk.END, line); self.log.see(tk.END); self.log.configure(state=tk.DISABLED)

    def _stop(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate(); self.state.set("正在停止…")

    def _hand_control(self) -> None:
        if messagebox.askyesno("确认", "将启动 IH01 EtherCAT 手动控制台。确认急停可用且只连接预期从站？"):
            subprocess.Popen(["make", "hand-control", f"SIDE={self.mapping_mode.get()}"], cwd=ROOT)

    def _close(self) -> None:
        self._stop(); self.destroy()


if __name__ == "__main__":
    TeleopUi().mainloop()
