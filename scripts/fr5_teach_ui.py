#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FR5 操作面板：测通讯 → 使能 → 单关节 → 末端世界系平移/姿态 → 拖动。

安全：默认不下发运动；须勾选「确认真机」；速度/步长受 config 限幅；急停 ImmStopJOG+StopMotion。
用法：
  python3 scripts/fr5_teach_ui.py
"""

from __future__ import annotations

import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
SDK_LINUX = ROOT / "third_party" / "fairino-python-sdk" / "linux"

sys.path.insert(0, str(SDK_LINUX))
sys.path.insert(0, str(ROOT / "scripts"))

from fairino_session import (
    activate_world_wobj,
    call_err,
    cart_goto_async,
    connect_robot,
    goto_joints_by_jog,
    joints_violate_soft,
    prepare_motion,
    probe_ports,
    start_joint_jog,
    stop_jog,
    unwrap_pair,
)
from fr5_config import build_joint_limits, load_cfg
from soft_zero import default_path as soft_zero_default_path, load_soft_zero, save_soft_zero
from world_frame import base_pose_to_world, rotate_pose_about_axis, world_axis_unit_in_base


def ping_ok(ip: str, timeout_s: float = 1.0) -> bool:
    try:
        r = subprocess.run(
            ["ping", "-c", "1", "-W", str(max(1, int(timeout_s))), ip],
            capture_output=True,
            text=True,
            timeout=timeout_s + 2,
        )
        return r.returncode == 0
    except Exception:
        return False


class Fr5TeachApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("FAIRINO FR5 操作面板")
        self.geometry("920x720")
        self.cfg = load_cfg()
        self.ip = str(self.cfg["robot"]["default_ip"])
        self.tool = int(self.cfg["coords"]["tool_id_default"])
        self.user = int(self.cfg["coords"]["user_id_base"])
        self.world_yaw_cw = float(self.cfg["coords"].get("world_yaw_cw_deg", 45.0))
        self.world_wobj_id = int(self.cfg["coords"].get("world_wobj_id", 1))
        self.prefer_wobj_jog = bool(self.cfg["coords"].get("prefer_wobj_jog", True))
        self.wobj_jog_ready = False
        self.joint_hard_limits, self.joint_soft_limits, self.joint_soft_margin = build_joint_limits(self.cfg)
        self.speed = float(self.cfg["limits"]["global_speed_pct_default"])
        self.jog_vel = float(self.cfg["limits"]["jog_vel_pct_default"])
        self.jog_j_max = float(self.cfg["limits"].get("jog_max_dis_joint_deg", 30.0))
        self.slider_lead = float(self.cfg["limits"].get("slider_lead_deg", 15.0))
        self.jog_acc = float(self.cfg["limits"].get("jog_acc_pct_default", 100.0))
        self.jog_vel_max = float(self.cfg["limits"].get("jog_vel_pct_max_ui", 100.0))

        # SDK RPC 句柄；未连接前为 None。标 Any 避免分析器把字面 None 固化成不可调用类型。
        self.robot: Any = None
        self.connected = False
        self.degraded = False
        self.enabled = False
        self.drag_on = False
        self.allow_degrade = bool(self.cfg.get("allow_xmlrpc_degrade", True))
        self.prefer_xmlrpc_only = bool(self.cfg.get("prefer_xmlrpc_only", True))
        self.soft_zero_path = Path(str(self.cfg.get("soft_zero_file") or soft_zero_default_path(ROOT)))
        if not self.soft_zero_path.is_absolute():
            self.soft_zero_path = ROOT / self.soft_zero_path
        self.soft_zero = load_soft_zero(self.soft_zero_path)
        self._soft_zero_label = tk.StringVar(value=self._soft_zero_summary())

        self._jog_stop_flag = False
        self._slider_jog_dir = [0] * 6
        self._joint_grabbed = [False] * 6
        self._actual_cache = [0.0] * 6
        self._slider_tick_on = False
        self._actual_labels: list[ttk.Label] = []
        self._busy = False
        self._joint_vars: list[tk.DoubleVar] = []
        self._joint_labels: list[ttk.Label] = []
        self._suppress_joint = False
        self._action_buttons: list[Any] = []
        self._slider_scales: list[ttk.Scale] = []
        # 在 _build 前占位，满足基于过程的实例字段初始化检查
        self.var_jog_vel = tk.DoubleVar(value=self.jog_vel)
        self.lbl_jog_vel: Any = None
        self.scale_jog_vel: Any = None
        self.tcp_text = tk.StringVar(value="TCP: —")

        self._build()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(200, self._poll_status)

    # ---------- soft zero / header ----------
    def _soft_zero_summary(self) -> str:
        if not self.soft_zero:
            return "软零位: 未记录"
        j = self.soft_zero["joints_deg"]
        return "软零位: " + ", ".join(f"J{i+1}={j[i]:.1f}" for i in range(6))

    def _hdr_text(self, jog_hint: str | None = None) -> str:
        soft = "已记软零" if self.soft_zero else "无软零"
        base = f"IP {self.ip}  |  世界系 CW {self.world_yaw_cw:.0f}°  wobj#{self.world_wobj_id}"
        if jog_hint is not None:
            return f"{base}  |  {jog_hint}  |  {soft}"
        return f"{base}  |  {soft}"

    def _refresh_soft_zero_label(self) -> None:
        self._soft_zero_label.set(self._soft_zero_summary())
        try:
            jog = "JOG=工件系" if self.wobj_jog_ready else ("JOG=兜底MoveJ" if self.connected else None)
            if jog is not None:
                self.hdr_text.set(self._hdr_text(jog))
            else:
                self.hdr_text.set(self._hdr_text())
        except Exception:
            pass

    # ---------- UI ----------
    def _build(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)

        self.var_allow = tk.BooleanVar(value=False)
        self.hdr_text = tk.StringVar(value=self._hdr_text())
        ttk.Checkbutton(top, text="确认真机（急停在手边）", variable=self.var_allow).pack(anchor=tk.W)
        ttk.Label(top, textvariable=self.hdr_text).pack(anchor=tk.W, pady=(4, 0))

        row = ttk.Frame(top)
        row.pack(fill=tk.X, pady=6)
        for label, cmd in [
            ("① 测通讯", self.on_test_comm),
            ("连接 SDK", self.on_connect),
            ("断开", self.on_disconnect),
            ("② 上使能", lambda: self.on_enable(True)),
            ("下使能", lambda: self.on_enable(False)),
            ("一键就绪", self.on_motion_ready),
            ("复位错误", self.on_reset_error),
            ("拖动模式 ON", lambda: self.on_drag(True)),
            ("拖动 OFF", lambda: self.on_drag(False)),
        ]:
            b = ttk.Button(row, text=label, command=cmd)
            b.pack(side=tk.LEFT, padx=2)
            self._action_buttons.append(b)

        tk.Button(
            row,
            text="急停 STOP",
            bg="#c0392b",
            fg="white",
            activebackground="#922b21",
            command=self.on_estop,
        ).pack(side=tk.RIGHT, padx=4)

        row2 = ttk.Frame(top)
        row2.pack(fill=tk.X, pady=(0, 4))
        b_rec = ttk.Button(row2, text="记录软零位", command=self.on_record_soft_zero)
        b_go = ttk.Button(row2, text="归软零", command=self.on_goto_soft_zero)
        b_rec.pack(side=tk.LEFT, padx=2)
        b_go.pack(side=tk.LEFT, padx=2)
        self._action_buttons.extend([b_rec, b_go])
        ttk.Label(row2, textvariable=self._soft_zero_label, foreground="#555").pack(side=tk.LEFT, padx=8)

        self.status = tk.StringVar(value="未连接")
        ttk.Label(top, textvariable=self.status, foreground="#1a5276").pack(anchor=tk.W)

        bottom = ttk.Frame(self, padding=(8, 0, 8, 8))
        bottom.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Button(bottom, text="退出", width=10, command=self._on_close).pack(side=tk.RIGHT)

        nb = ttk.Notebook(self)
        nb.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.tab_joint = ttk.Frame(nb, padding=8)
        self.tab_cart = ttk.Frame(nb, padding=8)
        nb.add(self.tab_joint, text="单关节")
        nb.add(self.tab_cart, text="末端")
        self._build_joint_tab()
        self._build_cart_tab()
        self._refresh_soft_zero_label()

    def _build_joint_tab(self) -> None:
        sp = ttk.Frame(self.tab_joint)
        sp.pack(fill=tk.X, pady=4)
        ttk.Label(sp, text="速度档 vel%").pack(side=tk.LEFT)
        self.var_jog_vel.set(self.jog_vel)
        self.lbl_jog_vel = ttk.Label(sp, text=f"{self.jog_vel:.0f}%", width=5)
        self.lbl_jog_vel.pack(side=tk.LEFT, padx=(4, 2))
        self.scale_jog_vel = ttk.Scale(
            sp,
            from_=1,
            to=max(1.0, self.jog_vel_max),
            variable=self.var_jog_vel,
            orient=tk.HORIZONTAL,
            command=self._on_vel_scale,
        )
        self.scale_jog_vel.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)

        gears = ttk.Frame(self.tab_joint)
        gears.pack(fill=tk.X, pady=2)
        ttk.Label(gears, text="一键档位:").pack(side=tk.LEFT)
        for label, pct in (("10%", 10), ("20%", 20), ("40%", 40), ("60%", 60), ("100%", 100)):
            ttk.Button(gears, text=label, width=9, command=lambda p=pct: self.set_speed_gear(p)).pack(
                side=tk.LEFT, padx=2
            )

        self._slider_scales = []
        self._slider_jog_dir = [0] * 6
        self._actual_labels = []
        self._joint_grabbed = [False] * 6
        self._jog_stop_flag = False

        for i in range(6):
            lo, hi = self._soft_lo_hi(i)
            hlo, hhi = self.joint_hard_limits[i]
            fr = ttk.LabelFrame(
                self.tab_joint,
                text=f"J{i+1}  软[{lo:.0f},{hi:.0f}]°  硬[{hlo:.0f},{hhi:.0f}]°",
                padding=4,
            )
            fr.pack(fill=tk.X, pady=3)

            act = ttk.Label(fr, text="实测 — °", width=14, foreground="#555")
            act.pack(side=tk.LEFT)
            self._actual_labels.append(act)

            var = tk.DoubleVar(value=0.0)
            self._joint_vars.append(var)
            tgt = ttk.Label(fr, text="目标 — °", width=12)
            tgt.pack(side=tk.LEFT)
            self._joint_labels.append(tgt)

            scale = ttk.Scale(fr, from_=hlo, to=hhi, variable=var, orient=tk.HORIZONTAL)
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
            self._slider_scales.append(scale)
            scale.configure(command=lambda v, a=i: self._on_joint_slider_target(a, v))
            scale.bind("<ButtonPress-1>", lambda e, a=i: self._joint_grab_set(a, True))
            scale.bind("<ButtonRelease-1>", lambda e, a=i: self._on_joint_slider_release(a))

            bf = ttk.Frame(fr)
            bf.pack(side=tk.RIGHT)
            ttk.Button(bf, text="−2°", width=4, command=lambda a=i: self.on_joint_step(a, -2.0)).pack(
                side=tk.LEFT, padx=1
            )
            ttk.Button(bf, text="+2°", width=4, command=lambda a=i: self.on_joint_step(a, 2.0)).pack(
                side=tk.LEFT, padx=1
            )
            bm = ttk.Button(bf, text="−JOG", width=5)
            bp = ttk.Button(bf, text="+JOG", width=5)
            bm.pack(side=tk.LEFT, padx=1)
            bp.pack(side=tk.LEFT, padx=1)
            bm.bind("<ButtonPress-1>", lambda e, a=i + 1: self._jog_joint_hold(a, 0, True))
            bm.bind("<ButtonRelease-1>", lambda e, a=i + 1: self._jog_joint_hold(a, 0, False))
            bp.bind("<ButtonPress-1>", lambda e, a=i + 1: self._jog_joint_hold(a, 1, True))
            bp.bind("<ButtonRelease-1>", lambda e, a=i + 1: self._jog_joint_hold(a, 1, False))

        ttk.Button(self.tab_joint, text="刷新关节角", command=self.refresh_joints).pack(anchor=tk.W, pady=6)

    def _build_cart_tab(self) -> None:
        self.tcp_text.set("TCP: —")
        ttk.Label(self.tab_cart, textvariable=self.tcp_text).pack(anchor=tk.W, pady=4)

        pad = ttk.LabelFrame(self.tab_cart, text="平移 XYZ", padding=8)
        pad.pack(fill=tk.X, pady=4)

        def jog_btn(parent, text, nb, direction, row, col):
            b = ttk.Button(parent, text=text, width=8)
            b.grid(row=row, column=col, padx=2, pady=2)
            b.bind("<ButtonPress-1>", lambda e, n=nb, d=direction: self._jog_cart_hold(n, d, True))
            b.bind("<ButtonRelease-1>", lambda e: self._jog_cart_hold(0, 0, False))
            return b

        g = ttk.Frame(pad)
        g.pack()
        jog_btn(g, "Y+", 2, 1, 0, 1)
        jog_btn(g, "X−", 1, 0, 1, 0)
        jog_btn(g, "Z+", 3, 1, 1, 1)
        jog_btn(g, "X+", 1, 1, 1, 2)
        jog_btn(g, "Y−", 2, 0, 2, 1)
        jog_btn(g, "Z−", 3, 0, 2, 2)

        rot = ttk.LabelFrame(self.tab_cart, text="姿态 Rx/Ry/Rz", padding=6)
        rot.pack(fill=tk.X, pady=8)
        for name, nb in (("Rx", 4), ("Ry", 5), ("Rz", 6)):
            f = ttk.Frame(rot)
            f.pack(side=tk.LEFT, padx=8)
            ttk.Label(f, text=name).pack()
            bm = ttk.Button(f, text="−", width=3)
            bp = ttk.Button(f, text="+", width=3)
            bm.pack(side=tk.LEFT)
            bp.pack(side=tk.LEFT)
            bm.bind("<ButtonPress-1>", lambda e, n=nb: self._jog_cart_hold(n, 0, True))
            bm.bind("<ButtonRelease-1>", lambda e: self._jog_cart_hold(0, 0, False))
            bp.bind("<ButtonPress-1>", lambda e, n=nb: self._jog_cart_hold(n, 1, True))
            bp.bind("<ButtonRelease-1>", lambda e: self._jog_cart_hold(0, 0, False))

        ttk.Button(self.tab_cart, text="刷新 TCP", command=self.refresh_tcp).pack(anchor=tk.W)

    # ---------- gates / helpers ----------
    def _set_status(self, msg: str) -> None:
        self.status.set(msg)

    def _require_allow(self) -> bool:
        if not self.var_allow.get():
            messagebox.showwarning("安全门控", "请先勾选「确认真机」后再下发运动/使能。")
            return False
        return True

    def _soft_lo_hi(self, idx: int) -> tuple[float, float]:
        return self.joint_soft_limits[idx]

    def _clamp_soft(self, idx: int, ang: float) -> float:
        lo, hi = self._soft_lo_hi(idx)
        return max(lo, min(hi, float(ang)))

    def _soft_room(self, idx: int, angle: float, direction: int) -> float:
        lo, hi = self._soft_lo_hi(idx)
        a = float(angle)
        if direction == 1:
            return max(0.0, hi - a)
        return max(0.0, a - lo)

    def _jog_vel_pct(self) -> float:
        return max(1.0, min(float(self.jog_vel_max), float(self.var_jog_vel.get())))

    def _jog_acc_pct(self) -> float:
        return max(1.0, min(100.0, float(self.jog_acc)))

    def _on_vel_scale(self, _value: str = "") -> None:
        self.lbl_jog_vel.configure(text=f"{self._jog_vel_pct():.0f}%")

    def set_speed_gear(self, pct: float) -> None:
        pct = max(1.0, min(float(self.jog_vel_max), float(pct)))
        self.var_jog_vel.set(pct)
        self.lbl_jog_vel.configure(text=f"{pct:.0f}%")
        self.speed = pct
        self._set_status(f"vel={pct:.0f}%")
        if self.connected and self.robot is not None:
            def work():
                e = call_err(self.robot.SetSpeed(pct))
                self.after(0, lambda e=e, pct=pct: self._set_status(f"vel={pct:.0f}% SetSpeed={e}"))
            threading.Thread(target=work, daemon=True).start()

    def _require_robot(self, need_enable: bool = False) -> bool:
        if not self.connected or self.robot is None:
            messagebox.showwarning("未连接", "请先「测通讯」并「连接 SDK」。")
            return False
        if need_enable and not self.enabled:
            messagebox.showwarning("未使能", "请先上使能。")
            return False
        return True

    def _set_busy(self, busy: bool) -> None:
        self._busy = busy
        state = tk.DISABLED if busy else tk.NORMAL
        for w in self._action_buttons:
            try:
                w.configure(state=state)
            except tk.TclError:
                pass

    def _run_bg(self, fn, done_msg: str | None = None, busy_hint: str = "处理中…") -> None:
        if self._busy:
            self._set_status("忙碌中，请稍候再点（勿连点）")
            return
        self._set_busy(True)
        self._set_status(busy_hint)

        def work():
            try:
                fn()
                if done_msg is not None:
                    msg = done_msg
                    self.after(0, lambda m=msg: self._set_status(m))
            except Exception as e:
                self.after(0, lambda err=str(e): messagebox.showerror("异常", err))
                self.after(0, lambda err=str(e): self._set_status(f"错误: {err}"))
            finally:
                self.after(0, lambda: self._set_busy(False))

        threading.Thread(target=work, daemon=True).start()

    def _ensure_motion_ready(self) -> None:
        """运动前：退出拖动 → 停运动 → 清错 → 手动模式 → 重新上使能。

        拖动退出后本机常出现 MoveJ=14；须重新使能，归零优先走 JOG。
        """
        import time as _time

        robot = self.robot
        if robot is None:
            return
        try:
            call_err(robot.DragTeachSwitch(0))
            self.drag_on = False
        except Exception:
            pass
        try:
            st = robot.IsInDragTeach()
            if isinstance(st, (list, tuple)) and len(st) >= 2 and int(st[0]) == 0 and int(st[1]) != 0:
                call_err(robot.DragTeachSwitch(0))
                self.drag_on = False
        except Exception:
            pass
        try:
            robot.ImmStopJOG()
        except Exception:
            pass
        try:
            robot.StopMotion()
        except Exception:
            pass
        # 拖动后残留 [main,sub] 非 0 时必须清；即使读数为 0 也清一次更稳
        try:
            call_err(robot.ResetAllError())
        except Exception:
            pass
        _time.sleep(0.25)
        try:
            call_err(robot.Mode(1))
        except Exception:
            pass
        try:
            call_err(robot.RobotEnable(1))
            self.enabled = True
        except Exception:
            pass
        _time.sleep(0.2)

    def _stop_jog(self, joint: bool) -> None:
        if self.robot is None:
            return

        def work():
            stop_jog(self.robot, joint=joint)

        threading.Thread(target=work, daemon=True).start()

    # ---------- connect / safety ----------
    def on_test_comm(self) -> None:
        def work():
            lines = []
            ok_ping = ping_ok(self.ip)
            lines.append(f"ping {self.ip}: {'OK' if ok_ping else 'FAIL'}")
            port_map = probe_ports(self.ip)
            labels = {
                80: "WebApp",
                20003: "XML-RPC",
                20004: "状态(旧)",
                20005: "CNDE",
                20006: "CNDE-UDP",
                8080: "TCP cmd",
                8083: "status",
            }
            for p, name in labels.items():
                lines.append(f"tcp {p} ({name}): {'OPEN' if port_map.get(p) else 'closed'}")
            msg = " | ".join(lines)
            self.after(0, lambda: self._set_status(msg))
            xml_ok = port_map.get(20003, False)
            cnde_ok = port_map.get(20005, False)
            if ok_ping and xml_ok:
                hint = "可点「连接 SDK」。"
                if not cnde_ok:
                    hint += "\n20005 未开：默认直连 20003（本机已验证可 JOG/Move）。"
                self.after(0, lambda: messagebox.showinfo("测通讯", hint + "\n" + "\n".join(lines)))
            else:
                self.after(
                    0,
                    lambda: messagebox.showwarning(
                        "测通讯",
                        "未完全打通。请检查网线/fr5-eth(192.168.58.10)与控制箱。\n" + "\n".join(lines),
                    ),
                )

        self._run_bg(work, busy_hint="测通讯中…")

    def on_connect(self) -> None:
        if self.connected and self.robot is not None:
            messagebox.showinfo("已连接", "SDK 已连接，无需重复点。若要重连请先「断开」。")
            return

        def work():
            res = connect_robot(
                self.ip, allow_degrade=self.allow_degrade, prefer_xmlrpc_only=self.prefer_xmlrpc_only
            )
            robot = res.robot
            err, codes = unwrap_pair(robot.GetRobotErrorCode())
            self.robot = robot
            self.connected = True
            self.degraded = res.degraded

            mode_err = call_err(robot.Mode(1))
            spd_err = call_err(robot.SetSpeed(self.speed))
            wobj_info = activate_world_wobj(
                robot, wobj_id=self.world_wobj_id, yaw_cw_deg=self.world_yaw_cw
            )
            self.wobj_jog_ready = bool(self.prefer_wobj_jog and wobj_info.get("written"))
            wobj_hint = f" | wobj {wobj_info.get('detail')}"
            if wobj_info.get("written") and not wobj_info.get("cur_applied"):
                wobj_hint += " | 列表已写但当前系可能未切换(GetCur=0)，若JOG方向不对再到WebApp应用该工件系"
            en_hint = ""
            if mode_err == 14:
                en_hint = " | 警告: Mode(手动)返回14(接口失败)，可能处于示教器配对中，请先完成配对/使能末端绿灯常亮"

            link_mode = "20003" if res.degraded else "全通"
            status = (
                f"已连接({link_mode}) {self.ip} | {res.detail} | err={err} codes={codes}"
                f" | Mode={mode_err} SetSpeed={spd_err}{en_hint}{wobj_hint}"
            )
            self.after(0, lambda: self._set_status(status))
            self.after(0, self._refresh_soft_zero_label)

            if res.degraded:
                self.after(
                    0,
                    lambda: messagebox.showinfo(
                        "已连接（XML-RPC 20003）",
                        "默认直连控制箱 20003，不依赖 CNDE 20005。\n"
                        "请看状态栏「已连接」再点上使能/一键就绪。\n"
                        "急停请保持在手边。",
                    ),
                )
            if mode_err == 14:
                self.after(
                    0,
                    lambda: messagebox.showwarning(
                        "机器人可能处于配对中",
                        "Mode(手动) 返回 14（接口执行失败）。\n"
                        "三色灯交替闪多为示教器/按钮盒配对或未就绪：\n"
                        "1) 在示教器完成配对或退出配对\n"
                        "2) 急停松开、控制箱重新上电\n"
                        "3) 末端 LED 绿色常亮后再上使能/JOG\n"
                        "config 里 mode=sim 不影响连接，只影响项目脚本标记。",
                    ),
                )
            self.after(0, self.refresh_joints)
            self.after(0, self.refresh_tcp)

        self._run_bg(work, busy_hint="连接 SDK 中…（终端 CNDE 失败可忽略）")

    def on_disconnect(self) -> None:
        def work():
            if self.robot is not None:
                try:
                    if self.drag_on:
                        self.robot.DragTeachSwitch(0)
                    if self.enabled:
                        self.robot.RobotEnable(0)
                    self.robot.CloseRPC()
                except Exception:
                    pass
            self.robot = None
            self.connected = False
            self.degraded = False
            self.enabled = False
            self.drag_on = False
            self.wobj_jog_ready = False
            self.after(0, lambda: self._set_status("已断开"))
            self.after(0, self._refresh_soft_zero_label)

        self._run_bg(work, busy_hint="断开中…")

    def on_enable(self, on: bool) -> None:
        if on and not self._require_allow():
            return
        if not self._require_robot(False):
            return

        def work():
            err = call_err(self.robot.RobotEnable(1 if on else 0))
            self.enabled = on and err == 0
            msg = f"{'上' if on else '下'}使能 err={err} enabled={self.enabled}"
            if on and err == 101:
                msg += " | 未使能(101)：检查配对/急停/示教器"
            elif on and err == 14:
                msg += " | 接口失败(14)：可能仍在配对中"
            self.after(0, lambda: self._set_status(msg))
            if on and err != 0:
                self.after(
                    0,
                    lambda e=err: messagebox.showwarning(
                        "使能失败",
                        f"RobotEnable 返回 {e}。\n若灯在闪：先完成示教器配对，等末端绿灯常亮。",
                    ),
                )
            if on:
                self.after(0, self.refresh_joints)

        self._run_bg(work)

    def on_reset_error(self) -> None:
        if not self._require_robot(False):
            return

        def work():
            r = call_err(self.robot.ResetAllError())
            try:
                self.robot.ImmStopJOG()
            except Exception:
                pass
            self.after(0, lambda: self._set_status(f"复位错误完成 ResetAllError={r}"))
            self.after(
                0,
                lambda: messagebox.showinfo(
                    "复位", f"ResetAllError={r}\n请到 WebApp 确认错误已消失后再点动。"
                ),
            )

        self._run_bg(work, busy_hint="复位错误中…")

    def on_motion_ready(self) -> None:
        if not self._require_allow():
            return
        if not self._require_robot(False):
            return

        def work():
            vel = self._jog_vel_pct()
            prep = prepare_motion(self.robot, speed_pct=vel, enable=True)
            self.enabled = prep.get("enable", -1) == 0
            msg = f"就绪 {prep} enabled={self.enabled}"
            if not self.enabled:
                msg += " | 使能失败：检查配对/急停/WebApp"
            self.after(0, lambda: self._set_status(msg))
            if not self.enabled:
                self.after(
                    0,
                    lambda: messagebox.showwarning(
                        "就绪失败",
                        f"RobotEnable 未成功：{prep}\n请示教器配对完成、末端绿灯常亮后再试。",
                    ),
                )
            else:
                self.after(0, self.refresh_joints)

        self._run_bg(work, busy_hint="就绪中（Mode/清错/使能）…")

    def on_drag(self, on: bool) -> None:
        if on and not self._require_allow():
            return
        if not self._require_robot(need_enable=True):
            return

        def work():
            try:
                self.robot.Mode(1)
            except Exception:
                pass
            err = self.robot.DragTeachSwitch(1 if on else 0)
            ok = err == 0 if isinstance(err, int) else (isinstance(err, (list, tuple)) and err[0] == 0)
            self.drag_on = on and ok
            self.after(0, lambda: self._set_status(f"拖动模式 {'ON' if self.drag_on else 'OFF'}，返回={err}"))

        self._run_bg(work)

    def on_estop(self) -> None:
        def work():
            if self.robot is None:
                return
            try:
                self.robot.ImmStopJOG()
            except Exception:
                pass
            try:
                self.robot.StopMotion()
            except Exception:
                pass
            try:
                if self.drag_on:
                    self.robot.DragTeachSwitch(0)
                    self.drag_on = False
            except Exception:
                pass
            try:
                self.robot.ResetAllError()
            except Exception:
                pass
            self.after(0, lambda: self._set_busy(False))
            self.after(0, lambda: self._set_status("急停: ImmStopJOG + StopMotion + ResetAllError"))

        threading.Thread(target=work, daemon=True).start()

    # ---------- soft zero ----------
    def on_record_soft_zero(self) -> None:
        if not self._require_robot(False):
            return

        def work():
            try:
                err, pos = unwrap_pair(self.robot.GetActualJointPosDegree(1))
                if err != 0 or pos is None:
                    raise RuntimeError(f"读关节失败 err={err}")
                joints = [float(x) for x in list(pos)[:6]]
                bad = joints_violate_soft(joints, self.joint_soft_limits, eps=0.0)
                tcp = None
                try:
                    e2, pose = unwrap_pair(self.robot.GetActualTCPPose(1))
                    if e2 == 0 and pose is not None:
                        tcp = [float(x) for x in list(pose)[:6]]
                except Exception:
                    pass
                save_soft_zero(self.soft_zero_path, joints, tcp_mm_deg=tcp, note="ui")
                self.soft_zero = load_soft_zero(self.soft_zero_path)
                self.after(0, self._refresh_soft_zero_label)
                tip = "已记录软零位 → " + ", ".join(f"{v:.2f}" for v in joints)
                if bad:
                    tip += "\n警告：当前已越软件软限，归零时请确认路径安全。"
                self.after(0, lambda: self._set_status(tip.replace("\n", " | ")))
                self.after(
                    0,
                    lambda: messagebox.showinfo("记录软零位", tip + f"\n保存: {self.soft_zero_path}"),
                )
            except Exception as ex:
                self.after(0, lambda err=str(ex): messagebox.showerror("记录软零位", err))

        self._run_bg(work, busy_hint="记录软零位…")

    def on_goto_soft_zero(self) -> None:
        if not self._require_allow() or not self._require_robot(True):
            return
        if not self.soft_zero:
            messagebox.showwarning("归软零", "尚未记录软零位，请先点「记录软零位」。")
            return
        if self.drag_on:
            messagebox.showwarning("拖动中", "请先「拖动 OFF」再归软零。")
            return
        joints = list(self.soft_zero["joints_deg"])
        preview = ", ".join(f"J{i+1}={joints[i]:.1f}" for i in range(6))
        if not messagebox.askyesno(
            "归软零",
            f"将逐轴 JOG 回到软零位（本机 MoveJ 拖动后常返回 14）：\n{preview}\n\n"
            "请确认空间无干涉；可用急停中止。",
        ):
            return

        self._jog_stop_flag = False

        def work():
            try:
                self._ensure_motion_ready()
                vel = min(15.0, self._jog_vel_pct())
                # 先试 MoveJ；14 则自动改 JOG 逐轴
                mv = call_err(
                    self.robot.MoveJ(
                        joint_pos=joints,
                        tool=self.tool,
                        user=self.user,
                        vel=min(30.0, vel),
                        blendT=-1.0,
                    )
                )
                if mv == 0:
                    self.after(0, lambda: self._set_status(f"归软零 MoveJ 成功 vel={vel:.0f}%"))
                else:
                    self.after(
                        0,
                        lambda e=mv: self._set_status(f"MoveJ={e}，改走 JOG 归软零…"),
                    )
                    res = goto_joints_by_jog(
                        self.robot,
                        joints,
                        vel_pct=vel,
                        acc_pct=min(60.0, self._jog_acc_pct()),
                        tol_deg=0.5,
                        stop_flag=lambda: self._jog_stop_flag,
                    )
                    if res.get("ok"):
                        self.after(
                            0,
                            lambda d=res.get("detail", ""): self._set_status(f"归软零 JOG 完成 | {d}"),
                        )
                    else:
                        self.after(
                            0,
                            lambda r=res: messagebox.showwarning(
                                "归软零失败",
                                f"MoveJ 与 JOG 均未完成。\n"
                                f"detail={r.get('detail')} last_err={r.get('last_err')}\n"
                                "请看 WebApp 故障灯，急停松开后：拖动 OFF → 复位 → 一键就绪。",
                            ),
                        )
                self.after(200, self.refresh_joints)
                self.after(200, self.refresh_tcp)
            except Exception as ex:
                self.after(0, lambda err=str(ex): messagebox.showerror("归软零", err))

        self._run_bg(work, busy_hint="归软零中…")

    # ---------- status refresh ----------
    def refresh_joints(self) -> None:
        if not self.connected or self.robot is None:
            return

        def work():
            try:
                err, joints = unwrap_pair(self.robot.GetActualJointPosDegree(1))
                if err != 0 or joints is None:
                    self.after(0, lambda err=err: self._set_status(f"读关节失败 err={err}"))
                    return
                joints = [float(x) for x in list(joints)[:6]]
                self._actual_cache = joints

                def ui():
                    self._suppress_joint = True
                    for i, j in enumerate(joints):
                        if i < len(self._actual_labels):
                            mark = ""
                            lo, hi = self._soft_lo_hi(i)
                            if j < lo or j > hi:
                                mark = " !软"
                            self._actual_labels[i].configure(text=f"实测 {j:.2f}°{mark}")
                        if not self._joint_grabbed[i]:
                            self._joint_vars[i].set(j)
                            self._joint_labels[i].configure(text=f"目标 {j:.2f} °")
                    self._suppress_joint = False
                    self._set_status("关节角已刷新")

                self.after(0, ui)
            except Exception as e:
                self.after(0, lambda err=str(e): self._set_status(f"读关节异常: {err}"))

        threading.Thread(target=work, daemon=True).start()

    def refresh_tcp(self) -> None:
        if not self.connected or self.robot is None:
            return

        def work():
            try:
                err, pose = unwrap_pair(self.robot.GetActualTCPPose(1))
                if err != 0 or pose is None:
                    self.after(0, lambda err=err: self._set_status(f"读 TCP 失败 err={err}"))
                    return
                pose = [float(v) for v in list(pose)[:6]]
                w = base_pose_to_world(pose, self.world_yaw_cw)
                txt = (
                    "TCP世界[mm,deg]: "
                    + ", ".join(f"{v:.2f}" for v in w)
                    + "  |  基座: "
                    + ", ".join(f"{v:.2f}" for v in pose)
                )
                self.after(0, lambda txt=txt: self.tcp_text.set(txt))
            except Exception as e:
                self.after(0, lambda err=str(e): self._set_status(f"读 TCP 异常: {err}"))

        threading.Thread(target=work, daemon=True).start()

    def _poll_status(self) -> None:
        if self.connected and self.robot is not None and not any(self._joint_grabbed) and not self._busy:
            self._poll_actual_joints_light()
        self.after(500, self._poll_status)

    def _poll_actual_joints_light(self) -> None:
        if not self.connected or self.robot is None:
            return

        def work():
            try:
                err, joints = unwrap_pair(self.robot.GetActualJointPosDegree(1))
                if err != 0 or joints is None:
                    return
                joints = [float(x) for x in list(joints)[:6]]
                self._actual_cache = joints

                def ui():
                    self._actual_cache = joints
                    for i, j in enumerate(joints):
                        if i < len(self._actual_labels):
                            self._actual_labels[i].configure(text=f"实测 {j:.2f} °")
                        if not self._joint_grabbed[i] and not self._suppress_joint and not self._busy:
                            self._suppress_joint = True
                            j2 = self._clamp_soft(i, j)
                            self._joint_vars[i].set(j2)
                            self._joint_labels[i].configure(text=f"目标 {j2:.2f} °")
                            self._suppress_joint = False

                self.after(0, ui)
            except Exception:
                pass

        threading.Thread(target=work, daemon=True).start()

    # ---------- joint slider JOG ----------
    def _joint_grab_set(self, idx: int, grabbed: bool) -> None:
        self._joint_grabbed[idx] = grabbed
        if grabbed:
            try:
                self._actual_cache[idx] = float(self._joint_vars[idx].get())
            except Exception:
                pass
            self._ensure_slider_tick()

    def _ensure_slider_tick(self) -> None:
        if self._slider_tick_on:
            return
        self._slider_tick_on = True
        self.after(80, self._slider_tick)

    def _slider_tick(self) -> None:
        if not any(self._joint_grabbed):
            self._slider_tick_on = False
            return
        if self.connected and self.robot is not None and self.enabled and self.var_allow.get() and not self.drag_on:
            for i, grabbed in enumerate(self._joint_grabbed):
                if grabbed:
                    self._slider_live_jog(i, from_tick=True)
        self.after(100, self._slider_tick)

    def _clamp_slider_lead(self, idx: int, raw: float) -> float:
        actual = float(self._actual_cache[idx])
        lead = max(1.0, float(self.slider_lead))
        clamped = max(actual - lead, min(actual + lead, raw))
        return self._clamp_soft(idx, clamped)

    def _on_joint_slider_target(self, idx: int, value: str) -> None:
        if self._suppress_joint:
            return
        try:
            raw = float(value)
        except ValueError:
            return
        if not self._joint_grabbed[idx]:
            self._joint_labels[idx].configure(text=f"目标 {raw:.2f} °")
            return

        clamped = self._clamp_slider_lead(idx, raw)
        if abs(clamped - raw) > 0.05:
            self._suppress_joint = True
            self._joint_vars[idx].set(clamped)
            self._suppress_joint = False
        self._joint_labels[idx].configure(text=f"目标 {clamped:.2f} °")

        if not self.var_allow.get():
            self._set_status("拖条：请先勾选「确认真机」")
            return
        if not self.connected or self.robot is None:
            self._set_status("拖条：请先连接 SDK")
            return
        if not self.enabled:
            self._set_status("拖条：请先「一键就绪」或上使能")
            return
        if self.drag_on:
            return
        self._slider_live_jog(idx, from_tick=False)

    def _slider_live_jog(self, idx: int, from_tick: bool = False) -> None:
        if self.robot is None or not self._joint_grabbed[idx]:
            return

        def work():
            try:
                err, cur = unwrap_pair(self.robot.GetActualJointPosDegree(1))
                if err != 0 or cur is None:
                    return
                joints = [float(x) for x in list(cur)[:6]]
                self._actual_cache = joints
                actual = joints[idx]
                self.after(0, lambda: self._actual_labels[idx].configure(text=f"实测 {actual:.2f} °"))

                try:
                    raw = float(self._joint_vars[idx].get())
                except Exception:
                    return
                clamped = self._clamp_slider_lead(idx, raw)
                if abs(clamped - raw) > 0.05:
                    def snap(c=clamped):
                        self._suppress_joint = True
                        self._joint_vars[idx].set(c)
                        self._joint_labels[idx].configure(text=f"目标 {c:.2f} °")
                        self._suppress_joint = False

                    self.after(0, snap)

                dead = 0.35
                diff = clamped - actual
                if abs(diff) <= dead:
                    if self._slider_jog_dir[idx] != 0:
                        self._slider_jog_dir[idx] = 0
                        stop_jog(self.robot, joint=True)
                    return

                direction = 1 if diff > 0 else 0
                signed = 1 if diff > 0 else -1
                room = self._soft_room(idx, actual, direction)
                if room <= 0.15:
                    if self._slider_jog_dir[idx] != 0:
                        self._slider_jog_dir[idx] = 0
                        stop_jog(self.robot, joint=True)
                    self.after(0, lambda: self._set_status(f"J{idx+1} 已到软限位，停止 JOG"))
                    return
                if not from_tick and self._slider_jog_dir[idx] == signed:
                    return
                self._slider_jog_dir[idx] = signed
                vel = self._jog_vel_pct()
                max_dis = max(abs(diff), float(self.slider_lead))
                max_dis = min(max_dis, max(1.0, float(self.jog_j_max)), room)
                e = start_joint_jog(
                    self.robot,
                    idx + 1,
                    direction,
                    max_dis_deg=max_dis,
                    vel_pct=vel,
                    acc_pct=self._jog_acc_pct(),
                )
                if not from_tick or e != 0:
                    self.after(
                        0,
                        lambda: self._set_status(
                            f"拖条 JOG J{idx+1} tgt={clamped:.1f} act={actual:.1f} "
                            f"lead±{self.slider_lead:.0f} err={e}"
                        ),
                    )
            except Exception as ex:
                self.after(0, lambda err=str(ex): self._set_status(f"拖条 JOG 异常: {err}"))

        threading.Thread(target=work, daemon=True).start()

    def _on_joint_slider_release(self, idx: int) -> None:
        self._joint_grabbed[idx] = False
        self._slider_jog_dir[idx] = 0
        self._stop_jog(joint=True)
        self.after(120, self.refresh_joints)

    def on_joint_step(self, idx: int, delta_deg: float) -> None:
        if not self._require_allow() or not self._require_robot(True):
            return
        if self.drag_on:
            messagebox.showwarning("拖动中", "请先关闭拖动模式。")
            return

        def work():
            try:
                self._ensure_motion_ready()
                err, pos = unwrap_pair(self.robot.GetActualJointPosDegree(1))
                if err != 0 or pos is None:
                    raise RuntimeError(f"读关节失败 {err}")
                target = list(float(x) for x in pos[:6])
                target[idx] = self._clamp_soft(idx, target[idx] + float(delta_deg))
                vel = self._jog_vel_pct()
                mv = call_err(
                    self.robot.MoveJ(
                        joint_pos=target, tool=self.tool, user=self.user, vel=vel, blendT=-1.0
                    )
                )
                self.after(
                    0,
                    lambda: self._set_status(
                        f"MoveJ J{idx+1} Δ={delta_deg:+.1f}° -> {target[idx]:.2f}° err={mv}"
                    ),
                )
                if mv != 0:
                    self.after(0, lambda e=mv: messagebox.showwarning("MoveJ 失败", f"错误码 {e}，请复位后重试"))
                self.after(200, self.refresh_joints)
            except Exception as ex:
                self.after(0, lambda err=str(ex): messagebox.showerror("MoveJ", err))

        self._run_bg(work, busy_hint=f"J{idx+1} MoveJ…")

    def _jog_joint_hold(self, nb: int, direction: int, press: bool) -> None:
        if not press:
            self._jog_stop_flag = True
            self._stop_jog(joint=True)
            return
        if not self._require_allow() or not self._require_robot(True):
            return
        if self.drag_on:
            return
        self._jog_stop_flag = False
        vel = self._jog_vel_pct()

        def work():
            try:
                if self._jog_stop_flag:
                    return
                self._ensure_motion_ready()
                err, cur = unwrap_pair(self.robot.GetActualJointPosDegree(1))
                if err != 0 or cur is None:
                    self.after(0, lambda err=err: self._set_status(f"JOG 读角失败 err={err}"))
                    return
                idx = nb - 1
                actual = float(cur[idx])
                lo, hi = self._soft_lo_hi(idx)
                room = self._soft_room(idx, actual, direction)
                if room <= 0.2:
                    self.after(
                        0,
                        lambda: self._set_status(
                            f"J{nb} 软限位拦截 act={actual:.1f}° 软[{lo:.0f},{hi:.0f}]"
                        ),
                    )
                    return
                max_dis = min(90.0, room)
                e = start_joint_jog(
                    self.robot, nb, direction, max_dis_deg=max_dis, vel_pct=vel, acc_pct=self._jog_acc_pct()
                )
                if self._jog_stop_flag:
                    stop_jog(self.robot, joint=True)
                    return
                self.after(
                    0,
                    lambda e=e, nb=nb, direction=direction, vel=vel, room=room: self._set_status(
                        f"JOG J{nb} dir={direction} vel={vel:.0f}% room={room:.1f}° err={e}"
                    ),
                )
                if e != 0:
                    self.after(
                        0,
                        lambda e=e: messagebox.showwarning(
                            "JOG 失败",
                            f"StartJOG={e}。\n14=接口失败：请先「拖动 OFF」→「复位错误」→「一键就绪」再拖条。\n"
                            "若刚用 WebApp 拖动过，必须退出拖动模式。",
                        ),
                    )
            except Exception as ex:
                self.after(0, lambda err=str(ex): self._set_status(f"JOG 异常: {err}"))

        threading.Thread(target=work, daemon=True).start()

    # ---------- cart / world JOG ----------
    def _monitor_soft_limits_while_cart(self) -> None:
        """末端 JOG 期间：仅当关节已越软限才停；往内侧可继续。"""

        def work():
            while not self._jog_stop_flag and self.robot is not None:
                try:
                    err, cur = unwrap_pair(self.robot.GetActualJointPosDegree(1))
                    if err == 0 and cur is not None:
                        bad = joints_violate_soft(cur, self.joint_soft_limits, eps=0.0)
                        if bad:
                            try:
                                self.robot.ImmStopJOG()
                            except Exception:
                                pass
                            try:
                                self.robot.StopMotion()
                            except Exception:
                                pass
                            try:
                                self.robot.StopJOG(9)
                            except Exception:
                                pass
                            self._jog_stop_flag = True
                            b = bad[0]
                            msg = (
                                f"末端软限停: J{b[0]+1}={b[1]:.1f}° 软[{b[2]:.0f},{b[3]:.0f}]"
                                "（往限位内侧即可继续）"
                            )
                            self.after(0, lambda m=msg: self._set_status(m))
                            return
                except Exception:
                    pass
                time.sleep(0.05)

        threading.Thread(target=work, daemon=True).start()

    def _jog_cart_hold(self, nb: int, direction: int, press: bool) -> None:
        """世界系点动：优先工件系 StartJOG(ref=8)；否则远点 MoveJ 兜底。"""
        if not press:
            self._jog_stop_flag = True
            if self.robot is not None:
                def stop_work():
                    try:
                        self.robot.StopMotion()
                    except Exception:
                        pass
                    try:
                        self.robot.ImmStopJOG()
                    except Exception:
                        pass
                    try:
                        self.robot.StopJOG(9)
                    except Exception:
                        pass

                threading.Thread(target=stop_work, daemon=True).start()
            self.after(120, self.refresh_tcp)
            return
        if not self._require_allow() or not self._require_robot(True):
            return
        if self.drag_on:
            return
        self._jog_stop_flag = False
        vel = self._jog_vel_pct()
        sign = 1.0 if direction == 1 else -1.0
        yaw = float(self.world_yaw_cw)

        def work():
            try:
                try:
                    self._ensure_motion_ready()
                    err0, cur0 = unwrap_pair(self.robot.GetActualJointPosDegree(1))
                    if err0 == 0 and cur0 is not None:
                        bad0 = joints_violate_soft(cur0, self.joint_soft_limits, eps=0.0)
                        if bad0:
                            b = bad0[0]
                            msg = (
                                f"末端拒绝：J{b[0]+1}={b[1]:.1f}° 已越软限[{b[2]:.0f},{b[3]:.0f}]；"
                                "请单关节往限位内侧离开后再末端点动"
                            )
                            self.after(0, lambda m=msg: self._set_status(m))
                            return
                except Exception:
                    pass

                if self.wobj_jog_ready:
                    max_dis = 40.0 if nb <= 3 else 20.0
                    e = call_err(
                        self.robot.StartJOG(8, nb, direction, max_dis, vel=vel, acc=self._jog_acc_pct())
                    )
                    axis = {1: "X", 2: "Y", 3: "Z", 4: "Rx", 5: "Ry", 6: "Rz"}.get(nb, str(nb))
                    if e == 0:
                        self._monitor_soft_limits_while_cart()
                        self.after(
                            0,
                            lambda a=axis, e=e: self._set_status(f"工件系 JOG {a} ref=8 err={e}"),
                        )
                        return
                    self.after(
                        0,
                        lambda a=axis, e=e: self._set_status(f"工件系 JOG {a} 失败 err={e}，改走远点兜底"),
                    )

                if nb == 3:
                    e = call_err(
                        self.robot.StartJOG(2, 3, direction, 40.0, vel=vel, acc=self._jog_acc_pct())
                    )
                    if e == 0:
                        self._monitor_soft_limits_while_cart()
                    self.after(0, lambda e=e: self._set_status(f"基座 Z JOG err={e}"))
                    return
                if nb == 6:
                    e = call_err(
                        self.robot.StartJOG(2, 6, direction, 25.0, vel=vel, acc=self._jog_acc_pct())
                    )
                    if e == 0:
                        self._monitor_soft_limits_while_cart()
                    self.after(0, lambda e=e: self._set_status(f"基座 Rz JOG err={e}"))
                    return

                err, pose = unwrap_pair(self.robot.GetActualTCPPose(1))
                if err != 0 or pose is None:
                    self.after(0, lambda err=err: self._set_status(f"读 TCP 失败 {err}"))
                    return
                pose = [float(x) for x in pose[:6]]
                errj, jp = unwrap_pair(self.robot.GetActualJointPosDegree(1))
                jp = [float(x) for x in jp[:6]] if errj == 0 and jp is not None else None

                if nb in (1, 2):
                    travel = 40.0
                    unit = world_axis_unit_in_base(nb, yaw)
                    desc = [
                        pose[0] + sign * travel * unit[0],
                        pose[1] + sign * travel * unit[1],
                        pose[2] + sign * travel * unit[2],
                        pose[3],
                        pose[4],
                        pose[5],
                    ]
                else:
                    ang = sign * 12.0
                    ax = world_axis_unit_in_base(1 if nb == 4 else 2, yaw)
                    desc = rotate_pose_about_axis(pose, ax, ang)

                if self._jog_stop_flag:
                    return
                e = cart_goto_async(
                    self.robot,
                    desc,
                    tool=self.tool,
                    user=0,
                    vel=vel,
                    joint_ref=jp,
                    soft_limits=self.joint_soft_limits,
                )
                axis = {1: "X", 2: "Y", 4: "Rx", 5: "Ry"}.get(nb, str(nb))
                if e == 1001:
                    self.after(
                        0,
                        lambda a=axis: self._set_status(f"兜底远点 {a} 拒绝：逆解关节越软限位"),
                    )
                else:
                    if e == 0:
                        self._monitor_soft_limits_while_cart()
                    self.after(0, lambda a=axis, e=e: self._set_status(f"兜底远点 {a} err={e}"))
            except Exception as ex:
                self.after(0, lambda err=str(ex): self._set_status(f"世界系点动异常: {err}"))

        threading.Thread(target=work, daemon=True).start()

    def _on_close(self) -> None:
        try:
            if self.robot is not None:
                try:
                    self.robot.ImmStopJOG()
                except Exception:
                    pass
                try:
                    if self.drag_on:
                        self.robot.DragTeachSwitch(0)
                except Exception:
                    pass
                try:
                    self.robot.CloseRPC()
                except Exception:
                    pass
        finally:
            self.destroy()


def main() -> None:
    app = Fr5TeachApp()
    app.mainloop()


if __name__ == "__main__":
    main()
