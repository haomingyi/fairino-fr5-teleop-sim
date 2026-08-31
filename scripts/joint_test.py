#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FR5 单关节最小测试：连接 → 就绪 → JOG 或 MoveJ。

用法（须现场安全、急停在手边）：
  python3 scripts/joint_test.py --confirm-real jog --joint 1 --dir + --seconds 2
  python3 scripts/joint_test.py --confirm-real movej --joint 1 --delta 2
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "third_party" / "fairino-python-sdk" / "linux"))
sys.path.insert(0, str(ROOT / "scripts"))

from fairino_session import (
    call_err,
    connect_robot,
    prepare_motion,
    start_joint_jog,
    stop_jog,
    unwrap_pair,
)
from fr5_config import load_cfg


def read_joint(robot, idx: int) -> float:
    err, pos = unwrap_pair(robot.GetActualJointPosDegree(1))
    if err != 0 or pos is None:
        raise RuntimeError(f"读关节失败 err={err}")
    return float(pos[idx])


def main() -> int:
    ap = argparse.ArgumentParser(description="FR5 单关节 JOG/MoveJ 测试")
    ap.add_argument("--ip", default=None, help="控制器 IP，默认读 config")
    ap.add_argument("--confirm-real", action="store_true", help="确认允许真机运动")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_jog = sub.add_parser("jog", help="StartJOG 指定秒数")
    p_jog.add_argument("--joint", type=int, choices=range(1, 7), required=True)
    p_jog.add_argument("--dir", choices=["+", "-"], required=True)
    p_jog.add_argument("--seconds", type=float, default=2.0)
    p_jog.add_argument("--vel", type=float, default=None, help="速度%%，默认 config")

    p_mv = sub.add_parser("movej", help="MoveJ 相对步进")
    p_mv.add_argument("--joint", type=int, choices=range(1, 7), required=True)
    p_mv.add_argument("--delta", type=float, required=True, help="相对角度 °")
    p_mv.add_argument("--vel", type=float, default=None)

    args = ap.parse_args()
    if not args.confirm_real:
        print("拒绝：须加 --confirm-real 才下发运动")
        return 2

    cfg = load_cfg()
    lim = cfg.get("limits", {})
    vel_max = float(lim.get("jog_vel_pct_max_ui", 100))
    ip = args.ip or str(cfg.get("robot", {}).get("default_ip", "192.168.58.2"))
    raw_vel = float(args.vel if args.vel is not None else lim.get("jog_vel_pct_default", 20))
    vel = max(1.0, min(vel_max, raw_vel))
    allow_degrade = bool(cfg.get("allow_xmlrpc_degrade", True))
    prefer_xmlrpc_only = bool(cfg.get("prefer_xmlrpc_only", True))

    print(f"连接 {ip} …")
    res = connect_robot(ip, allow_degrade=allow_degrade, prefer_xmlrpc_only=prefer_xmlrpc_only)
    print(res.detail)
    robot = res.robot
    try:
        prep = prepare_motion(robot, speed_pct=vel, enable=True)
        print("prepare:", prep)
        if prep.get("enable", -1) != 0:
            print("使能失败，请检查配对/急停/WebApp 错误后重试")
            return 1

        idx = args.joint - 1
        before = read_joint(robot, idx)
        print(f"J{args.joint} before: {before:.3f}°")

        if args.cmd == "jog":
            direction = 1 if args.dir == "+" else 0
            e = start_joint_jog(robot, args.joint, direction, max_dis_deg=90.0, vel_pct=vel)
            print(f"StartJOG err={e}")
            if e != 0:
                return 1
            time.sleep(max(0.1, args.seconds))
            stop_jog(robot, joint=True)
        else:
            err, pos = unwrap_pair(robot.GetActualJointPosDegree(1))
            if err != 0 or pos is None:
                print("读关节失败")
                return 1
            target = list(float(x) for x in pos)
            target[idx] += float(args.delta)
            mv = call_err(robot.MoveJ(joint_pos=target, tool=0, user=0, vel=vel, blendT=0.0))
            print(f"MoveJ err={mv}")
            if mv != 0:
                return 1
            time.sleep(1.0)

        after = read_joint(robot, idx)
        delta = after - before
        print(f"J{args.joint} after: {after:.3f}°  delta={delta:+.3f}°")
        if abs(delta) < 0.05:
            print("警告：几乎无位移，请检查使能/错误码/速度")
            return 1
        print("OK")
        return 0
    finally:
        try:
            stop_jog(robot, joint=True)
        except Exception:
            pass
        try:
            robot.CloseRPC()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
