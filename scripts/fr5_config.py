# -*- coding: utf-8 -*-
"""FR5 配置加载与关节软/硬限位。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:
    yaml = None

ROOT = Path(__file__).resolve().parents[1]
CFG_PATH = ROOT / "config" / "fr5_defaults.yaml"

# 官方 URDF 硬限位默认值 (deg)；可被 config limits.joint_limits_hard_deg 覆盖
JOINT_LIMITS_HARD_DEG = [
    (-175.0, 175.0),
    (-265.0, 85.0),
    (-160.0, 160.0),
    (-265.0, 85.0),
    (-175.0, 175.0),
    (-175.0, 175.0),
]

_DEFAULTS: dict[str, Any] = {
    "robot": {"default_ip": "192.168.58.2"},
    "limits": {
        "global_speed_pct_default": 20,
        "jog_vel_pct_default": 20,
        "jog_acc_pct_default": 100,
        "jog_vel_pct_max_ui": 100,
        "movel_vel_pct_default": 20,
        "movej_vel_pct_default": 20,
        "joint_step_deg_max": 5.0,
        "cart_step_mm": 5.0,
        "cart_step_deg": 2.0,
        "jog_max_dis_joint_deg": 30.0,
        "slider_lead_deg": 15.0,
        "jog_max_dis_cart_mm": 50.0,
        "joint_soft_limit_margin_deg": 2.0,
    },
    "coords": {
        "tool_id_default": 0,
        "user_id_base": 0,
        "world_yaw_cw_deg": 45.0,
        "world_wobj_id": 1,
        "prefer_wobj_jog": True,
    },
    "mode": "real",
    "allow_xmlrpc_degrade": False,
    "prefer_xmlrpc_only": False,
    "soft_zero_file": "config/soft_zero.yaml",
}


def build_joint_limits(cfg: dict) -> tuple[list[tuple[float, float]], list[tuple[float, float]], float]:
    """返回 (硬限位, 软限位, margin)。软限位 = 硬限位上下各收紧 margin。"""
    lim = cfg.get("limits", {})
    margin = max(0.0, float(lim.get("joint_soft_limit_margin_deg", 2.0)))
    raw = lim.get("joint_limits_hard_deg")
    hard: list[tuple[float, float]] = []
    if isinstance(raw, list) and len(raw) >= 6:
        for i in range(6):
            pair = raw[i]
            hard.append((float(pair[0]), float(pair[1])))
    else:
        hard = list(JOINT_LIMITS_HARD_DEG)
    soft: list[tuple[float, float]] = []
    for lo, hi in hard:
        slo, shi = lo + margin, hi - margin
        if slo >= shi:
            mid = 0.5 * (lo + hi)
            slo, shi = mid - 0.5, mid + 0.5
        soft.append((slo, shi))
    return hard, soft, margin


def load_cfg(path: Path | None = None) -> dict:
    cfg_path = path or CFG_PATH
    data: dict = {}
    if yaml is not None and cfg_path.is_file():
        with open(cfg_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    for k, v in _DEFAULTS.items():
        if k not in data or (isinstance(v, dict) and not isinstance(data.get(k), dict)):
            data[k] = v if not isinstance(v, dict) else dict(v)
        elif isinstance(v, dict):
            for kk, vv in v.items():
                data[k].setdefault(kk, vv)
    return data
