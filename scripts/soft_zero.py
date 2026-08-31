#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""软件软零位：记录/加载当前关节角，供 UI「归软零」MoveJ。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:
    yaml = None


def default_path(root: Path) -> Path:
    return root / "config" / "soft_zero.yaml"


def _parse_simple_soft_zero(text: str) -> dict[str, Any]:
    """无 PyYAML 时解析本模块写出的简易 YAML 片段。"""
    data: dict[str, Any] = {}
    key = None
    values: list[float] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.endswith(":") and not line.startswith("-"):
            if key is not None:
                data[key] = values
            key = line[:-1].strip()
            values = []
            continue
        if line.startswith("-") and key is not None:
            values.append(float(line[1:].strip()))
            continue
        if ":" in line and key is None:
            k, v = line.split(":", 1)
            data[k.strip()] = v.strip().strip('"').strip("'")
    if key is not None:
        data[key] = values
    return data


def load_soft_zero(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        raw = path.read_text(encoding="utf-8")
        if yaml is not None:
            data = yaml.safe_load(raw) or {}
        else:
            data = _parse_simple_soft_zero(raw)
    except Exception:
        return None
    joints = data.get("joints_deg")
    if not isinstance(joints, list) or len(joints) < 6:
        return None
    tcp_raw = data.get("tcp_mm_deg") or []
    tcp = [float(x) for x in tcp_raw[:6]] if isinstance(tcp_raw, list) and len(tcp_raw) >= 6 else None
    return {
        "joints_deg": [float(x) for x in joints[:6]],
        "tcp_mm_deg": tcp,
        "note": str(data.get("note") or ""),
    }


def save_soft_zero(
    path: Path,
    joints_deg: list[float],
    *,
    tcp_mm_deg: list[float] | None = None,
    note: str = "",
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "joints_deg": [float(x) for x in joints_deg[:6]],
        "note": note,
    }
    if tcp_mm_deg is not None and len(tcp_mm_deg) >= 6:
        payload["tcp_mm_deg"] = [float(x) for x in tcp_mm_deg[:6]]
    if yaml is None:
        # 无 pyyaml 时简易落盘
        lines = ["joints_deg:"] + [f"  - {float(x)}" for x in joints_deg[:6]]
        if tcp_mm_deg is not None and len(tcp_mm_deg) >= 6:
            lines.append("tcp_mm_deg:")
            lines.extend(f"  - {float(x)}" for x in tcp_mm_deg[:6])
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return
    path.write_text(yaml.safe_dump(payload, allow_unicode=True, sort_keys=False), encoding="utf-8")
