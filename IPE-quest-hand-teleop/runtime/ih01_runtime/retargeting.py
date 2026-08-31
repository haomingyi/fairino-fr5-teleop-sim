"""Minimal six-DOF conversion shared by the portable IH01 simulator.

The upstream streamer delivers landmarks; the fork retargeter produces the
six normalized IH01 targets.  This adapter also supports recorded states.
"""

from __future__ import annotations

from typing import Any

import numpy as np

ACTIVE_DOF_NAMES = (
    "pinky_flex",
    "ring_flex",
    "middle_flex",
    "index_flex",
    "thumb_flex",
    "thumb_opposition",
)


def visual_joints_to_active_dof(
    joints: dict[str, dict[str, float]],
    pinch_distance: float,
    thumb_opposition: float | None = None,
) -> dict[str, float]:
    """Compress visual finger bends to the IH01 active channel order."""
    opposition = 1.0 - pinch_distance / 1.2 if thumb_opposition is None else thumb_opposition
    active = {
        "thumb_opposition": float(np.clip(opposition, 0.0, 1.0)),
        "thumb_flex": float(np.clip(
            0.05 * joints["thumb"]["cmc"]
            + 0.35 * joints["thumb"]["mcp"]
            + 0.60 * joints["thumb"]["ip"], 0.0, 1.0
        )),
    }
    for finger in ("index", "middle", "ring", "pinky"):
        values = joints[finger]
        active[f"{finger}_flex"] = float(np.clip(
            0.35 * values["mcp"] + 0.55 * values["pip"] + 0.10 * values["dip"],
            0.0, 1.0,
        ))
    return {name: round(active[name], 4) for name in ACTIVE_DOF_NAMES}


def active_dof_from_state(state: dict[str, Any]) -> dict[str, float]:
    """Read stored targets where possible, with a visual-joint fallback."""
    recorded = state.get("active_dof_targets")
    if isinstance(recorded, dict) and all(name in recorded for name in ACTIVE_DOF_NAMES):
        return {name: float(np.clip(float(recorded[name]), 0.0, 1.0)) for name in ACTIVE_DOF_NAMES}
    return visual_joints_to_active_dof(
        state["joint_flexion"], float(state.get("pinch_distance", 1.2))
    )
