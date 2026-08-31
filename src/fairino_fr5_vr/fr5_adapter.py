"""Commissioning-only FAIRINO SDK adapter with fail-closed tool validation."""
from __future__ import annotations
from pathlib import Path
import sys
from typing import Any

def _error_code(value: Any) -> int:
    if isinstance(value, int): return value
    if isinstance(value, (list, tuple)) and value: return int(value[0])
    return -1

def validate_end_effector(cfg: dict) -> list[str]:
    """Return reasons why the mounted IH01 is not ready for physical motion."""
    ee = cfg.get("end_effector") or {}; problems: list[str] = []
    if ee.get("model") != "IH01-X1-R": problems.append("end_effector.model must be IH01-X1-R")
    if ee.get("approved_for_motion") is not True: problems.append("end_effector.approved_for_motion is false")
    mass = ee.get("mass_kg")
    if not isinstance(mass, (int, float)) or not 0.0 < float(mass) <= 5.0: problems.append("measured mass_kg must be in (0, 5]")
    com = ee.get("center_of_mass_mm")
    if not isinstance(com, list) or len(com) != 3: problems.append("measured center_of_mass_mm needs three values")
    tcp = ee.get("flange_to_tcp_mm_deg")
    if not isinstance(tcp, list) or len(tcp) != 6: problems.append("measured flange_to_tcp_mm_deg needs six values")
    if int(cfg.get("tool_id", 0)) == 0: problems.append("tool_id 0 is the flange; use a calibrated IH01 tool frame")
    return problems

class FR5Adapter:
    """Own one SDK servo session; never connects until configuration is approved."""
    def __init__(self, root: Path, cfg: dict) -> None:
        self.root, self.cfg, self.robot, self.servo_active = root, cfg, None, False
    def connect_and_enable(self) -> tuple[float, ...]:
        problems = validate_end_effector(self.cfg)
        if problems: raise RuntimeError("FR5/IH01 commissioning gate: " + "; ".join(problems))
        scripts = self.root / "scripts"
        if str(scripts) not in sys.path: sys.path.insert(0, str(scripts))
        from fairino_session import connect_robot, prepare_motion, unwrap_pair
        result = connect_robot(str(self.cfg["ip"]), allow_degrade=False, prefer_xmlrpc_only=False)
        self.robot = result.robot
        prep = prepare_motion(self.robot, speed_pct=float(self.cfg["speed_pct"]), enable=True)
        if prep.get("enable") != 0: self.close(); raise RuntimeError(f"FR5 enable failed: {prep}")
        err, pose = unwrap_pair(self.robot.GetActualTCPPose(0))
        if err != 0 or pose is None: self.close(); raise RuntimeError(f"FR5 pose read failed: {err}")
        servo_err = _error_code(self.robot.ServoMoveStart())
        if servo_err != 0: self.close(); raise RuntimeError(f"FR5 ServoMoveStart failed: {servo_err}")
        self.servo_active = True
        return tuple(float(value) for value in pose[:6])
    def send(self, pose: tuple[float, ...]) -> None:
        if self.robot is None or not self.servo_active: raise RuntimeError("FR5 servo session is not active")
        if len(pose) != 6: raise ValueError("FR5 Cartesian pose needs six values")
        safety = int(self.robot.GetSafetyCode())
        if safety != 0: self.stop(); raise RuntimeError(f"FR5 safety state blocks motion: {safety}")
        err = _error_code(self.robot.ServoCart(0, list(pose), [0.0,0.0,0.0,0.0], cmdT=float(self.cfg.get("servo_period_s",0.008))))
        if err != 0: self.stop(); raise RuntimeError(f"FR5 ServoCart failed: {err}")
    def stop(self) -> None:
        if self.robot is None: return
        if self.servo_active:
            try: self.robot.ServoMoveEnd()
            except Exception: pass
            self.servo_active = False
        try: self.robot.StopMotion()
        finally:
            try: self.robot.RobotEnable(0)
            except Exception: pass
    def close(self) -> None:
        self.stop()
        if self.robot is not None:
            try: self.robot.CloseRPC()
            except Exception: pass
            self.robot = None
