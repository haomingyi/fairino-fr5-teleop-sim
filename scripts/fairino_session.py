# -*- coding: utf-8 -*-
"""法奥 SDK 连接：CNDE(20005) 不可用时 XML-RPC 降级。"""

from __future__ import annotations

import contextlib
import io
import socket
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Tuple

SAFETY_STOP_CODE = 99
SAFETY_STATE_UNAVAILABLE = 199

_SDK_LINUX = Path(__file__).resolve().parents[1] / "third_party" / "fairino-python-sdk" / "linux"
if _SDK_LINUX.is_dir() and str(_SDK_LINUX) not in sys.path:
    sys.path.insert(0, str(_SDK_LINUX))


@dataclass
class ConnectResult:
    robot: Any
    full: bool          # CNDE + XML-RPC 全通
    degraded: bool      # 仅 XML-RPC，已强制 is_connect
    cnde_ok: bool
    xmlrpc_ok: bool
    detail: str


def probe_ports(ip: str) -> dict[int, bool]:
    ports = (80, 20003, 20004, 20005, 20006, 8080, 8083)
    return {p: _tcp_open(ip, p) for p in ports}


def _tcp_open(ip: str, port: int, timeout_s: float = 1.2) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def _xmlrpc_ok(robot_proxy) -> bool:
    old = socket.getdefaulttimeout()
    try:
        socket.setdefaulttimeout(2.0)
        robot_proxy.GetControllerIP()
        return True
    except Exception:
        return False
    finally:
        socket.setdefaulttimeout(old)


def patch_degraded_robot(robot: Any) -> None:
    """CNDE 不可用时只接受控制器通过 XML-RPC 返回的安全状态。

    旧实现把安全状态固定返回 0。那会在实时状态链路缺失时伪造“安全”，属于
    fail-open。现在任何缺失、格式异常或 RPC 错误都返回 199 并阻止运动。
    """
    proxy = robot.robot

    def _get_error_code() -> Tuple[int, Any]:
        try:
            ret = proxy.GetRobotErrorCode()
            if isinstance(ret, (list, tuple)) and len(ret) >= 3:
                return int(ret[0]), [int(ret[1]), int(ret[2])]
            if isinstance(ret, (list, tuple)) and len(ret) == 2:
                second = ret[1]
                if isinstance(second, (list, tuple)):
                    codes = [int(x) for x in list(second)[:2]]
                    while len(codes) < 2:
                        codes.append(0)
                    return int(ret[0]), codes
                # 常见 [err, main]；prepare_motion 看 codes[0]
                return int(ret[0]), [int(second), 0]
            return call_err(ret), [0, 0]
        except Exception:
            return -1, [-1, -1]

    def _get_safety_code() -> int:
        try:
            emergency = proxy.GetRobotEmergencyStopState()
            safety = proxy.GetSafetyStopState()
            e_err, e_state = _unwrap_scalar(emergency)
            s_err, s_state = _unwrap_states(safety)
            if e_err != 0 or s_err != 0:
                return SAFETY_STATE_UNAVAILABLE
            if int(e_state) != 0 or any(int(value) != 0 for value in s_state):
                return SAFETY_STOP_CODE
            return 0
        except Exception:
            return SAFETY_STATE_UNAVAILABLE

    robot.GetRobotErrorCode = _get_error_code  # type: ignore[method-assign]
    robot.GetSafetyCode = _get_safety_code  # type: ignore[method-assign]


def patch_realtime_safety(robot: Any) -> None:
    """Include emergency-stop state in the bundled SDK's realtime gate."""
    def _get_safety_code() -> int:
        state = robot.robot_state_pkg
        if (
            int(getattr(state, "EmergencyStop", 1)) != 0
            or int(getattr(state, "safety_stop0_state", 1)) != 0
            or int(getattr(state, "safety_stop1_state", 1)) != 0
        ):
            return SAFETY_STOP_CODE
        return 0
    robot.GetSafetyCode = _get_safety_code  # type: ignore[method-assign]


def _unwrap_scalar(ret: Any) -> tuple[int, int]:
    if isinstance(ret, (list, tuple)) and len(ret) >= 2:
        return int(ret[0]), int(ret[1])
    return -1, -1


def _unwrap_states(ret: Any) -> tuple[int, list[int]]:
    if not isinstance(ret, (list, tuple)) or len(ret) < 2:
        return -1, [-1, -1]
    if isinstance(ret[1], (list, tuple)):
        values = [int(value) for value in ret[1][:2]]
    else:
        values = [int(value) for value in ret[1:3]]
    if len(values) != 2:
        return -1, [-1, -1]
    return int(ret[0]), values


def connect_robot(
    ip: str,
    allow_degrade: bool = True,
    *,
    prefer_xmlrpc_only: bool = True,
) -> ConnectResult:
    """连接 Robot.RPC。

    本机固件通常无 20005 CNDE：默认 prefer_xmlrpc_only=True，跳过 CNDE 探测，
    直连 XML-RPC 20003，并抑制官方 SDK 的 CNDE 失败刷屏。
    """
    from fairino import Robot
    import fairino.Robot as robot_mod

    orig_cnde_connect = robot_mod.FRCNDEClient.connect

    def _skip_cnde(self, ip_addr: str | None = None, port: int | None = None) -> int:
        # 不建 TCP、不 print，直接视为未开 CNDE
        try:
            self._sock_com_err[0] = robot_mod.FRCNDEClient.ERR_SOCKET_COM_FAILED
        except Exception:
            pass
        return robot_mod.FRCNDEClient.ERR_SOCKET_COM_FAILED

    if prefer_xmlrpc_only:
        robot_mod.FRCNDEClient.connect = _skip_cnde  # type: ignore[method-assign]

    try:
        # 吞掉 SDK __init__ 里「CNDE失败/is_connect=False」等调试输出
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            robot = Robot.RPC(ip)
    finally:
        if prefer_xmlrpc_only:
            robot_mod.FRCNDEClient.connect = orig_cnde_connect  # type: ignore[method-assign]

    rpc_cls = Robot.RPC
    xmlrpc_ok = _xmlrpc_ok(robot.robot)

    if rpc_cls.is_connect and not prefer_xmlrpc_only:
        patch_realtime_safety(robot)
        return ConnectResult(
            robot=robot,
            full=True,
            degraded=False,
            cnde_ok=True,
            xmlrpc_ok=True,
            detail="CNDE(20005)+XML-RPC(20003) 全通",
        )

    if xmlrpc_ok and (allow_degrade or prefer_xmlrpc_only):
        rpc_cls.is_connect = True
        patch_degraded_robot(robot)
        return ConnectResult(
            robot=robot,
            full=False,
            degraded=True,
            cnde_ok=False,
            xmlrpc_ok=True,
            detail=(
                "XML-RPC 20003 降级连接；运动仅在控制器能通过 XML-RPC 返回"
                "急停与安全停止状态时允许，否则 fail-closed"
            ),
        )

    raise ConnectionError(
        f"SDK 连接失败 (XML-RPC:{xmlrpc_ok})。请检查 192.168.58.x 网段与控制箱 20003 端口。"
    )


def call_err(ret: Any) -> int:
    """统一解析 SDK 返回错误码。"""
    if isinstance(ret, int):
        return ret
    if isinstance(ret, (list, tuple)) and ret:
        return int(ret[0])
    return 0


def unwrap_pair(ret: Any) -> Tuple[int, Any]:
    """解析 (err, payload)。兼容扁平 [err, v1..v6, ...] 与三元组 (err, coord, ref)。"""
    if isinstance(ret, (list, tuple)) and ret:
        err = int(ret[0])
        if len(ret) >= 7 and not isinstance(ret[1], (list, tuple)):
            return err, [ret[i] for i in range(1, 7)]
        if len(ret) >= 2:
            return err, ret[1]
        return err, None
    return call_err(ret), None


def prepare_motion(robot: Any, speed_pct: float = 15.0, enable: bool = True) -> dict[str, int]:
    """标准就绪：手动模式 → 全局速度 → 清错 → 上使能。"""
    out: dict[str, int] = {}
    out["safety"] = int(robot.GetSafetyCode())
    if out["safety"] != 0:
        out["enable"] = out["safety"]
        return out
    out["mode"] = call_err(robot.Mode(1))
    out["speed"] = call_err(robot.SetSpeed(speed_pct))
    if out["mode"] != 0 or out["speed"] != 0:
        out["enable"] = out["mode"] or out["speed"]
        return out
    err, codes = unwrap_pair(robot.GetRobotErrorCode())
    if isinstance(codes, (list, tuple)) and codes:
        main = int(codes[0])
    elif isinstance(codes, (int, float)):
        main = int(codes)
    else:
        main = 0
    if err != 0 or main != 0:
        out["reset"] = call_err(robot.ResetAllError())
        check_err, check_codes = unwrap_pair(robot.GetRobotErrorCode())
        if isinstance(check_codes, (list, tuple)) and check_codes:
            remaining_main = int(check_codes[0])
        else:
            remaining_main = -1
        if out["reset"] != 0 or check_err != 0 or remaining_main != 0:
            out["enable"] = out["reset"] or check_err or remaining_main or -1
            return out
    else:
        out["reset"] = 0
    if enable:
        out["enable"] = call_err(robot.RobotEnable(1))
    return out


def start_joint_jog(
    robot: Any,
    joint_nb: int,
    direction: int,
    *,
    max_dis_deg: float = 90.0,
    vel_pct: float = 15.0,
    acc_pct: float = 40.0,
) -> int:
    """单关节 StartJOG；max_dis 取大，靠 ImmStopJOG 松手停。"""
    return call_err(
        robot.StartJOG(0, int(joint_nb), int(direction), float(max_dis_deg), vel=vel_pct, acc=acc_pct)
    )


def stop_jog(robot: Any, joint: bool = True) -> None:
    try:
        robot.ImmStopJOG()
    except Exception:
        try:
            robot.StopJOG(1 if joint else 3)
        except Exception:
            pass


def goto_joints_by_jog(
    robot: Any,
    target_deg: list,
    *,
    vel_pct: float = 12.0,
    acc_pct: float = 40.0,
    tol_deg: float = 0.4,
    stop_flag: Any = None,
) -> dict[str, Any]:
    """逐轴 StartJOG 到位（本机拖动后 MoveJ 常返回 14，JOG 仍可用）。

    stop_flag: 可选，返回 True 时中止。返回 ok / moved / detail / last_err。
    """
    import time as _time

    tgt = [float(x) for x in list(target_deg)[:6]]
    vel = max(1.0, min(40.0, float(vel_pct)))
    acc = max(1.0, min(100.0, float(acc_pct)))
    tol = max(0.15, float(tol_deg))
    moved: list[str] = []
    last_err = 0

    def _should_stop() -> bool:
        if stop_flag is None:
            return False
        try:
            return bool(stop_flag())
        except Exception:
            return False

    for i in range(6):
        if _should_stop():
            stop_jog(robot, joint=True)
            return {"ok": False, "moved": moved, "last_err": last_err, "detail": "用户中止"}
        err, cur = unwrap_pair(robot.GetActualJointPosDegree(1))
        if err != 0 or cur is None:
            return {"ok": False, "moved": moved, "last_err": err, "detail": f"读角失败 {err}"}
        cur_a = float(cur[i])
        delta = tgt[i] - cur_a
        if abs(delta) <= tol:
            continue
        direction = 1 if delta > 0 else 0
        # 分段：单次 max_dis 不宜过大，便于中途急停
        remain = abs(delta)
        while remain > tol:
            if _should_stop():
                stop_jog(robot, joint=True)
                return {"ok": False, "moved": moved, "last_err": last_err, "detail": "用户中止"}
            step = min(remain, 25.0)
            last_err = start_joint_jog(
                robot, i + 1, direction, max_dis_deg=step, vel_pct=vel, acc_pct=acc
            )
            if last_err != 0:
                stop_jog(robot, joint=True)
                return {
                    "ok": False,
                    "moved": moved,
                    "last_err": last_err,
                    "detail": f"J{i+1} StartJOG={last_err}",
                }
            # 按步长粗估等待，并轮询靠近目标
            t_end = _time.time() + max(0.4, step / max(3.0, vel * 0.35))
            while _time.time() < t_end:
                if _should_stop():
                    stop_jog(robot, joint=True)
                    return {"ok": False, "moved": moved, "last_err": last_err, "detail": "用户中止"}
                _time.sleep(0.08)
                e2, c2 = unwrap_pair(robot.GetActualJointPosDegree(1))
                if e2 == 0 and c2 is not None and abs(float(c2[i]) - tgt[i]) <= tol:
                    break
            stop_jog(robot, joint=True)
            _time.sleep(0.12)
            e3, c3 = unwrap_pair(robot.GetActualJointPosDegree(1))
            if e3 != 0 or c3 is None:
                return {"ok": False, "moved": moved, "last_err": e3, "detail": f"J{i+1} 读角失败"}
            cur_a = float(c3[i])
            remain = abs(tgt[i] - cur_a)
            direction = 1 if (tgt[i] - cur_a) > 0 else 0
        moved.append(f"J{i+1}:{cur_a:.1f}->{tgt[i]:.1f}")

    return {"ok": True, "moved": moved, "last_err": 0, "detail": ", ".join(moved) or "已在软零附近"}


def move_l_compat(
    robot: Any,
    desc_pos: list,
    *,
    tool: int = 0,
    user: int = 0,
    vel: float = 20.0,
    blend_r: float = -1.0,
    joint_pos: list | None = None,
) -> int:
    """笛卡尔直线兼容封装。

    本机控制箱固件与新版 SDK 的 MoveL 单数组打包不兼容（Fault -502）。
    优先尝试旧式分参数 MoveL；失败则 IK + MoveJ（点到点，短步进时近似直线）。
    """
    from xmlrpc.client import Fault

    desc = [float(x) for x in desc_pos[:6]]
    if joint_pos is None:
        err, joints = unwrap_pair(robot.GetInverseKin(0, desc, -1))
        if err != 0 or joints is None:
            return err if err else -1
        if not isinstance(joints, (list, tuple)):
            return -1
        joints = [float(x) for x in list(joints)[:6]]
    else:
        joints = [float(x) for x in joint_pos[:6]]

    proxy = robot.robot
    # 1) 旧固件常见：与 MoveJ 类似的分参数
    for args in (
        (joints, desc, int(tool), int(user), float(vel), 0.0, 100.0, float(blend_r)),
        (
            joints,
            desc,
            int(tool),
            int(user),
            float(vel),
            0.0,
            100.0,
            float(blend_r),
            [0.0, 0.0, 0.0, 0.0],
            0,
            0,
            [0.0] * 6,
        ),
        (joints + desc,),  # 仅 12 元
    ):
        try:
            return call_err(proxy.MoveL(*args))
        except Fault:
            continue
        except Exception:
            continue

    # 2) 新 SDK 路径（部分固件可用）
    try:
        return call_err(
            robot.MoveL(desc_pos=desc, tool=tool, user=user, joint_pos=joints, vel=vel, blendR=blend_r)
        )
    except Fault:
        pass
    except Exception:
        pass

    # 3) 短行程可用：关节空间到位
    return call_err(
        robot.MoveJ(
            joint_pos=joints,
            tool=tool,
            user=user,
            vel=vel,
            blendT=-1.0 if blend_r < 0 else 200.0,
        )
    )


def joints_violate_soft(
    joints: list,
    soft_limits: list[tuple[float, float]],
    *,
    eps: float = 0.05,
) -> list[tuple[int, float, float, float]]:
    """返回越界关节 [(idx, angle, lo, hi), ...]。"""
    bad = []
    for i, ang in enumerate(list(joints)[:6]):
        if i >= len(soft_limits):
            break
        lo, hi = soft_limits[i]
        a = float(ang)
        if a < lo - eps or a > hi + eps:
            bad.append((i, a, lo, hi))
    return bad


def cart_goto_async(
    robot: Any,
    desc_pos: list,
    *,
    tool: int = 0,
    user: int = 0,
    vel: float = 20.0,
    joint_ref: list | None = None,
    soft_limits: list[tuple[float, float]] | None = None,
) -> int:
    """非阻塞笛卡尔到位：IK(优先 Ref) + MoveJ(blendT>0)，松手可 StopMotion。

    若提供 soft_limits，逆解关节越软限位则拒绝下发（返回 1001）。
    """
    desc = [float(x) for x in desc_pos[:6]]
    joints = None
    if joint_ref is not None:
        err, j = unwrap_pair(robot.GetInverseKinRef(0, desc, list(map(float, joint_ref[:6]))))
        if err == 0 and isinstance(j, (list, tuple)):
            joints = [float(x) for x in list(j)[:6]]
    if joints is None:
        err, j = unwrap_pair(robot.GetInverseKin(0, desc, -1))
        if err != 0 or not isinstance(j, (list, tuple)):
            return err if err else -1
        joints = [float(x) for x in list(j)[:6]]
    if soft_limits is not None:
        bad = joints_violate_soft(joints, soft_limits)
        if bad:
            return 1001
    return call_err(
        robot.MoveJ(joint_pos=joints, tool=tool, user=user, vel=vel, blendT=50.0)
    )


def activate_world_wobj(
    robot: Any,
    *,
    wobj_id: int = 1,
    yaw_cw_deg: float = 45.0,
) -> dict[str, Any]:
    """写入并尝试激活「操作台世界」工件系（相对基座绕 Z 顺时针 yaw）。

    本机固件 SetWObj* 为两参数 (id, coord)；新 SDK 三参数会 Fault -502。
    返回: ok / written / cur_applied / offset / detail
    """
    from xmlrpc.client import Fault

    # 右手系 Rz = -顺时针角
    rz = -float(yaw_cw_deg)
    coord = [0.0, 0.0, 0.0, 0.0, 0.0, rz]
    wid = int(wobj_id)
    proxy = robot.robot
    written = False
    last_err: Any = None
    # 先两参（本机固件），再三参（新固件/SDK）
    for name, fn in (
        ("SetWObjList2", lambda: proxy.SetWObjList(wid, coord)),
        ("SetWObjCoord2", lambda: proxy.SetWObjCoord(wid, coord)),
        ("SetWObjList3", lambda: proxy.SetWObjList(wid, coord, 0)),
        ("SetWObjCoord3", lambda: proxy.SetWObjCoord(wid, coord, 0)),
    ):
        try:
            last_err = fn()
            if call_err(last_err) == 0:
                written = True
                last_err = f"{name}:0"
                break
            last_err = f"{name}:{last_err}"
        except Fault as e:
            last_err = f"{name}:{e}"
        except Exception as e:
            last_err = f"{name}:{e}"

    offset = None
    cur = None
    stored = None
    # XML-RPC 可读：按 id 取已写工件系（无 CNDE 时 GetCurWObjCoord 只读陈旧 state_pkg）
    try:
        err, off = unwrap_pair(robot.GetWObjCoordWithID(wid))
        if err == 0 and isinstance(off, (list, tuple)):
            stored = [float(x) for x in list(off)[:6]]
    except Exception:
        pass
    try:
        err, off = unwrap_pair(robot.GetWObjOffset(1))
        if err == 0 and isinstance(off, (list, tuple)):
            offset = [float(x) for x in list(off)[:6]]
    except Exception:
        pass
    try:
        err, c = unwrap_pair(robot.GetCurWObjCoord())
        if err == 0 and isinstance(c, (list, tuple)):
            cur = [float(x) for x in list(c)[:6]]
    except Exception:
        pass

    cur_applied = bool(cur and abs(cur[5]) > 0.5)
    stored_ok = bool(stored and abs(stored[5] - rz) < 0.6)
    offset_ok = bool(offset and abs(offset[5] - rz) < 0.6)
    detail = (
        f"wobj#{wid} rz={rz:.1f} written={written} stored={stored} offset={offset} cur={cur} "
        f"last={last_err}"
    )
    return {
        "ok": written and (stored_ok or offset_ok or cur_applied),
        "written": written,
        "cur_applied": cur_applied,
        "offset_ok": offset_ok,
        "stored_ok": stored_ok,
        "offset": offset,
        "stored": stored,
        "cur": cur,
        "coord": coord,
        "detail": detail,
    }
