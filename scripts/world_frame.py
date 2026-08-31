# -*- coding: utf-8 -*-
"""操作台世界系 ↔ 机器人基座系（仅绕 Z 水平转）。"""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple


def yaw_cw_to_rad(yaw_cw_deg: float) -> float:
    """顺时针为正（度）→ 右手系绕 +Z 的弧度（逆时针为正）。"""
    return -math.radians(float(yaw_cw_deg))


def _rotz(theta_rad: float) -> Tuple[Tuple[float, float, float], ...]:
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return (
        (c, -s, 0.0),
        (s, c, 0.0),
        (0.0, 0.0, 1.0),
    )


def _mul(R: Sequence[Sequence[float]], v: Sequence[float]) -> List[float]:
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


def world_delta_to_base(dx_w: float, dy_w: float, dz_w: float, yaw_cw_deg: float) -> List[float]:
    """世界系位移 → 基座系位移。"""
    R = _rotz(yaw_cw_to_rad(yaw_cw_deg))
    return _mul(R, [float(dx_w), float(dy_w), float(dz_w)])


def base_pos_to_world(x: float, y: float, z: float, yaw_cw_deg: float) -> List[float]:
    """基座系位置 → 世界系位置（原点重合，仅绕 Z）。"""
    R_t = _rotz(-yaw_cw_to_rad(yaw_cw_deg))
    return _mul(R_t, [float(x), float(y), float(z)])


def base_pose_to_world(pose6: Iterable[float], yaw_cw_deg: float) -> List[float]:
    """TCP 基座位姿 → 世界系显示用 [x,y,z,rx,ry,rz]；姿态 Rz 减去偏航。"""
    p = [float(v) for v in pose6]
    xyz = base_pos_to_world(p[0], p[1], p[2], yaw_cw_deg)
    # ZYX 展示：水平偏航从基座 Rz 扣掉平台相对角，便于读数对齐操作台
    rz_w = p[5] + float(yaw_cw_deg)  # 基座→世界：顺时针平台 = 读数 +cw
    return [xyz[0], xyz[1], xyz[2], p[3], p[4], rz_w]


def world_axis_unit_in_base(axis: int, yaw_cw_deg: float) -> List[float]:
    """世界系轴单位向量（1=X,2=Y,3=Z）在基座下的分量。"""
    if axis == 1:
        return world_delta_to_base(1.0, 0.0, 0.0, yaw_cw_deg)
    if axis == 2:
        return world_delta_to_base(0.0, 1.0, 0.0, yaw_cw_deg)
    return [0.0, 0.0, 1.0]


def _rpy_deg_to_mat(rx: float, ry: float, rz: float) -> Tuple[Tuple[float, float, float], ...]:
    """ZYX 固有角（度）→ 旋转矩阵。"""
    ax, ay, az = math.radians(rx), math.radians(ry), math.radians(rz)
    cx, sx = math.cos(ax), math.sin(ax)
    cy, sy = math.cos(ay), math.sin(ay)
    cz, sz = math.cos(az), math.sin(az)
    return (
        (cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
        (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
        (-sy, cy * sx, cy * cx),
    )


def _mat_to_rpy_deg(R: Sequence[Sequence[float]]) -> List[float]:
    sy = -R[2][0]
    cy = math.sqrt(max(0.0, 1.0 - sy * sy))
    if cy > 1e-6:
        rx = math.atan2(R[2][1], R[2][2])
        ry = math.atan2(sy, cy)
        rz = math.atan2(R[1][0], R[0][0])
    else:
        rx = math.atan2(-R[1][2], R[1][1])
        ry = math.atan2(sy, cy)
        rz = 0.0
    return [math.degrees(rx), math.degrees(ry), math.degrees(rz)]


def _axis_angle_mat(ax: float, ay: float, az: float, theta: float) -> Tuple[Tuple[float, float, float], ...]:
    c, s = math.cos(theta), math.sin(theta)
    C = 1.0 - c
    return (
        (c + ax * ax * C, ax * ay * C - az * s, ax * az * C + ay * s),
        (ay * ax * C + az * s, c + ay * ay * C, ay * az * C - ax * s),
        (az * ax * C - ay * s, az * ay * C + ax * s, c + az * az * C),
    )


def _matmul3(
    A: Sequence[Sequence[float]], B: Sequence[Sequence[float]]
) -> Tuple[Tuple[float, float, float], ...]:
    return tuple(
        tuple(A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j] for j in range(3))
        for i in range(3)
    )


def rotate_pose_about_axis(
    pose6: Sequence[float], axis_xyz: Sequence[float], angle_deg: float
) -> List[float]:
    """绕过 TCP 原点、方向为 axis_xyz 的轴旋转姿态（位置不变）；angle 为度。"""
    p = [float(v) for v in pose6]
    ax, ay, az = float(axis_xyz[0]), float(axis_xyz[1]), float(axis_xyz[2])
    n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / n, ay / n, az / n
    R = _rpy_deg_to_mat(p[3], p[4], p[5])
    dR = _axis_angle_mat(ax, ay, az, math.radians(float(angle_deg)))
    rx, ry, rz = _mat_to_rpy_deg(_matmul3(dR, R))
    return [p[0], p[1], p[2], rx, ry, rz]
