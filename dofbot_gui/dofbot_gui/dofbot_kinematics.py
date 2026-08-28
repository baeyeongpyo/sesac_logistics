#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DOFBOT 5-DOF 로봇팔 정기구학(FK) / 역기구학(IK) 모듈.

URDF 기반 링크 파라미터 (단위: m):
  joint1: base→link1  Tz(0.064)                        rot Z
  joint2: link1→link2  Tz(0.0435) Ry(π/2)              rot Z
  joint3: link2→link3  Tx(-0.08285)                     rot Z
  joint4: link3→link4  Tx(-0.08285)                     rot Z
  joint5: link4→link5  T(-0.07385, -0.00215, 0) Ry(-π/2) rot Z

서보 각도 ↔ 내부 라디안 변환:
  내부(rad) = (servo_deg − 90) × π/180
  servo_deg = 내부(rad) × 180/π + 90
"""

import math
import numpy as np

# ── 상수 ──────────────────────────────────────────────
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# 조인트 한계 (서보 각도 기준)
JOINT_LIMITS_DEG = [
    (0, 180),   # joint1
    (0, 180),   # joint2
    (0, 180),   # joint3
    (0, 180),   # joint4
    (0, 270),   # joint5
]

# 그리퍼(joint6) 길이 (m) — wrist FK 끝(joint5 EE)에서 gripper jaw까지.
# dofbot.urdf joint5 x=-0.18385 = wrist간격(0.07385) + gripper body(0.11)
# FK는 wrist 위치만 계산하므로 pick시 이 값만큼 wrist를 뒤로 이동해야 함.
GRIPPER_LENGTH = 0.11


# ── 변환 행렬 유틸리티 ─────────────────────────────────
def _Rz(q):
    """Z축 회전 4×4."""
    c, s = math.cos(q), math.sin(q)
    return np.array([
        [ c, -s, 0, 0],
        [ s,  c, 0, 0],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1],
    ])


def _Ry(q):
    """Y축 회전 4×4."""
    c, s = math.cos(q), math.sin(q)
    return np.array([
        [ c, 0,  s, 0],
        [ 0, 1,  0, 0],
        [-s, 0,  c, 0],
        [ 0, 0,  0, 1],
    ])


def _Rx(q):
    """X축 회전 4×4."""
    c, s = math.cos(q), math.sin(q)
    return np.array([
        [1,  0,  0, 0],
        [0,  c, -s, 0],
        [0,  s,  c, 0],
        [0,  0,  0, 1],
    ])


def _Trans(x, y, z):
    """평행이동 4×4."""
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def _rot2rpy(R):
    """3×3 회전행렬 → (roll, pitch, yaw) 라디안.  ZYX 컨벤션."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll  = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = math.atan2(R[1, 0], R[0, 0])
    else:
        roll  = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = 0.0
    return roll, pitch, yaw


def _rpy2rot(roll, pitch, yaw):
    """(roll, pitch, yaw) 라디안 → 3×3 회전행렬.  ZYX 컨벤션."""
    Rz = _Rz(yaw)[:3, :3]
    Ry = _Ry(pitch)[:3, :3]
    Rx = _Rx(roll)[:3, :3]
    return Rz @ Ry @ Rx


# ── 정기구학 (FK) ─────────────────────────────────────
def _joint_transforms(q):
    """
    5개 조인트 라디안 → 각 조인트별 변환행렬 리스트.
    q: (q1, q2, q3, q4, q5)  내부 라디안 (서보 90도 = 0 rad)
    """
    T1 = _Trans(0, 0, 0.064)              @ _Rz(q[0])
    T2 = _Trans(0, 0, 0.0435) @ _Ry(math.pi / 2) @ _Rz(q[1])
    T3 = _Trans(-0.08285, 0, 0)           @ _Rz(q[2])
    T4 = _Trans(-0.08285, 0, 0)           @ _Rz(q[3])
    T5 = _Trans(-0.07385, -0.00215, 0) @ _Ry(-math.pi / 2) @ _Rz(q[4])
    return [T1, T2, T3, T4, T5]


def forward_kinematics(q):
    """
    정기구학: 5개 조인트 라디안 → 말단 위치/자세.

    Parameters
    ----------
    q : array-like, shape (5,)
        내부 라디안 (서보 90° = 0 rad).

    Returns
    -------
    xyz : (x, y, z) 단위 m
    rpy : (roll, pitch, yaw) 단위 rad
    T   : 4×4 동차 변환 행렬
    """
    transforms = _joint_transforms(q)
    T = np.eye(4)
    for Ti in transforms:
        T = T @ Ti
    xyz = (T[0, 3], T[1, 3], T[2, 3])
    rpy = _rot2rpy(T[:3, :3])
    return xyz, rpy, T


def forward_kinematics_servo(servo_angles_deg):
    """
    서보 각도(도) → 말단 위치/자세.

    Parameters
    ----------
    servo_angles_deg : array-like, shape (5,)
        서보 각도 [0-180, 0-180, 0-180, 0-180, 0-270].

    Returns
    -------
    xyz_cm  : (x, y, z) 단위 cm
    rpy_deg : (roll, pitch, yaw) 단위 도
    """
    q = [(a - 90) * DEG2RAD for a in servo_angles_deg]
    xyz, rpy, _ = forward_kinematics(q)
    xyz_cm = (xyz[0] * 100, xyz[1] * 100, xyz[2] * 100)
    rpy_deg = (rpy[0] * RAD2DEG, rpy[1] * RAD2DEG, rpy[2] * RAD2DEG)
    return xyz_cm, rpy_deg


# ── 역기구학 (IK) ─────────────────────────────────────
def _numerical_jacobian(q, eps=1e-6):
    """중앙차분으로 6×5 야코비안 산출 (dx,dy,dz,droll,dpitch,dyaw) / dq."""
    J = np.zeros((6, 5))
    _, _, T0 = forward_kinematics(q)
    p0 = np.array([T0[0, 3], T0[1, 3], T0[2, 3]])
    rpy0 = np.array(_rot2rpy(T0[:3, :3]))
    for i in range(5):
        dq = np.array(q, dtype=float)
        dq[i] += eps
        _, _, T1 = forward_kinematics(dq)
        p1 = np.array([T1[0, 3], T1[1, 3], T1[2, 3]])
        rpy1 = np.array(_rot2rpy(T1[:3, :3]))
        J[:3, i] = (p1 - p0) / eps
        J[3:, i] = (rpy1 - rpy0) / eps
    return J


def inverse_kinematics(target_xyz, target_rpy,
                       q_init=None,
                       max_iter=500,
                       tol_pos=5e-4,
                       tol_rot=5e-3,
                       damping=0.01,
                       pos_weight=1.0,
                       rot_weight=0.3):
    """
    역기구학: 목표 좌표 → 5개 조인트 각도 (라디안).
    5-DOF 팔이므로 위치(3) 우선, 자세(3)는 가중치를 낮게 설정.

    Parameters
    ----------
    target_xyz : (x, y, z)  단위 m
    target_rpy : (roll, pitch, yaw)  단위 rad
    q_init     : 초기 추정치 (5,), None이면 홈 자세
    max_iter   : 최대 반복 횟수
    tol_pos    : 위치 오차 허용치 (m)
    tol_rot    : 자세 오차 허용치 (rad)
    damping    : 댐핑 계수 (DLS)
    pos_weight : 위치 오차 가중치
    rot_weight : 자세 오차 가중치 (5-DOF이므로 낮게)

    Returns
    -------
    q       : (5,) 라디안 또는 None (실패)
    success : bool
    """
    if q_init is None:
        q = np.zeros(5)
    else:
        q = np.array(q_init, dtype=float)

    target = np.array([
        target_xyz[0], target_xyz[1], target_xyz[2],
        target_rpy[0], target_rpy[1], target_rpy[2],
    ])

    # 가중치 행렬 (W): 위치 우선
    W = np.diag([pos_weight, pos_weight, pos_weight,
                 rot_weight, rot_weight, rot_weight])

    limits_rad = [((lo - 90) * DEG2RAD, (hi - 90) * DEG2RAD)
                  for lo, hi in JOINT_LIMITS_DEG]

    for it in range(max_iter):
        xyz, rpy, _ = forward_kinematics(q)
        current = np.array([xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]])
        err = target - current

        # 각도 래핑
        for k in range(3, 6):
            while err[k] > math.pi:
                err[k] -= 2.0 * math.pi
            while err[k] < -math.pi:
                err[k] += 2.0 * math.pi

        pos_err = np.linalg.norm(err[:3])
        rot_err = np.linalg.norm(err[3:])
        if pos_err < tol_pos and rot_err < tol_rot:
            return q.tolist(), True

        # 가중 오차
        we = W @ err

        J = _numerical_jacobian(q)
        WJ = W @ J

        # 적응형 댐핑: 오차가 클 때 댐핑 증가
        lam = damping * (1.0 + 10.0 * min(pos_err, 0.1))
        lam2 = lam ** 2

        # Damped Least Squares: dq = (WJ)^T ((WJ)(WJ)^T + λ²I)^{-1} we
        A = WJ @ WJ.T + lam2 * np.eye(6)
        dq = WJ.T @ np.linalg.solve(A, we)

        # 스텝 크기 제한
        step_norm = np.linalg.norm(dq)
        max_step = 0.3
        if step_norm > max_step:
            dq = dq * (max_step / step_norm)

        q += dq

        # 조인트 한계 클램프
        for i in range(5):
            q[i] = max(limits_rad[i][0], min(limits_rad[i][1], q[i]))

    return q.tolist(), False


def inverse_kinematics_servo(target_xyz_cm, target_rpy_deg,
                             q_init_servo=None, **kwargs):
    """
    서보 각도(도) 기준 역기구학.

    Parameters
    ----------
    target_xyz_cm  : (x, y, z) 단위 cm
    target_rpy_deg : (roll, pitch, yaw) 단위 도
    q_init_servo   : 초기 서보 각도 (5,), None이면 홈(90,90,90,90,135)

    Returns
    -------
    servo_angles : list[5] (도)  또는 None
    success      : bool
    """
    target_xyz = [v / 100.0 for v in target_xyz_cm]
    target_rpy = [v * DEG2RAD for v in target_rpy_deg]

    if q_init_servo is not None:
        q_init = [(a - 90) * DEG2RAD for a in q_init_servo]
    else:
        q_init = None

    q, ok = inverse_kinematics(target_xyz, target_rpy, q_init=q_init, **kwargs)
    if q is None:
        return None, False

    servo = [qi * RAD2DEG + 90 for qi in q]
    return servo, ok


# ── 편의 함수 ─────────────────────────────────────────
def get_all_joint_positions(q):
    """
    5개 조인트 라디안 → 각 관절 위치(x,y,z) 리스트 (6개: base + 5관절).
    시각화 등에 활용.
    """
    transforms = _joint_transforms(q)
    positions = []
    T = np.eye(4)
    positions.append((T[0, 3], T[1, 3], T[2, 3]))
    for Ti in transforms:
        T = T @ Ti
        positions.append((T[0, 3], T[1, 3], T[2, 3]))
    return positions


def get_all_joint_positions_servo(servo_angles_deg):
    """서보 각도(도) → 각 관절 위치(cm) 리스트."""
    q = [(a - 90) * DEG2RAD for a in servo_angles_deg]
    positions = get_all_joint_positions(q)
    return [(x * 100, y * 100, z * 100) for x, y, z in positions]


# ── CLI 테스트 ────────────────────────────────────────
if __name__ == "__main__":
    import sys

    print("=== DOFBOT Kinematics Test ===\n")

    # 홈 자세 FK
    home = [90, 90, 90, 90, 135]
    xyz_cm, rpy_deg = forward_kinematics_servo(home)
    print(f"[FK] Home servo angles: {home}")
    print(f"     XYZ (cm) : ({xyz_cm[0]:.3f}, {xyz_cm[1]:.3f}, {xyz_cm[2]:.3f})")
    print(f"     RPY (deg): ({rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f})")
    print()

    # 각 관절 위치
    positions = get_all_joint_positions_servo(home)
    print("[FK] Joint positions (cm):")
    for i, (x, y, z) in enumerate(positions):
        label = "base" if i == 0 else f"joint{i}"
        print(f"     {label}: ({x:.3f}, {y:.3f}, {z:.3f})")
    print()

    # IK 테스트: FK 결과를 다시 역변환
    print(f"[IK] Target XYZ={xyz_cm}, RPY={rpy_deg}")
    servo_result, ok = inverse_kinematics_servo(xyz_cm, rpy_deg)
    if ok:
        print(f"     Result servo angles: {[round(a, 2) for a in servo_result]}")
        # FK 재검증
        xyz2, rpy2 = forward_kinematics_servo(servo_result)
        print(f"     Re-FK XYZ (cm) : ({xyz2[0]:.3f}, {xyz2[1]:.3f}, {xyz2[2]:.3f})")
        print(f"     Re-FK RPY (deg): ({rpy2[0]:.2f}, {rpy2[1]:.2f}, {rpy2[2]:.2f})")
    else:
        print(f"     IK failed (best attempt: {[round(a, 2) for a in servo_result]})")

    # 추가: 사용자 입력
    if len(sys.argv) >= 6:
        angles = [float(x) for x in sys.argv[1:6]]
        print(f"\n[FK] Custom servo angles: {angles}")
        xyz_cm, rpy_deg = forward_kinematics_servo(angles)
        print(f"     XYZ (cm) : ({xyz_cm[0]:.3f}, {xyz_cm[1]:.3f}, {xyz_cm[2]:.3f})")
        print(f"     RPY (deg): ({rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f})")
