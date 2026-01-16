"""
Minimal Mahony IMU (6-axis) AHRS implementation.

Quaternion convention: (w, x, y, z)
Gyro units: rad/s
Accel units: m/s^2 (will be normalized internally)

This is intentionally dependency-free (no numpy) for Raspberry Pi usage.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple


def _inv_sqrt(x: float) -> float:
    return 1.0 / math.sqrt(x) if x > 0.0 else 0.0


def _normalize3(x: float, y: float, z: float) -> Tuple[float, float, float]:
    n2 = x * x + y * y + z * z
    if n2 <= 0.0:
        return 0.0, 0.0, 0.0
    inv = _inv_sqrt(n2)
    return x * inv, y * inv, z * inv


def quat_normalize(qw: float, qx: float, qy: float, qz: float) -> Tuple[float, float, float, float]:
    n2 = qw * qw + qx * qx + qy * qy + qz * qz
    if n2 <= 0.0:
        return 1.0, 0.0, 0.0, 0.0
    inv = _inv_sqrt(n2)
    return qw * inv, qx * inv, qy * inv, qz * inv


def quat_mul(
    aw: float, ax: float, ay: float, az: float, bw: float, bx: float, by: float, bz: float
) -> Tuple[float, float, float, float]:
    # (a ⊗ b)
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def gravity_body_from_quat(qw: float, qx: float, qy: float, qz: float) -> Tuple[float, float, float]:
    """
    Returns the direction of gravity (unit vector) expressed in the body frame,
    derived from the current quaternion estimate.
    """
    # This matches common Mahony/Madgwick reference implementation.
    gx = 2.0 * (qx * qz - qw * qy)
    gy = 2.0 * (qw * qx + qy * qz)
    gz = qw * qw - qx * qx - qy * qy + qz * qz
    return _normalize3(gx, gy, gz)


@dataclass
class MahonyIMU:
    kp: float = 1.0
    ki: float = 0.0
    q: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    _iex: float = 0.0
    _iey: float = 0.0
    _iez: float = 0.0

    def update(self, gx: float, gy: float, gz: float, ax: float, ay: float, az: float, dt: float) -> Tuple[float, float, float, float]:
        """
        One IMU update.

        - gx,gy,gz: rad/s
        - ax,ay,az: m/s^2 (will be normalized to a direction)
        - dt: seconds
        """
        if dt <= 0.0:
            return self.q

        qw, qx, qy, qz = self.q

        # Normalize accelerometer measurement
        axn, ayn, azn = _normalize3(ax, ay, az)
        if axn == 0.0 and ayn == 0.0 and azn == 0.0:
            # No accel correction possible; integrate gyro only.
            return self._integrate_gyro(gx, gy, gz, dt)

        # Estimated gravity direction from quaternion
        vx, vy, vz = gravity_body_from_quat(qw, qx, qy, qz)

        # Error is cross product between measured and estimated gravity
        ex = (ayn * vz - azn * vy)
        ey = (azn * vx - axn * vz)
        ez = (axn * vy - ayn * vx)

        if self.ki > 0.0:
            self._iex += ex * dt
            self._iey += ey * dt
            self._iez += ez * dt
        else:
            self._iex = 0.0
            self._iey = 0.0
            self._iez = 0.0

        # Apply feedback terms
        gx_corr = gx + self.kp * ex + self.ki * self._iex
        gy_corr = gy + self.kp * ey + self.ki * self._iey
        gz_corr = gz + self.kp * ez + self.ki * self._iez

        # Integrate quaternion rate: qDot = 0.5 * q ⊗ [0, gyro]
        qDot = quat_mul(qw, qx, qy, qz, 0.0, gx_corr, gy_corr, gz_corr)
        qw += 0.5 * qDot[0] * dt
        qx += 0.5 * qDot[1] * dt
        qy += 0.5 * qDot[2] * dt
        qz += 0.5 * qDot[3] * dt

        self.q = quat_normalize(qw, qx, qy, qz)
        return self.q

    def update_marg(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        mx: float,
        my: float,
        mz: float,
        dt: float,
    ) -> Tuple[float, float, float, float]:
        """
        AHRS update using gyro + accel + magnetometer (MARG).

        - gx,gy,gz: rad/s
        - ax,ay,az: m/s^2 (normalized internally)
        - mx,my,mz: any magnetic units (normalized internally)
        - dt: seconds
        """
        if dt <= 0.0:
            return self.q

        qw, qx, qy, qz = self.q

        axn, ayn, azn = _normalize3(ax, ay, az)
        if axn == 0.0 and ayn == 0.0 and azn == 0.0:
            return self._integrate_gyro(gx, gy, gz, dt)

        mxn, myn, mzn = _normalize3(mx, my, mz)
        if mxn == 0.0 and myn == 0.0 and mzn == 0.0:
            # Can't use mag correction; fall back to IMU update.
            return self.update(gx=gx, gy=gy, gz=gz, ax=ax, ay=ay, az=az, dt=dt)

        # Estimated gravity direction
        vx, vy, vz = gravity_body_from_quat(qw, qx, qy, qz)

        # Reference direction of Earth's magnetic field (in body frame)
        # This is the standard Mahony AHRS approach.
        # Compute 'h' = q ⊗ m ⊗ q* and then derive (bx, bz).
        hx = (
            2.0 * mxn * (0.5 - qy * qy - qz * qz)
            + 2.0 * myn * (qx * qy - qw * qz)
            + 2.0 * mzn * (qx * qz + qw * qy)
        )
        hy = (
            2.0 * mxn * (qx * qy + qw * qz)
            + 2.0 * myn * (0.5 - qx * qx - qz * qz)
            + 2.0 * mzn * (qy * qz - qw * qx)
        )
        bx = math.sqrt(hx * hx + hy * hy)
        bz = (
            2.0 * mxn * (qx * qz - qw * qy)
            + 2.0 * myn * (qy * qz + qw * qx)
            + 2.0 * mzn * (0.5 - qx * qx - qy * qy)
        )

        # Estimated direction of magnetic field (body frame)
        wx = 2.0 * bx * (0.5 - qy * qy - qz * qz) + 2.0 * bz * (qx * qz - qw * qy)
        wy = 2.0 * bx * (qx * qy - qw * qz) + 2.0 * bz * (qw * qx + qy * qz)
        wz = 2.0 * bx * (qw * qy + qx * qz) + 2.0 * bz * (0.5 - qx * qx - qy * qy)

        # Error is sum of cross products between measured and estimated directions
        ex = (ayn * vz - azn * vy) + (myn * wz - mzn * wy)
        ey = (azn * vx - axn * vz) + (mzn * wx - mxn * wz)
        ez = (axn * vy - ayn * vx) + (mxn * wy - myn * wx)

        if self.ki > 0.0:
            self._iex += ex * dt
            self._iey += ey * dt
            self._iez += ez * dt
        else:
            self._iex = 0.0
            self._iey = 0.0
            self._iez = 0.0

        gx_corr = gx + self.kp * ex + self.ki * self._iex
        gy_corr = gy + self.kp * ey + self.ki * self._iey
        gz_corr = gz + self.kp * ez + self.ki * self._iez

        qDot = quat_mul(qw, qx, qy, qz, 0.0, gx_corr, gy_corr, gz_corr)
        qw += 0.5 * qDot[0] * dt
        qx += 0.5 * qDot[1] * dt
        qy += 0.5 * qDot[2] * dt
        qz += 0.5 * qDot[3] * dt

        self.q = quat_normalize(qw, qx, qy, qz)
        return self.q

    def _integrate_gyro(self, gx: float, gy: float, gz: float, dt: float) -> Tuple[float, float, float, float]:
        qw, qx, qy, qz = self.q
        qDot = quat_mul(qw, qx, qy, qz, 0.0, gx, gy, gz)
        qw += 0.5 * qDot[0] * dt
        qx += 0.5 * qDot[1] * dt
        qy += 0.5 * qDot[2] * dt
        qz += 0.5 * qDot[3] * dt
        self.q = quat_normalize(qw, qx, qy, qz)
        return self.q


