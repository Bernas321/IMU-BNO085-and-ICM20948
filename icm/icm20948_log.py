#!/usr/bin/env python3
"""
ICM-20948 I2C logger: quaternion + roll/pitch/yaw + calibrated linear accel + gyro.

Output: one sample per line in a fixed schema (CSV header once, or JSONL objects).

This uses Adafruit's CircuitPython driver for ICM-20948 (via Blinka) for I2C reads,
but keeps fusion/calibration/output self-contained.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from typing import IO, Dict, Optional, Tuple

from mahony import MahonyIMU, gravity_body_from_quat


G0 = 9.80665  # m/s^2


def _rpy_from_quat_deg(qw: float, qx: float, qy: float, qz: float) -> Tuple[float, float, float]:
    # Aerospace Tait-Bryan angles: roll(X), pitch(Y), yaw(Z)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def _open_out(path: str) -> IO[str]:
    if path == "-" or path == "":
        return sys.stdout
    return open(path, "w", buffering=1)


@dataclass
class Biases:
    gyro_bias_rads: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_bias_mps2: Tuple[float, float, float] = (0.0, 0.0, 0.0)


def _calibrate_stationary(
    read_sample_fn,
    seconds: float,
    rate_hz: float,
) -> Biases:
    """
    Simple stationary calibration:
    - gyro bias = mean gyro while still
    - accel bias = shift mean accel to have magnitude g (keeps direction)
    """
    if seconds <= 0.0:
        return Biases()

    n = max(1, int(seconds * rate_hz))
    sum_ax = sum_ay = sum_az = 0.0
    sum_gx = sum_gy = sum_gz = 0.0

    period = 1.0 / rate_hz if rate_hz > 0 else 0.01
    t_next = time.monotonic()

    for _ in range(n):
        ax, ay, az, gx, gy, gz = read_sample_fn()
        sum_ax += ax
        sum_ay += ay
        sum_az += az
        sum_gx += gx
        sum_gy += gy
        sum_gz += gz

        # keep consistent spacing
        t_next += period
        delay = t_next - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    mean_ax = sum_ax / n
    mean_ay = sum_ay / n
    mean_az = sum_az / n
    mean_gx = sum_gx / n
    mean_gy = sum_gy / n
    mean_gz = sum_gz / n

    amag = math.sqrt(mean_ax * mean_ax + mean_ay * mean_ay + mean_az * mean_az)
    if amag > 1e-6:
        ux, uy, uz = (mean_ax / amag, mean_ay / amag, mean_az / amag)
        target_ax, target_ay, target_az = (ux * G0, uy * G0, uz * G0)
        abx, aby, abz = (mean_ax - target_ax, mean_ay - target_ay, mean_az - target_az)
    else:
        abx, aby, abz = (0.0, 0.0, 0.0)

    return Biases(
        gyro_bias_rads=(mean_gx, mean_gy, mean_gz),
        accel_bias_mps2=(abx, aby, abz),
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="ICM-20948 I2C logger (quaternion + RPY + lin acc + gyro).")
    ap.add_argument("--rate", type=float, default=100.0, help="Target sample rate in Hz (default: 100).")
    ap.add_argument("--format", choices=["csv", "jsonl"], default="csv", help="Output format per sample.")
    ap.add_argument("--out", default="-", help="Output path (default: stdout). Use '-' for stdout.")
    ap.add_argument("--calibrate-seconds", type=float, default=2.0, help="Stationary calibration duration in seconds.")
    ap.add_argument("--kp", type=float, default=5.0, help="Mahony proportional gain (higher locks roll/pitch faster).")
    ap.add_argument("--ki", type=float, default=0.0, help="Mahony integral gain (0 disables integral).")
    ap.add_argument(
        "--gyro-units",
        choices=["rads", "dps"],
        default="rads",
        help="Units returned by the driver for gyro (default: rads).",
    )
    ap.add_argument(
        "--dp",
        type=int,
        default=None,
        help="Optional rounding/formatting: number of decimals for float fields (e.g. 4 -> 0.0000).",
    )
    ap.add_argument(
        "--use-mag",
        action="store_true",
        help="Use magnetometer in fusion (MARG) to reduce yaw drift. Mag is often noisy indoors.",
    )
    ap.add_argument(
        "--include-mag",
        action="store_true",
        help="Include magnetometer fields in output (adds mx_uT,my_uT,mz_uT columns/keys).",
    )
    ap.add_argument(
        "--mag-bias",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("MX", "MY", "MZ"),
        help="Optional magnetometer bias to subtract (same units as driver, typically uT).",
    )
    ap.add_argument(
        "--print-hz",
        type=float,
        default=None,
        help="Optional output throttle in Hz (e.g. 5 prints ~5 lines/sec). Overrides --print-every.",
    )
    ap.add_argument("--print-every", type=int, default=1, help="Write every Nth sample (default: 1).")
    args = ap.parse_args()

    # Import ICM driver lazily so the script still prints a useful error if missing.
    try:
        import board  # type: ignore
        import adafruit_icm20x  # type: ignore
    except Exception as e:
        print(
            "Missing dependency for I2C access. Install with:\n"
            "  pip3 install adafruit-blinka adafruit-circuitpython-icm20x\n"
            f"Import error: {e}",
            file=sys.stderr,
        )
        return 2

    i2c = board.I2C()
    imu = adafruit_icm20x.ICM20948(i2c)

    # Adafruit driver commonly returns:
    # - imu.acceleration in m/s^2
    # - imu.gyro in rad/s
    # Some platforms/drivers may expose deg/s; use --gyro-units=dps if needed.
    def read_sample() -> Tuple[float, float, float, float, float, float]:
        ax, ay, az = imu.acceleration
        gx, gy, gz = imu.gyro
        if args.gyro_units == "dps":
            gx = math.radians(gx)
            gy = math.radians(gy)
            gz = math.radians(gz)
        return float(ax), float(ay), float(az), float(gx), float(gy), float(gz)

    def read_mag() -> Tuple[float, float, float]:
        try:
            mx, my, mz = imu.magnetic
        except Exception as e:
            raise RuntimeError(f"Magnetometer read failed (does this driver expose imu.magnetic?): {e}") from e
        mx -= args.mag_bias[0]
        my -= args.mag_bias[1]
        mz -= args.mag_bias[2]
        return float(mx), float(my), float(mz)

    if args.calibrate_seconds > 0.0:
        print(
            f"Calibrating for {args.calibrate_seconds:.2f}s @ {args.rate:.1f}Hz - keep IMU still...",
            file=sys.stderr,
            flush=True,
        )
    biases = _calibrate_stationary(read_sample, seconds=args.calibrate_seconds, rate_hz=args.rate)
    print(
        f"gyro_bias_rads={biases.gyro_bias_rads} accel_bias_mps2={biases.accel_bias_mps2}",
        file=sys.stderr,
        flush=True,
    )

    filt = MahonyIMU(kp=args.kp, ki=args.ki)

    out = _open_out(args.out)
    close_out = out is not sys.stdout
    # When stdout is piped (not a TTY), Python may buffer heavily; flush so the user
    # can see live values in pipelines like `... | cut ...`.
    flush_each_line = (out is sys.stdout) and (not sys.stdout.isatty())
    try:
        if args.print_hz is not None and args.print_hz > 0.0:
            # Choose an integer decimation factor based on requested sample rate.
            # Example: rate=100, print_hz=5 -> print_every=20.
            args.print_every = max(1, int(round(args.rate / args.print_hz)))

        if args.format == "csv":
            out.write(
                "t_ns,t_s,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,lax_mps2,lay_mps2,laz_mps2,gx_rads,gy_rads,gz_rads"
                + (",mx_uT,my_uT,mz_uT" if args.include_mag else "")
                + "\n"
            )
            if flush_each_line:
                out.flush()

        def fmt_f(x: float, dp: Optional[int]) -> str:
            if dp is None:
                # Keep previous per-field formatting defaults (set below).
                return str(x)
            return f"{x:.{dp}f}"

        period = 1.0 / args.rate if args.rate > 0.0 else 0.01
        last_ns: Optional[int] = None
        i = 0
        t_next = time.monotonic()

        while True:
            now_ns = time.monotonic_ns()
            if last_ns is None:
                dt = period
            else:
                dt = (now_ns - last_ns) * 1e-9
                if dt <= 0.0:
                    dt = period
            last_ns = now_ns

            ax, ay, az, gx, gy, gz = read_sample()
            mx = my = mz = 0.0
            if args.use_mag or args.include_mag:
                mx, my, mz = read_mag()

            # Bias correct (gyro bias in rad/s, accel bias in m/s^2)
            gx -= biases.gyro_bias_rads[0]
            gy -= biases.gyro_bias_rads[1]
            gz -= biases.gyro_bias_rads[2]
            ax -= biases.accel_bias_mps2[0]
            ay -= biases.accel_bias_mps2[1]
            az -= biases.accel_bias_mps2[2]

            if args.use_mag:
                qw, qx, qy, qz = filt.update_marg(gx=gx, gy=gy, gz=gz, ax=ax, ay=ay, az=az, mx=mx, my=my, mz=mz, dt=dt)
            else:
                qw, qx, qy, qz = filt.update(gx=gx, gy=gy, gz=gz, ax=ax, ay=ay, az=az, dt=dt)
            roll_deg, pitch_deg, yaw_deg = _rpy_from_quat_deg(qw, qx, qy, qz)

            # Gravity removal to get linear acceleration in body frame
            gdx, gdy, gdz = gravity_body_from_quat(qw, qx, qy, qz)
            gxb, gyb, gzb = (gdx * G0, gdy * G0, gdz * G0)
            lax = ax - gxb
            lay = ay - gyb
            laz = az - gzb

            i += 1
            if args.print_every > 1 and (i % args.print_every) != 0:
                # still keep timing stable
                t_next += period
                delay = t_next - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                continue

            t_s = now_ns * 1e-9

            if args.format == "csv":
                if args.dp is None:
                    # Legacy formatting (more precision than necessary, but stable & readable)
                    qw_s, qx_s, qy_s, qz_s = (f"{qw:.8f}", f"{qx:.8f}", f"{qy:.8f}", f"{qz:.8f}")
                    roll_s, pitch_s, yaw_s = (f"{roll_deg:.4f}", f"{pitch_deg:.4f}", f"{yaw_deg:.4f}")
                    lax_s, lay_s, laz_s = (f"{lax:.6f}", f"{lay:.6f}", f"{laz:.6f}")
                    gx_s, gy_s, gz_s = (f"{gx:.6f}", f"{gy:.6f}", f"{gz:.6f}")
                else:
                    qw_s, qx_s, qy_s, qz_s = (fmt_f(qw, args.dp), fmt_f(qx, args.dp), fmt_f(qy, args.dp), fmt_f(qz, args.dp))
                    roll_s, pitch_s, yaw_s = (fmt_f(roll_deg, args.dp), fmt_f(pitch_deg, args.dp), fmt_f(yaw_deg, args.dp))
                    lax_s, lay_s, laz_s = (fmt_f(lax, args.dp), fmt_f(lay, args.dp), fmt_f(laz, args.dp))
                    gx_s, gy_s, gz_s = (fmt_f(gx, args.dp), fmt_f(gy, args.dp), fmt_f(gz, args.dp))
                out.write(
                    f"{now_ns},{t_s:.9f},"
                    f"{qw_s},{qx_s},{qy_s},{qz_s},"
                    f"{roll_s},{pitch_s},{yaw_s},"
                    f"{lax_s},{lay_s},{laz_s},"
                    f"{gx_s},{gy_s},{gz_s}"
                    + (f",{fmt_f(mx, args.dp)},{fmt_f(my, args.dp)},{fmt_f(mz, args.dp)}" if args.include_mag else "")
                    + "\n"
                )
                if flush_each_line:
                    out.flush()
            else:
                # Keep JSON keys aligned with CSV columns for easy parsing/diffing.
                dp = args.dp
                def r(x: float) -> float:
                    return round(x, dp) if dp is not None else x
                obj: Dict[str, object] = {
                    "t_ns": now_ns,
                    "t_s": t_s,
                    "qw": r(qw),
                    "qx": r(qx),
                    "qy": r(qy),
                    "qz": r(qz),
                    "roll_deg": r(roll_deg),
                    "pitch_deg": r(pitch_deg),
                    "yaw_deg": r(yaw_deg),
                    "lax_mps2": r(lax),
                    "lay_mps2": r(lay),
                    "laz_mps2": r(laz),
                    "gx_rads": r(gx),
                    "gy_rads": r(gy),
                    "gz_rads": r(gz),
                }
                if args.include_mag:
                    obj["mx_uT"] = r(mx)
                    obj["my_uT"] = r(my)
                    obj["mz_uT"] = r(mz)
                out.write(json.dumps(obj, separators=(",", ":")) + "\n")
                if flush_each_line:
                    out.flush()

            # Rate control
            t_next += period
            delay = t_next - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    except KeyboardInterrupt:
        return 0
    finally:
        if close_out:
            out.close()


if __name__ == "__main__":
    raise SystemExit(main())


