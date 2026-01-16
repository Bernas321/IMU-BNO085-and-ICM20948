#!/usr/bin/env python3
"""
Quick IMU log sanity metrics for a JSONL log produced by icm20948_log.py.

Focus: what you normally check before moving on to a visual/VIO test:
- Mean + std-dev of gyro (rad/s) during a stationary window
- Mean + std-dev of linear accel (m/s^2) during a stationary window
- Optional mean norm checks
"""

from __future__ import annotations

import argparse
import json
import math
from typing import Iterable, Tuple


def _mean_std(xs: Iterable[float]) -> Tuple[float, float]:
    xs = list(xs)
    if not xs:
        return 0.0, 0.0
    m = sum(xs) / len(xs)
    v = sum((x - m) ** 2 for x in xs) / max(1, (len(xs) - 1))
    return m, math.sqrt(v)


def main() -> int:
    ap = argparse.ArgumentParser(description="Compute mean/std-dev on a JSONL IMU log (stationary window).")
    ap.add_argument("jsonl", help="Path to JSONL file")
    ap.add_argument("--start-s", type=float, default=None, help="Window start time (t_s). Default: start of file.")
    ap.add_argument("--duration-s", type=float, default=10.0, help="Window duration seconds (default: 10s).")
    args = ap.parse_args()

    t0 = None
    t_start = None
    t_end = None

    gx = []
    gy = []
    gz = []
    ax = []
    ay = []
    az = []
    gnorm = []
    anorm = []

    with open(args.jsonl, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            ts = float(obj["t_s"])
            if t0 is None:
                t0 = ts
                t_start = (args.start_s if args.start_s is not None else ts)
                t_end = t_start + args.duration_s
            if ts < t_start:
                continue
            if ts > t_end:
                break

            gxv = float(obj["gx_rads"])
            gyv = float(obj["gy_rads"])
            gzv = float(obj["gz_rads"])
            axv = float(obj["lax_mps2"])
            ayv = float(obj["lay_mps2"])
            azv = float(obj["laz_mps2"])

            gx.append(gxv); gy.append(gyv); gz.append(gzv)
            ax.append(axv); ay.append(ayv); az.append(azv)
            gnorm.append(math.sqrt(gxv * gxv + gyv * gyv + gzv * gzv))
            anorm.append(math.sqrt(axv * axv + ayv * ayv + azv * azv))

    if not gx:
        print("No samples in window.")
        return 1

    mx, sx = _mean_std(gx)
    my, sy = _mean_std(gy)
    mz, sz = _mean_std(gz)
    max_, sax = _mean_std(ax)
    may_, say = _mean_std(ay)
    maz_, saz = _mean_std(az)
    mgn, sgn = _mean_std(gnorm)
    man, san = _mean_std(anorm)

    print(f"samples={len(gx)} window=[{t_start:.3f},{t_end:.3f}]s")
    print(f"gyro_mean_rads=({mx:.6f},{my:.6f},{mz:.6f}) gyro_std_rads=({sx:.6f},{sy:.6f},{sz:.6f})")
    print(f"linacc_mean_mps2=({max_:.6f},{may_:.6f},{maz_:.6f}) linacc_std_mps2=({sax:.6f},{say:.6f},{saz:.6f})")
    print(f"|gyro| mean/std = ({mgn:.6f},{sgn:.6f}) rad/s")
    print(f"|linacc| mean/std = ({man:.6f},{san:.6f}) m/s^2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


