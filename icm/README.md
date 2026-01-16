## ICM-20948 (I2C) logger

This folder contains a simple Raspberry Pi friendly logger that streams:
- **Orientation**: quaternion + roll/pitch/yaw
- **Linear acceleration**: accelerometer with gravity removed (body frame)
- **Gyro**: bias-corrected angular velocity
- **Timestamp**: `t_ns` from `time.monotonic_ns()`

Each sample is one line of **CSV** or **JSONL** with a fixed schema.

### Wiring (Raspberry Pi)
- **VCC** → 3V3
- **GND** → GND
- **SDA** → SDA (GPIO2, pin 3)
- **SCL** → SCL (GPIO3, pin 5)

Enable I2C (Raspberry Pi): `sudo raspi-config` → Interface Options → I2C.

Optional sanity check (should show `0x68` or `0x69`):

```bash
sudo i2cdetect -y 1
```

### Install deps

```bash
pip3 install adafruit-blinka adafruit-circuitpython-icm20x
```

### Run

CSV to stdout (default):

```bash
python3 icm/icm20948_log.py --rate 100 --format csv --out -
```

If you want to **watch values change** while moving the sensor, slow the print rate:

```bash
# ~5 lines/sec output, but still timestamps each printed sample
python3 icm/icm20948_log.py --rate 100 --print-hz 5 --dp 4 --format csv --out -
```

If you want to watch only a few columns (time + RPY + gyro):

```bash
python3 -u icm/icm20948_log.py --rate 100 --print-hz 5 --dp 4 --format csv --out - \
| cut -d, -f2,7-9,13-15
```

JSONL to a file:

```bash
python3 icm/icm20948_log.py --rate 100 --format jsonl --out icm20948.jsonl
```

Round float fields (both CSV and JSONL) to 4 decimals:

```bash
python3 icm/icm20948_log.py --dp 4 --rate 100 --format jsonl --out icm20948.jsonl
```

If your platform’s driver reports gyro in **deg/s**, force conversion to rad/s:

```bash
python3 icm/icm20948_log.py --gyro-units dps --rate 100 --format jsonl --out icm20948.jsonl
```

### Optional magnetometer (recommended to keep optional for SLAM)

You can optionally:
- **use magnetometer in fusion** to reduce yaw drift: `--use-mag`
- **log magnetometer values**: `--include-mag`

Example (MARG fusion + log mag):

```bash
python3 icm/icm20948_log.py --use-mag --include-mag --rate 100 --format jsonl --out icm20948.jsonl
```

If you have a known hard-iron bias (in the same units as the driver, typically µT), subtract it:

```bash
python3 icm/icm20948_log.py --use-mag --mag-bias 12.3 -4.5 8.9 --rate 100 --format jsonl --out icm20948.jsonl
```

### Output schema

- **CSV columns**:
  `t_ns,t_s,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,lax_mps2,lay_mps2,laz_mps2,gx_rads,gy_rads,gz_rads`
- **Units**:
  - `t_ns`: nanoseconds (monotonic)
  - `t_s`: seconds (monotonic)
  - `q*`: unit quaternion (w,x,y,z)
  - `roll/pitch/yaw`: degrees
  - `a*`: m/s² (linear accel, gravity removed, body frame)
  - `g*`: rad/s (gyro, bias-corrected)

### Visual test (3D cube in browser)

Install deps:

```bash
pip3 install -r icm/requirements.txt
```

Start the visualizer (live sensor → WebSocket + HTTP):

```bash
python3 icm/viz/viz_server.py --use-mag --include-mag
```

Then open `http://localhost:8000` and you should see a cube follow the IMU orientation.

Replay a saved JSONL file:

```bash
python3 icm/viz/viz_server.py --file icm20948.jsonl
```

### Quick measurement sanity checks (mean/std-dev)

For a stationary window (default first 10s):

```bash
python3 icm/analyze_log.py icm20948.jsonl --duration-s 10
```

### Notes
- This is **6-axis fusion** (accel+gyro). Yaw will drift over time without magnetometer fusion.
- Calibration is a simple **stationary bias estimate** for gyro, and an accel bias that forces the mean acceleration magnitude to \(g\) while still.


