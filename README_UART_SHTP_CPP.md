## UART-SHTP (SH-2) C++ test for BNO085 on Raspberry Pi

This repo now includes a minimal **C++ UART-SHTP** test binary that uses the open-source **SH-2/SHTP** host code (via SparkFun/CEVA) and a Linux `termios` UART HAL.

### What this gives you
- Streams **accelerometer (m/s²)** and **gyro (rad/s)** at a requested rate (e.g. 250Hz)
- Prints **timestamps**:
  - `host_now_ns` from `CLOCK_MONOTONIC`
  - `sensor_us` from SH-2 sensor timestamp
  - `host_from_sensor_ns` = sensor timestamp mapped into host monotonic time via a fixed offset (first sample)

### Wiring / mode
To use **UART-SHTP** (not RVC):
- Set protocol pins **PS1 = 3V3**, **PS0/WAKE = GND**
  - On Adafruit BNO085 breakout this typically means **P1 = 3V3** and **P0 = GND**
- UART pins on the Adafruit breakout:
  - **SDA** = sensor TX (data out) → Raspberry Pi **RXD0 GPIO15 (pin 10)**
  - **SCL** = sensor RX (data in)  ← Raspberry Pi **TXD0 GPIO14 (pin 8)**
- Power:
  - **VIN → 3V3**
  - **GND → GND**

### Build
From the repo root:

```bash
cmake -S . -B build
cmake --build build -j
```

### Run
Default expects `/dev/serial0` @ `3000000` baud and 250Hz:

```bash
./build/bno085_uart_shtp_test --port /dev/serial0 --baud 3000000 --hz 250
```

If your device uses a different port/baud, adjust `--port` / `--baud`.

### Output format (examples)

```
ACC host_now_ns=... host_from_sensor_ns=... sensor_us=... ax=... ay=... az=...
GYR host_now_ns=... host_from_sensor_ns=... sensor_us=... gx=... gy=... gz=...
```

### Notes
- If you still have the board strapped for **UART-RVC** (P0=3V3, P1=GND), this program will *not* work. Switch to **UART-SHTP** mode first.
- For the highest quality timing, wiring `H_INTN` to a GPIO and timestamping the edge is ideal, but this minimal test does not require it.


