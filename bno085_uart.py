import time
import serial
from adafruit_bno08x_rvc import BNO08x_RVC
from adafruit_bno08x_rvc import RVCReadTimeoutError

# Blinka's `busio.UART` is not supported on this Debian/Raspberry Pi setup.
# Use pyserial directly.
#
# /dev/serial0 is the recommended stable alias on Raspberry Pi; it maps to the
# correct UART device (e.g. /dev/ttyS0 or /dev/ttyAMA0) based on config.
#
# Important: the underlying library indexes `data[0]` / `data[1]` after
# `read(2)`, so we must never return fewer than 2 bytes.
#
# We set a small serial timeout and pad short reads with zeros; this keeps the
# library's own timeout logic working (instead of blocking forever).
PORT = "/dev/serial0"
# Your sniffer output showed valid RVC frames at 115200 and none at 3000000.
BAUDRATE = 115200
# Small timeout so reads don't block forever; we accumulate to exact lengths below.
SERIAL_TIMEOUT_S = 0.1


class _ExactSerial:
    def __init__(self, ser: serial.Serial):
        self._ser = ser

    def reset_input_buffer(self) -> None:
        self._ser.reset_input_buffer()

    def read(self, nbytes: int) -> bytes:
        # PySerial can return short reads when a timeout is set; the upstream
        # library expects *exactly* nbytes (it indexes data[0]/data[1] and reads
        # fixed-length frames). Accumulate until we have the requested length.
        data = b""
        # Should fill quickly when the stream is correct; keep this comfortably
        # below the library's own 1s read timeout.
        deadline = time.monotonic() + 0.2
        while len(data) < nbytes and time.monotonic() < deadline:
            chunk = self._ser.read(nbytes - len(data))
            if chunk:
                data += chunk
        # If we still couldn't get enough bytes, pad the remainder. This will
        # fail checksum and allow the library's higher-level timeout to fire,
        # but avoids IndexError.
        if len(data) < nbytes:
            data += b"\x00" * (nbytes - len(data))
        return data


uart = _ExactSerial(serial.Serial(PORT, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT_S))
rvc = BNO08x_RVC(uart)
print(f"Listening for BNO08x RVC on {PORT} @ {BAUDRATE} baud...", flush=True)
while True:
    try:
        roll, pitch, yaw, x_accel, y_accel, z_accel = rvc.heading
        print("Roll: %2.2f Pitch: %2.2f Yaw: %2.2f Degrees" % (roll, pitch, yaw), flush=True)
        print(
            "Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel),
            flush=True,
        )
        print("", flush=True)
    except RVCReadTimeoutError:
        print(
            f"No RVC data yet on {PORT} @ {BAUDRATE} baud (check wiring/UART/baudrate).",
            flush=True,
        )
    time.sleep(0.1)