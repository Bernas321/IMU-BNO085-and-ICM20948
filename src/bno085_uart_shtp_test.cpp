#include "uart_sh2_hal.h"

extern "C" {
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
}

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <string>
#include <thread>

#include <time.h>

static uint64_t mono_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

struct TimeMapper {
  bool have_offset = false;
  int64_t offset_ns = 0;  // host_ns - sensor_ns

  uint64_t sensor_to_host_ns(uint64_t sensor_us, uint64_t host_now_ns) {
    const int64_t sensor_ns = static_cast<int64_t>(sensor_us) * 1000LL;
    if (!have_offset) {
      offset_ns = static_cast<int64_t>(host_now_ns) - sensor_ns;
      have_offset = true;
    }
    return static_cast<uint64_t>(sensor_ns + offset_ns);
  }
};

static TimeMapper g_time_map;

static void on_event(void* /*cookie*/, sh2_AsyncEvent_t* pEvent) {
  if (!pEvent) return;
  if (pEvent->eventId == SH2_RESET) {
    std::fprintf(stderr, "[sh2] RESET event\n");
  }
}

static void on_sensor(void* /*cookie*/, sh2_SensorEvent_t* pEvent) {
  if (!pEvent) return;
  sh2_SensorValue_t v{};
  if (sh2_decodeSensorEvent(&v, pEvent) != SH2_OK) {
    return;
  }

  const uint64_t host_now = mono_ns();
  const uint64_t sensor_us = v.timestamp;
  const uint64_t host_from_sensor = g_time_map.sensor_to_host_ns(sensor_us, host_now);

  if (v.sensorId == SH2_ACCELEROMETER) {
    // Calibrated accel is in m/s^2.
    std::printf(
        "ACC host_now_ns=%llu host_from_sensor_ns=%llu sensor_us=%llu ax=%.6f ay=%.6f az=%.6f\n",
        (unsigned long long)host_now,
        (unsigned long long)host_from_sensor,
        (unsigned long long)sensor_us,
        (double)v.un.accelerometer.x,
        (double)v.un.accelerometer.y,
        (double)v.un.accelerometer.z);
    std::fflush(stdout);
  } else if (v.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    // Calibrated gyro is in rad/s.
    std::printf(
        "GYR host_now_ns=%llu host_from_sensor_ns=%llu sensor_us=%llu gx=%.6f gy=%.6f gz=%.6f\n",
        (unsigned long long)host_now,
        (unsigned long long)host_from_sensor,
        (unsigned long long)sensor_us,
        (double)v.un.gyroscope.x,
        (double)v.un.gyroscope.y,
        (double)v.un.gyroscope.z);
    std::fflush(stdout);
  }
}

static void usage(const char* argv0) {
  std::fprintf(stderr,
               "Usage: %s [--port /dev/serial0] [--baud 3000000] [--hz 250]\n"
               "\n"
               "Wiring for UART-SHTP (BNO085):\n"
               "- Set protocol pins for UART-SHTP: PS1=3V3, PS0/WAKE=GND (Adafruit boards: P1=3V3, P0=GND)\n"
               "- BNO SDA (TX out) -> Pi RX (GPIO15, pin 10)\n"
               "- BNO SCL (RX in)  -> Pi TX (GPIO14, pin 8)\n"
               "- 3V3 + GND\n",
               argv0);
}

int main(int argc, char** argv) {
  UartSh2HalConfig cfg{};
  int hz = 250;

  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    if (a == "--port" && i + 1 < argc) {
      cfg.port = argv[++i];
    } else if (a == "--baud" && i + 1 < argc) {
      cfg.baud = std::atoi(argv[++i]);
    } else if (a == "--hz" && i + 1 < argc) {
      hz = std::atoi(argv[++i]);
    } else if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "Unknown arg: %s\n", a.c_str());
      usage(argv[0]);
      return 2;
    }
  }

  UartSh2Hal uart{};
  init_uart_sh2_hal(uart, cfg);

  std::fprintf(stderr, "Opening BNO08x UART-SHTP on %s @ %d baud\n", cfg.port.c_str(), cfg.baud);

  const int rc = sh2_open(&uart.hal, on_event, nullptr);
  if (rc != SH2_OK) {
    std::fprintf(stderr, "sh2_open failed: %d\n", rc);
    return 1;
  }

  // Verify link by reading Product IDs (same idea as SparkFun's init).
  sh2_ProductIds_t prod{};
  int prod_rc = SH2_ERR_BAD_PARAM;
  // On some setups the advertisement phase can take a moment; retry for up to ~2s.
  const uint64_t start_ns = mono_ns();
  while ((mono_ns() - start_ns) < 2000000000ULL) {
    prod_rc = sh2_getProdIds(&prod);
    if (prod_rc == SH2_OK) break;
    sh2_service();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (prod_rc != SH2_OK) {
    std::fprintf(stderr,
                 "sh2_getProdIds failed: %d\n"
                 "- If you still see AA AA frames on 115200, you're still in UART-RVC.\n"
                 "- If you see no bytes at 3,000,000, check UART-SHTP wiring (try swapping SDA/SCL) and power-cycle.\n",
                 prod_rc);
    return 2;
  }
  std::fprintf(stderr,
               "ProdId: resetCause=%u sw=%u.%u.%u part=%u build=%u\n",
               prod.entry[0].resetCause,
               prod.entry[0].swVersionMajor,
               prod.entry[0].swVersionMinor,
               prod.entry[0].swVersionPatch,
               prod.entry[0].swPartNumber,
               prod.entry[0].swBuildNumber);

  if (sh2_setSensorCallback(on_sensor, nullptr) != SH2_OK) {
    std::fprintf(stderr, "sh2_setSensorCallback failed\n");
    return 1;
  }

  // Enable raw gyro + raw accel at requested rate.
  sh2_SensorConfig_t c{};
  c.changeSensitivityEnabled = false;
  c.changeSensitivityRelative = false;
  c.wakeupEnabled = false;
  c.alwaysOnEnabled = true;
  c.changeSensitivity = 0;
  c.reportInterval_us = (hz > 0) ? static_cast<uint32_t>(1000000 / hz) : 4000;
  c.batchInterval_us = 0;
  c.sensorSpecific = 0;

  if (sh2_setSensorConfig(SH2_ACCELEROMETER, &c) != SH2_OK) {
    std::fprintf(stderr, "Failed to enable ACCELEROMETER\n");
  }
  if (sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &c) != SH2_OK) {
    std::fprintf(stderr, "Failed to enable GYROSCOPE_CALIBRATED\n");
  }

  std::fprintf(stderr, "Streaming accel+gyro at ~%d Hz (Ctrl+C to stop)\n", hz);

  while (true) {
    sh2_service();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}


