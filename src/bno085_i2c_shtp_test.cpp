#include "i2c_sh2_hal.h"

extern "C" {
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
}

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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
  if (sh2_decodeSensorEvent(&v, pEvent) != SH2_OK) return;

  const uint64_t host_now = mono_ns();
  const uint64_t sensor_us = v.timestamp;
  const uint64_t host_from_sensor = g_time_map.sensor_to_host_ns(sensor_us, host_now);

  if (v.sensorId == SH2_ACCELEROMETER) {
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
               "Usage: %s [--dev /dev/i2c-3] [--addr 0x4B] [--hz 250]\n",
               argv0);
}

int main(int argc, char** argv) {
  I2cSh2HalConfig cfg{};
  int hz = 250;

  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    if (a == "--dev" && i + 1 < argc) {
      cfg.dev = argv[++i];
    } else if (a == "--addr" && i + 1 < argc) {
      cfg.addr = std::strtol(argv[++i], nullptr, 0);
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

  I2cSh2Hal i2c{};
  init_i2c_sh2_hal(i2c, cfg);

  std::fprintf(stderr, "Opening BNO08x I2C-SHTP on %s addr=0x%02X\n", cfg.dev.c_str(), cfg.addr);
  const int rc = sh2_open(&i2c.hal, on_event, nullptr);
  if (rc != SH2_OK) {
    std::fprintf(stderr, "sh2_open failed: %d\n", rc);
    return 1;
  }

  sh2_ProductIds_t prod{};
  int prod_rc = sh2_getProdIds(&prod);
  if (prod_rc != SH2_OK) {
    std::fprintf(stderr, "sh2_getProdIds failed: %d\n", prod_rc);
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



