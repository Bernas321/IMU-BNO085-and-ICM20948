#pragma once

#include <cstdint>
#include <string>

extern "C" {
#include "sh2_hal.h"
}

struct I2cSh2HalConfig {
  std::string dev = "/dev/i2c-1";
  int addr = 0x4B;          // common BNO08x address; scan with i2cdetect if unsure
  int max_xfer = 128;       // bytes per I2C transaction
};

struct I2cSh2Hal {
  sh2_Hal_t hal{};
  I2cSh2HalConfig cfg{};
  int fd = -1;
};

void init_i2c_sh2_hal(I2cSh2Hal& i2c, const I2cSh2HalConfig& cfg);



