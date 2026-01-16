#pragma once

#include <string>

// Forward declare (C struct)
extern "C" {
#include "sh2_hal.h"
}

struct UartSh2HalConfig {
  std::string port = "/dev/serial0";
  int baud = 3000000;
};

// Concrete SH2 HAL instance with internal state.
struct UartSh2Hal {
  sh2_Hal_t hal{};
  UartSh2HalConfig cfg{};

  int fd = -1;
  // Rx accumulation buffer for stream framing.
  unsigned pkt_start_us = 0;
  bool pkt_in_progress = false;
  std::string rx_buf{};
};

// Initialize function pointers in `uart.hal` and copy config.
void init_uart_sh2_hal(UartSh2Hal& uart, const UartSh2HalConfig& cfg);


