#include "i2c_sh2_hal.h"

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <linux/i2c-dev.h>

namespace {

uint64_t monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

uint32_t monotonic_us32() {
  return static_cast<uint32_t>(monotonic_ns() / 1000ULL);
}

I2cSh2Hal* container(sh2_Hal_t* self) {
  return reinterpret_cast<I2cSh2Hal*>(reinterpret_cast<char*>(self) - offsetof(I2cSh2Hal, hal));
}

int i2c_set_addr(int fd, int addr) {
  if (ioctl(fd, I2C_SLAVE, addr) < 0) return -1;
  return 0;
}

bool i2c_write_all(int fd, const uint8_t* buf, size_t len) {
  while (len > 0) {
    ssize_t n = write(fd, buf, len);
    if (n > 0) {
      buf += static_cast<size_t>(n);
      len -= static_cast<size_t>(n);
      continue;
    }
    return false;
  }
  return true;
}

bool i2c_read_all(int fd, uint8_t* buf, size_t len) {
  while (len > 0) {
    ssize_t n = read(fd, buf, len);
    if (n > 0) {
      buf += static_cast<size_t>(n);
      len -= static_cast<size_t>(n);
      continue;
    }
    return false;
  }
  return true;
}

int hal_open(sh2_Hal_t* self) {
  auto* st = container(self);
  st->fd = open(st->cfg.dev.c_str(), O_RDWR);
  if (st->fd < 0) return -1;
  if (i2c_set_addr(st->fd, st->cfg.addr) != 0) return -1;

  // SparkFun's I2C HAL sends a 5-byte "soft reset" packet during open:
  //   {5, 0, 1, 0, 1}
  const uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  bool ok = false;
  for (int attempts = 0; attempts < 5; attempts++) {
    if (i2c_write_all(st->fd, softreset_pkt, sizeof(softreset_pkt))) {
      ok = true;
      break;
    }
    usleep(30 * 1000);
  }
  if (!ok) return -1;
  usleep(300 * 1000);
  return 0;
}

void hal_close(sh2_Hal_t* self) {
  auto* st = container(self);
  if (st->fd >= 0) {
    close(st->fd);
    st->fd = -1;
  }
}

int hal_read(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len, uint32_t* t_us) {
  auto* st = container(self);
  if (st->fd < 0) return 0;

  uint8_t header[4];
  if (!i2c_read_all(st->fd, header, sizeof(header))) return 0;

  uint16_t packet_size = static_cast<uint16_t>(header[0]) | (static_cast<uint16_t>(header[1]) << 8);
  packet_size &= static_cast<uint16_t>(~0x8000);  // clear continuation bit

  if (packet_size == 0 || packet_size > len) {
    return 0;
  }

  // SparkFun copies from subsequent reads skipping the repeated 4-byte header.
  uint16_t cargo_remaining = packet_size;
  bool first_read = true;

  while (cargo_remaining > 0) {
    const unsigned max_xfer = static_cast<unsigned>(st->cfg.max_xfer);
    unsigned read_size = 0;
    if (first_read) {
      read_size = (cargo_remaining < max_xfer) ? cargo_remaining : max_xfer;
    } else {
      const unsigned want = static_cast<unsigned>(cargo_remaining) + 4U;
      read_size = (want < max_xfer) ? want : max_xfer;
    }

    uint8_t tmp[256];
    if (read_size > sizeof(tmp)) return 0;
    if (!i2c_read_all(st->fd, tmp, read_size)) return 0;

    unsigned cargo_read = 0;
    if (first_read) {
      cargo_read = read_size;
      std::memcpy(pBuffer, tmp, cargo_read);
      first_read = false;
    } else {
      if (read_size < 4) return 0;
      cargo_read = read_size - 4U;
      std::memcpy(pBuffer, tmp + 4, cargo_read);
    }

    pBuffer += cargo_read;
    cargo_remaining -= static_cast<uint16_t>(cargo_read);
  }

  if (t_us) *t_us = monotonic_us32();
  return static_cast<int>(packet_size);
}

int hal_write(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len) {
  auto* st = container(self);
  if (st->fd < 0) return 0;
  unsigned write_size = len;
  if (write_size > static_cast<unsigned>(st->cfg.max_xfer)) {
    write_size = static_cast<unsigned>(st->cfg.max_xfer);
  }
  if (!i2c_write_all(st->fd, pBuffer, write_size)) return 0;
  return static_cast<int>(write_size);
}

uint32_t hal_get_time_us(sh2_Hal_t* /*self*/) {
  return monotonic_us32();
}

}  // namespace

void init_i2c_sh2_hal(I2cSh2Hal& i2c, const I2cSh2HalConfig& cfg) {
  i2c.cfg = cfg;
  i2c.hal.open = hal_open;
  i2c.hal.close = hal_close;
  i2c.hal.read = hal_read;
  i2c.hal.write = hal_write;
  i2c.hal.getTimeUs = hal_get_time_us;
}



