#include "uart_sh2_hal.h"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <cstddef>
#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

// termios2 for arbitrary baud (BOTHER) without pulling in libc termios struct
// (which conflicts with the kernel header on some distros).
#include <asm/termbits.h>

namespace {

uint64_t monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

uint32_t monotonic_us32() {
  return static_cast<uint32_t>(monotonic_ns() / 1000ULL);
}

UartSh2Hal* container(sh2_Hal_t* self) {
  return reinterpret_cast<UartSh2Hal*>(reinterpret_cast<char*>(self) - offsetof(UartSh2Hal, hal));
}

int set_baud_termios2(int fd, int baud) {
  termios2 tio{};
  if (ioctl(fd, TCGETS2, &tio) != 0) {
    return -1;
  }

  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;

  tio.c_ispeed = static_cast<unsigned int>(baud);
  tio.c_ospeed = static_cast<unsigned int>(baud);

  // 8N1, no flow control, raw
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cflag |= (CLOCAL | CREAD);

  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;

  // Non-blocking reads with short timeout; we do stream framing ourselves.
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;  // 100ms

  if (ioctl(fd, TCSETS2, &tio) != 0) {
    return -1;
  }
  return 0;
}

// SHTP packet header is 4 bytes:
// - [0:2] length (little-endian), bit15 is continuation flag
// - [2] channel
// - [3] sequence
//
// We treat a "transfer" as one full SHTP packet.
bool try_extract_one_packet(UartSh2Hal& st, std::string& out_pkt) {
  if (st.rx_buf.size() < 4) return false;
  const uint8_t b0 = static_cast<uint8_t>(st.rx_buf[0]);
  const uint8_t b1 = static_cast<uint8_t>(st.rx_buf[1]);
  const uint16_t len_field = static_cast<uint16_t>(b0) | (static_cast<uint16_t>(b1) << 8);
  const uint16_t pkt_len = static_cast<uint16_t>(len_field & 0x7FFF);
  if (pkt_len < 4) {
    // Corrupt header; drop one byte and resync.
    st.rx_buf.erase(st.rx_buf.begin());
    st.pkt_in_progress = false;
    return false;
  }
  if (pkt_len > SH2_HAL_MAX_TRANSFER_IN) {
    // Too big for our buffers; drop one byte and resync.
    st.rx_buf.erase(st.rx_buf.begin());
    st.pkt_in_progress = false;
    return false;
  }
  if (st.rx_buf.size() < pkt_len) return false;

  out_pkt.assign(st.rx_buf.data(), pkt_len);
  st.rx_buf.erase(0, pkt_len);
  st.pkt_in_progress = false;
  return true;
}

int hal_open(sh2_Hal_t* self) {
  auto* st = container(self);
  st->fd = open(st->cfg.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (st->fd < 0) return -1;
  if (set_baud_termios2(st->fd, st->cfg.baud) != 0) return -1;

  // Bring the hub into a known state.
  //
  // SparkFun's HAL sends a 5-byte "soft reset" SHTP packet during open:
  //   {5, 0, 1, 0, 1}
  // We replicate that here for UART-SHTP to avoid starting mid-stream.
  const uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  for (int attempts = 0; attempts < 5; attempts++) {
    const ssize_t n = write(st->fd, softreset_pkt, sizeof(softreset_pkt));
    if (n == (ssize_t)sizeof(softreset_pkt)) break;
    usleep(30 * 1000);
  }
  usleep(300 * 1000);

  // Drop any stale bytes we might have buffered.
  st->rx_buf.clear();
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

  // Read whatever is available.
  uint8_t tmp[512];
  for (;;) {
    const ssize_t n = read(st->fd, tmp, sizeof(tmp));
    if (n > 0) {
      if (!st->pkt_in_progress) {
        st->pkt_start_us = monotonic_us32();
        st->pkt_in_progress = true;
      }
      st->rx_buf.append(reinterpret_cast<const char*>(tmp), static_cast<size_t>(n));
      continue;
    }
    if (n == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
    break;
  }

  std::string pkt;
  if (!try_extract_one_packet(*st, pkt)) return 0;
  if (pkt.size() > len) return 0;

  std::memcpy(pBuffer, pkt.data(), pkt.size());
  if (t_us) *t_us = st->pkt_start_us;
  return static_cast<int>(pkt.size());
}

int hal_write(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len) {
  auto* st = container(self);
  if (st->fd < 0) return 0;

  // Best-effort blocking write loop.
  size_t off = 0;
  while (off < len) {
    const ssize_t n = write(st->fd, pBuffer + off, len - off);
    if (n > 0) {
      off += static_cast<size_t>(n);
      continue;
    }
    if (n == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      usleep(1000);
      continue;
    }
    break;
  }
  return static_cast<int>(off);
}

uint32_t hal_get_time_us(sh2_Hal_t* self) {
  (void)self;
  return monotonic_us32();
}

}  // namespace

void init_uart_sh2_hal(UartSh2Hal& uart, const UartSh2HalConfig& cfg) {
  uart.cfg = cfg;
  uart.hal.open = hal_open;
  uart.hal.close = hal_close;
  uart.hal.read = hal_read;
  uart.hal.write = hal_write;
  uart.hal.getTimeUs = hal_get_time_us;
}


