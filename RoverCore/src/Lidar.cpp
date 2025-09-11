#ifdef __linux__

#include "Lidar.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <poll.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <utility>

using namespace std;

inline uint64_t mono_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

LidarReader::LidarReader(std::string port, int baud, int max_range_mm)
  : port_(std::move(port)), baud_(baud), max_range_mm_(max_range_mm) {}

LidarReader::~LidarReader() { close(); }

unsigned long baud_to_term_(int baud) {
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    default:     return B230400;
  }
}

bool LidarReader::configure_port_() {
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) { perror("tcgetattr"); return false; }

  cfmakeraw(&tio);
  cfsetspeed(&tio, baud_to_term_(baud_));

  // 8N1, local, receiver on
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;

  // No HW/SW flow control (some USB-UARTs misreport CTS/RTS)
  tio.c_cflag &= ~CRTSCTS;
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);

  // Nonblocking; we use poll()
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) { perror("tcsetattr"); return false; }

  tcflush(fd_, TCIFLUSH);
  return true;
}

bool LidarReader::open() {
  close();
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) { perror(("open " + port_).c_str()); return false; }
  if (!configure_port_()) { close(); return false; }
  return true;
}

void LidarReader::close() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// LD06: 47 bytes/frame; last byte = sum of first 46
inline bool ld06_checksum_ok(const uint8_t* f) {
  uint8_t s = 0;
  for (int i = 0; i < 46; ++i) s += f[i];
  return s == f[46];
}

bool LidarReader::pump(const Callback& on_point, int poll_timeout_ms) {
  if (fd_ < 0) return false;

  // 1) Read any available bytes (nonblocking)
  pollfd pfd{fd_, POLLIN, 0};
  int pr = ::poll(&pfd, 1, poll_timeout_ms);
  if (pr > 0 && (pfd.revents & POLLIN)) {
    for (;;) {
      uint8_t tmp[512];
      ssize_t r = ::read(fd_, tmp, sizeof(tmp));
      if (r > 0) {
        buf_.insert(buf_.end(), tmp, tmp + r);
        if (r < (ssize_t)sizeof(tmp)) break; // drained for now
      } else {
        break; // EAGAIN or error
      }
    }
  }

  // 2) Parse complete frames
  static constexpr size_t FRAME = 47;
  bool any = false;

  for (;;) {
    // Find header 0x54 0x2C
    size_t i = 0;
    bool found = false;
    for (; i + 2 <= buf_.size(); ++i) {
      if (buf_[i] == 0x54 && buf_[i + 1] == 0x2C) { found = true; break; }
    }
    if (!found) {
      // keep tail small, avoid unbounded growth
      if (buf_.size() > 4096) buf_.erase(buf_.begin(), buf_.end() - 1024);
      break;
    }

    // Not enough bytes for a full frame yet → keep header, wait for more
    if (i + FRAME > buf_.size()) {
      if (i > 0) buf_.erase(buf_.begin(), buf_.begin() + i);
      break;
    }

    // Copy frame BEFORE erasing (avoid dangling pointer)
    std::array<uint8_t, FRAME> fr{};
    std::copy(buf_.begin() + i, buf_.begin() + i + FRAME, fr.begin());
    buf_.erase(buf_.begin(), buf_.begin() + i + FRAME);

    // Validate checksum
    if (!ld06_checksum_ok(fr.data())) {
      // bad frame, continue searching
      continue;
    }

    // Parse angles/distances
    int sa_cent = int(fr[4])  | (int(fr[5])  << 8); // start angle *100
    int ea_cent = int(fr[42]) | (int(fr[43]) << 8); // end angle *100
    int span    = ea_cent - sa_cent; if (span < 0) span += 36000;

    constexpr int N = 12; // 12 points in payload
    const int denom = (N > 1) ? (N - 1) : 1;
    const uint64_t t = mono_ns();

    // 12 × (dist_lo, dist_hi, intensity) at bytes [6..41]
    for (int k = 0; k < N; ++k) {
      const uint8_t dL = fr[6 + k*3 + 0];
      const uint8_t dH = fr[6 + k*3 + 1];
      const uint8_t I  = fr[6 + k*3 + 2];
      uint32_t dmm = (uint32_t(dH) << 8) | uint32_t(dL);

      if (dmm == 0 || dmm > (uint32_t)max_range_mm_) continue;
      uint32_t a_cdeg = (uint32_t)(sa_cent + (span * k) / denom) % 36000;

      on_point(a_cdeg, dmm, (uint32_t)I, t);
      any = true;
    }
  }

  return any;
}

#endif // __linux__