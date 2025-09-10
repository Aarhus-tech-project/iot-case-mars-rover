// Lidar.cpp
#include "Lidar.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <array>

#ifdef _WIN32
// Windows stub uses simulator below
#else
#include <poll.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#endif

using namespace std;

// ===== LidarReader ==========================================================

LidarReader::LidarReader(std::string port, int baud, int max_range_mm)
  : port_(std::move(port)), baud_(baud), max_range_mm_(max_range_mm) {}

LidarReader::~LidarReader() { close(); }

uint64_t LidarReader::mono_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

unsigned long LidarReader::baud_to_term_(int baud) {
#ifdef _WIN32
  return 0;
#else
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:     return B230400;
  }
#endif
}

bool LidarReader::configure_port_() {
#ifdef _WIN32
  return false;
#else
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    perror("tcgetattr");
    return false;
  }

  // Raw 8N1
  cfmakeraw(&tio);
  cfsetspeed(&tio, baud_to_term_(baud_));

  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag &= ~CRTSCTS;    // no HW flow control

  // No SW flow control
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);

  // Nonblocking reads; we poll() outside
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    perror("tcsetattr");
    return false;
  }

  // Flush any garbage bytes
  tcflush(fd_, TCIFLUSH);
  return true;
#endif
}

bool LidarReader::open() {
#ifdef __linux__
  close();
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    perror(("open " + port_).c_str());
    return false;
  }
  if (!configure_port_()) {
    close();
    return false;
  }
  return true;
#else
  // non-Linux (macOS, Windows) -> OK; we’ll simulate frames in pump()
  return true;
#endif
}

void LidarReader::close() {
#ifdef __linux__
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#else
  // nothing to do for the simulator
#endif
}

// Helper: checksum (LD06 spec = sum of first 46 bytes equals last byte)
static inline bool ld06_checksum_ok(const uint8_t* f, size_t len) {
  if (len != 47) return false;
  uint8_t sum = 0;
  for (size_t i = 0; i < 46; ++i) sum += f[i];
  return sum == f[46];
}

bool LidarReader::pump(const Callback& on_point, int poll_timeout_ms) {
  // ---- Parser: safe, no dangling pointers ----
  auto parse_frames = [&](const Callback& cb)->bool {
    bool any = false;
    static constexpr size_t FRAME = 47; // correct LD06 frame size

    for (;;) {
      // 1) find header 0x54, length byte 0x2C
      size_t i = 0;
      bool found = false;
      for (; i + 2 <= buf_.size(); ++i) {
        if (buf_[i] == 0x54 && buf_[i + 1] == 0x2C) { found = true; break; }
      }
      if (!found) {
        // keep tail small
        if (buf_.size() > 4096) buf_.erase(buf_.begin(), buf_.end() - 1024);
        break;
      }

      // 2) if frame not complete yet, keep header and wait for more
      if (i + FRAME > buf_.size()) {
        if (i > 0) buf_.erase(buf_.begin(), buf_.begin() + i);
        break;
      }

      // 3) copy out the frame BEFORE erasing
      std::array<uint8_t, FRAME> fr{};
      std::copy(buf_.begin() + i, buf_.begin() + i + FRAME, fr.begin());

      // 4) erase this frame from buffer (including bytes before header)
      buf_.erase(buf_.begin(), buf_.begin() + i + FRAME);

      // 5) checksum
      if (!ld06_checksum_ok(fr.data(), FRAME)) {
        // bad frame; continue scanning
        continue;
      }

      // 6) parse fields
      int sa_cent = int(fr[4])  | (int(fr[5])  << 8); // start angle *100
      int ea_cent = int(fr[42]) | (int(fr[43]) << 8); // end angle *100
      int span    = ea_cent - sa_cent; if (span < 0) span += 36000;

      constexpr int N = 12;
      const int denom = (N > 1) ? (N - 1) : 1;
      const uint64_t t = mono_ns();

      // points: 12 × (dist_lo, dist_hi, intensity) at bytes [6..41]
      for (int k = 0; k < N; ++k) {
        const uint8_t dL = fr[6 + k*3 + 0];
        const uint8_t dH = fr[6 + k*3 + 1];
        const uint8_t I  = fr[6 + k*3 + 2];
        uint32_t dmm = (uint32_t(dH) << 8) | uint32_t(dL);

        if (dmm == 0 || dmm > (uint32_t)max_range_mm_) continue;
        uint32_t a_cdeg = (uint32_t)(sa_cent + (span * k) / denom) % 36000;

        cb(a_cdeg, dmm, (uint32_t)I, t);
        any = true;
      }
    }
    return any;
  };

#ifdef __linux__
  if (fd_ < 0) return false;

  // 1) gather bytes (nonblocking wait)
  pollfd pfd{fd_, POLLIN, 0};
  int pr = ::poll(&pfd, 1, poll_timeout_ms);
  if (pr > 0 && (pfd.revents & POLLIN)) {
    // read all available chunks
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

  // 2) parse frames
  return parse_frames(on_point);
#else
  // ---- Cross-platform simulator (macOS/Windows): emit one full revolution ----
  (void)poll_timeout_ms;

  #ifndef M_PI
  #define M_PI 3.14159265358979323846
  #endif

  static uint64_t t_rev = 0;
  if (t_rev == 0) t_rev = mono_ns();
  else t_rev += 100000000ull; // +100 ms per synthetic revolution

  static constexpr int STEP_CDEG = 100; // 1.00°
  for (int a_cdeg = 0; a_cdeg < 36000; a_cdeg += STEP_CDEG) {
    const double a_deg = a_cdeg / 100.0;
    double dist_m = 2.0 + std::sin(a_deg * M_PI / 180.0);
    if (dist_m < 0.05) dist_m = 0.05;

    uint32_t dmm = static_cast<uint32_t>(dist_m * 1000.0);
    if (dmm > static_cast<uint32_t>(max_range_mm_))
      dmm = static_cast<uint32_t>(max_range_mm_);

    const uint32_t intensity = 150;
    on_point(static_cast<uint32_t>(a_cdeg), dmm, intensity, t_rev);
  }
  return true;
#endif
}