#include "LidarReader.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <poll.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

LidarReader::LidarReader(std::string port, int baud, int max_range_mm)
  : port_(std::move(port)), baud_(baud), max_range_mm_(max_range_mm) {}

LidarReader::~LidarReader() { close(); }

uint64_t LidarReader::mono_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

unsigned long LidarReader::baud_to_term_(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B230400;
  }
}

bool LidarReader::configure_port_() {
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    perror("tcgetattr");
    return false;
  }

  cfmakeraw(&tio);
  cfsetspeed(&tio, baud_to_term_(baud_));

  // 8N1, enable receiver, local mode
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;

  // Nonblocking “packet at a time” via poll(); VMIN/VTIME = 0
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    perror("tcsetattr");
    return false;
  }
  tcflush(fd_, TCIFLUSH);
  return true;
}

bool LidarReader::open() {
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
}

void LidarReader::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool LidarReader::pump(const Callback& on_point, int poll_timeout_ms) {
  if (fd_ < 0) return false;
  bool any = false;

  // 1) Gather available bytes (nonblocking with small wait)
  pollfd pfd{fd_, POLLIN, 0};
  int pr = ::poll(&pfd, 1, poll_timeout_ms);
  if (pr > 0 && (pfd.revents & POLLIN)) {
    uint8_t tmp[512];
    ssize_t r = ::read(fd_, tmp, sizeof(tmp));
    if (r > 0) buf_.insert(buf_.end(), tmp, tmp + r);
  }

  // 2) Parse frames: LD06/LD19 → 46 bytes, starts 0x54 0x2C, 12 samples
  const size_t FRAME = 46;
  for (;;) {
    // Find header
    size_t i = 0;
    bool found = false;
    for (; i + FRAME <= buf_.size(); ++i) {
      if (buf_[i] == 0x54 && buf_[i + 1] == 0x2C) { found = true; break; }
    }
    if (!found) {
      // keep tail to avoid unbounded growth
      if (buf_.size() > 2048) buf_.erase(buf_.begin(), buf_.end() - 1024);
      break;
    }
    if (i + FRAME > buf_.size()) break; // incomplete

    const uint8_t* f = &buf_[i];
    buf_.erase(buf_.begin(), buf_.begin() + (i + FRAME));

    int sa_cent = (int)f[4]  | ((int)f[5]  << 8); // start angle *100
    int ea_cent = (int)f[42] | ((int)f[43] << 8); // end angle *100
    int span    = ea_cent - sa_cent; if (span < 0) span += 36000;

    constexpr int N = 12;
    const int denom = (N > 1) ? (N - 1) : 1;
    uint64_t t = mono_ns();

    for (int k = 0; k < N; ++k) {
      const uint8_t dL = f[6 + k*3 + 0];
      const uint8_t dH = f[6 + k*3 + 1];
      const uint8_t I  = f[6 + k*3 + 2];
      uint32_t dmm = ((uint32_t)dH << 8) | (uint32_t)dL;

      if (dmm == 0 || dmm > (uint32_t)max_range_mm_) continue;
      uint32_t a_cdeg = (uint32_t)(sa_cent + (span * k) / denom) % 36000;

      on_point(a_cdeg, dmm, (uint32_t)I, t);
      any = true;
    }
  }
  return any;
}