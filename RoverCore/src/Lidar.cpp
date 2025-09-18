#ifndef __linux__

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

inline uint64_t LidarReader::mono_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

LidarReader::LidarReader(std::string port, int baud, int max_range_mm)
  : port_(std::move(port)), baud_(baud), max_range_mm_(max_range_mm) {}

LidarReader::~LidarReader() { close(); }

unsigned long LidarReader::baud_to_term_(int baud) {
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
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

bool LidarReader::pump(const Callback& on_point, int poll_timeout_ms) {
  if (fd_ < 0) return false;

  constexpr uint8_t HDR0 = 0x54;
  constexpr uint8_t HDR1 = 0x2C;          // fixed length tag for LD06 12-pt packets
  constexpr size_t  PACKET_LEN = 47;      // total bytes including header+crc
  constexpr int     POINTS = 12;
  constexpr bool    DO_CHECK_CRC = true;

  auto crc8_ld06 = [](const uint8_t* data, size_t n) -> uint8_t {
    // CRC-8, poly 0x4D, init 0x00, no reflection, xorout 0x00
    uint8_t crc = 0x00;
    for (size_t i = 0; i < n; ++i) {
      crc ^= data[i];
      for (int b = 0; b < 8; ++b) {
        if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x4D);
        else            crc <<= 1;
      }
    }
    return crc;
  };

  // static buffer holds any leftover bytes between calls
  static std::vector<uint8_t> buf;

  // poll for input
  pollfd pfd{fd_, POLLIN, 0};
  int pr = ::poll(&pfd, 1, poll_timeout_ms);
  if (pr < 0) { perror("poll"); return false; }
  if (pr == 0 || !(pfd.revents & POLLIN)) {
    // timeout or no readable data; not fatal
    return true;
  }

  // read whatever is available
  uint8_t tmp[512];
  ssize_t n = ::read(fd_, tmp, sizeof(tmp));
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return true;
    perror("read");
    return false;
  }
  buf.insert(buf.end(), tmp, tmp + n);

  // parse frames
  size_t i = 0;
  auto now_ns = mono_ns();
  bool any_ok = false;

  while (true) {
    // find header 0x54 0x2C
    while (i + 1 < buf.size() && !(buf[i] == HDR0 && buf[i + 1] == HDR1)) ++i;
    if (i + 1 >= buf.size()) {
      // no header found; drop leading junk
      if (i > 0) buf.erase(buf.begin(), buf.begin() + i);
      break;
    }

    // need full packet
    if (buf.size() - i < PACKET_LEN) {
      // keep partial for next call
      if (i > 0) buf.erase(buf.begin(), buf.begin() + i);
      break;
    }

    const uint8_t* pkt = buf.data() + i;

    // optional CRC check (over first 46 bytes; last byte is CRC)
    if (DO_CHECK_CRC) {
      uint8_t want = pkt[PACKET_LEN - 1];
      uint8_t got  = crc8_ld06(pkt, PACKET_LEN - 1);
      if (want != got) {
        // bad frame; skip this header and try to re-sync
        ++i;
        continue;
      }
    }

    // unpack (little-endian)
    auto rd16 = [](const uint8_t* p) -> uint16_t { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); };

    uint16_t speed_dps     = rd16(pkt + 2);  // not used here, but parsed if needed
    uint16_t start_cdeg    = rd16(pkt + 4);
    // 12 samples start at offset 6, 3 bytes each: dist_lo, dist_hi, intensity
    uint16_t end_cdeg      = rd16(pkt + 6 + POINTS * 3);
    uint16_t ts_ms         = rd16(pkt + 6 + POINTS * 3 + 2);
    (void)speed_dps; (void)ts_ms;

    // fix wrap for the step computation
    int32_t start = (int32_t)start_cdeg;
    int32_t stop  = (int32_t)end_cdeg;
    if (stop < start) stop += 36000; // unwrap once

    // step across 11 intervals for 12 points
    const float step = (float)(stop - start) / (POINTS - 1);

    // emit points
    for (int k = 0; k < POINTS; ++k) {
      const uint8_t* p = pkt + 6 + k * 3;
      uint16_t dist_mm = rd16(p);
      uint8_t  intensity = p[2];

      if (dist_mm == 0 || (max_range_mm_ > 0 && dist_mm > (uint16_t)max_range_mm_)) {
        continue; // skip invalid/out-of-range
      }

      // angle in centi-degrees, wrapped to [0,36000)
      int32_t a = (int32_t)(start + step * k + 0.5f);
      a %= 36000;
      if (a < 0) a += 36000;

      Lidar point{ (uint32_t)a, (uint32_t)dist_mm, (uint32_t)intensity, now_ns };
      on_point(point);
      any_ok = true;
    }

    // consume this packet and continue parsing
    i += PACKET_LEN;
    if (i > 0) {
      buf.erase(buf.begin(), buf.begin() + i);
      i = 0;
    }
  }

  return any_ok || pr >= 0;
}

#endif // __linux__