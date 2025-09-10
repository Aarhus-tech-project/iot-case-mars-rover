#ifndef __linux__

#include "Lidar.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <utility>

using namespace std;

LidarReader::LidarReader(std::string port, int baud, int max_range_mm)
  : port_(std::move(port)), baud_(baud), max_range_mm_(max_range_mm) {}

LidarReader::~LidarReader() { close(); }

uint64_t LidarReader::mono_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

unsigned long LidarReader::baud_to_term_(int baud) {
  return 0;
}

bool LidarReader::configure_port_() {
  return false;
}

bool LidarReader::open() {
  return true;
}

void LidarReader::close() {

}

bool LidarReader::pump(const Callback& on_point, int poll_timeout_ms) {
  // local parser lambda so both branches use the exact same code
  auto parse_frames = [&](const Callback& cb)->bool {
    bool any = false;
    const size_t FRAME = 46;

    for (;;) {
      // find header
      size_t i = 0;
      bool found = false;
      for (; i + FRAME <= buf_.size(); ++i) {
        if (buf_[i] == 0x54 && buf_[i + 1] == 0x2C) { found = true; break; }
      }
      if (!found) {
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

        cb(a_cdeg, dmm, (uint32_t)I, t);
        any = true;
      }
    }
    return any;
  };

  static uint64_t last_t = 0;
  static uint32_t start_angle_cdeg = 0;  // frame start angle (centideg)
  uint64_t now = mono_ns();

  // time-based spin: ~600 RPM default (10 rps)
  const double rpm = 600.0;
  const double cdeg_per_sec = rpm * 36000.0 / 60.0;

  if (last_t == 0) last_t = now;
  double dt = (now - last_t) * 1e-9; // seconds since last frame
  last_t = now;

  // advance start angle according to time
  start_angle_cdeg = (start_angle_cdeg + (uint32_t)(cdeg_per_sec * dt)) % 36000;

  // frame parameters matching your parser
  constexpr int N = 12;
  const int step_cdeg = 300;                     // 3.00° per sample (arbitrary but consistent)
  const int span = step_cdeg * (N - 1);          // total span in centideg for this frame
  const uint32_t end_angle_cdeg = (start_angle_cdeg + span) % 36000;

  // synthesize one frame
  uint8_t f[46] = {};
  f[0] = 0x54; f[1] = 0x2C;                      // header
  // bytes [2..3] can be zero (speed/ct), not used by your parser
  f[4] = (uint8_t)(start_angle_cdeg & 0xFF);
  f[5] = (uint8_t)((start_angle_cdeg >> 8) & 0xFF);

  // distances + intensities: 12 triples at f[6 + 3*k]
  for (int k = 0; k < N; ++k) {
    // reproduce the same angle math your parser will compute
    uint32_t a_cdeg = (uint32_t)(start_angle_cdeg + (span * k) / (N - 1)) % 36000;
    double a_deg = a_cdeg / 100.0;

    // scene: 2m base + sine ripple (1..3m)
    double dist_m = 2.0 + std::sin(a_deg * M_PI / 180.0);
    if (dist_m < 0.05) dist_m = 0.05;            // avoid zero
    uint32_t dmm = (uint32_t)(dist_m * 1000.0);
    if (dmm > (uint32_t)max_range_mm_) dmm = (uint32_t)max_range_mm_;

    f[6 + 3*k + 0] = (uint8_t)(dmm & 0xFF);
    f[6 + 3*k + 1] = (uint8_t)((dmm >> 8) & 0xFF);
    f[6 + 3*k + 2] = (uint8_t)(100 + (a_cdeg / 100) % 100); // fake intensity 100..199
  }

  // leave bytes [42..43] as end angle (little endian) because you use them
  f[42] = (uint8_t)(end_angle_cdeg & 0xFF);
  f[43] = (uint8_t)((end_angle_cdeg >> 8) & 0xFF);
  // bytes [44..45] could be checksum; your parser doesn’t check, so zero is fine.

  // append frame to the same buffer the Linux path fills
  buf_.insert(buf_.end(), f, f + sizeof(f));

  // parse exactly like Linux
  bool any = parse_frames(on_point);
  // respect poll_timeout_ms semantics a bit: if caller asked to wait, emulate small delay
  (void)poll_timeout_ms; // not strictly needed; keeping signature
  return any;
}

#endif