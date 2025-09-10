#ifndef __linux__

#include "Lidar.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>

using namespace std;

static inline uint64_t mono_ns_now() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

// MSVC may not define M_PI unless _USE_MATH_DEFINES; provide a fallback.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LidarReader::LidarReader(std::string port, int baud, int max_range_mm)
  : port_(std::move(port)), baud_(baud), max_range_mm_(max_range_mm) {}

LidarReader::~LidarReader() { close(); }

uint64_t LidarReader::mono_ns() { return mono_ns_now(); }

unsigned long LidarReader::baud_to_term_(int) { return 0; }
bool LidarReader::configure_port_() { return false; }
bool LidarReader::open() { return true; }
void LidarReader::close() {}

// --- LD06 helpers ---
static inline void wr_le16(uint8_t* p, uint16_t v) {
  p[0] = uint8_t(v & 0xFF);
  p[1] = uint8_t((v >> 8) & 0xFF);
}
static inline uint8_t ld06_checksum47(const uint8_t* f) {
  uint8_t s = 0; for (int i = 0; i < 46; ++i) s += f[i]; return s;
}
static inline bool ld06_checksum_ok47(const uint8_t* f) {
  return ld06_checksum47(f) == f[46];
}

bool LidarReader::pump(const Callback& on_point, int /*poll_timeout_ms*/) {
  // ---------- Timing model ----------
  // Goal: 360 points/rev → 30 frames/rev × 12 points/frame.
  // Default: 600 RPM = 10 rps → 300 frames/sec → one frame every ~3.333 ms.
  static double rpm = 600.0;
  static constexpr int POINTS_PER_FRAME = 12;
  static constexpr int FRAMES_PER_REV   = 360 / POINTS_PER_FRAME; // 30
  static_assert(FRAMES_PER_REV * POINTS_PER_FRAME == 360, "must divide exactly");

  static uint64_t last_ns = 0;
  static double frame_accum = 0.0; // fractional frames due
  static uint32_t next_start_deg = 0; // 0..359 (whole degrees)

  const uint64_t now = mono_ns();
  if (last_ns == 0) last_ns = now;
  const double dt_s = (now - last_ns) * 1e-9;
  last_ns = now;

  const double rps = rpm / 60.0;
  const double frames_per_s = rps * FRAMES_PER_REV;         // 300 @ 10 rps
  frame_accum += dt_s * frames_per_s;

  // Emit as many whole frames as are due
  int frames_to_emit = (int)frame_accum;
  if (frames_to_emit <= 0) frames_to_emit = 1; // ensure progress even if caller is slow
  frame_accum -= frames_to_emit;

  for (int fidx = 0; fidx < frames_to_emit; ++fidx) {
    // Build one **valid 47-byte LD06 frame** with 12 points at 1° steps
    // Start angle = next_start_deg *100; points at start + k*1°, k=0..11
    // End angle = start + 11°
    const uint16_t start_cdeg = (uint16_t)(next_start_deg * 100);
    const uint16_t end_cdeg   = (uint16_t)(((next_start_deg + (POINTS_PER_FRAME - 1)) % 360) * 100);

    std::array<uint8_t, 47> fr{};
    fr[0] = 0x54; fr[1] = 0x2C;                // header + payload len
    wr_le16(&fr[2], (uint16_t)(rpm * 64.0));   // fake motor speed (not used by parser)
    wr_le16(&fr[4], start_cdeg);

    for (int k = 0; k < POINTS_PER_FRAME; ++k) {
      const int deg = (next_start_deg + k) % 360;         // exact 1° step
      const double a_deg = double(deg);
      // Simple scene: 2.0 m + 1.0 m * sin(), clamped at 50 mm
      double dist_m = 2.0 + std::sin(a_deg * (M_PI / 180.0));
      if (dist_m < 0.05) dist_m = 0.05;
      uint32_t dmm = (uint32_t)(dist_m * 1000.0);
      if (dmm > (uint32_t)max_range_mm_) dmm = (uint32_t)max_range_mm_;

      const int base = 6 + k * 3;
      fr[base + 0] = uint8_t(dmm & 0xFF);
      fr[base + 1] = uint8_t((dmm >> 8) & 0xFF);
      fr[base + 2] = uint8_t(120 + (deg % 80)); // fake intensity 120..199
    }

    wr_le16(&fr[42], end_cdeg);
    wr_le16(&fr[44], (uint16_t)((now / 1000) & 0xFFFF)); // dummy timestamp
    fr[46] = ld06_checksum47(fr.data());

    // Append to rx buffer to be parsed below
    buf_.insert(buf_.end(), fr.begin(), fr.end());

    // Advance to the next 12-degree block, wrap at 360
    next_start_deg = (next_start_deg + POINTS_PER_FRAME) % 360;
  }

  // ---------- Parse frames from buf_ (safe parser, 47 bytes) ----------
  auto parse_frames = [&](const Callback& cb)->bool {
    bool any = false;
    static constexpr size_t FRAME = 47;

    for (;;) {
      // Find header
      size_t i = 0;
      bool found = false;
      for (; i + 2 <= buf_.size(); ++i) {
        if (buf_[i] == 0x54 && buf_[i + 1] == 0x2C) { found = true; break; }
      }
      if (!found) {
        if (buf_.size() > 4096) buf_.erase(buf_.begin(), buf_.end() - 1024);
        break;
      }
      // Need full frame
      if (i + FRAME > buf_.size()) {
        if (i > 0) buf_.erase(buf_.begin(), buf_.begin() + i);
        break;
      }

      // Copy out before erasing (avoid dangling pointers)
      std::array<uint8_t, FRAME> f{};
      std::copy(buf_.begin() + i, buf_.begin() + i + FRAME, f.begin());
      buf_.erase(buf_.begin(), buf_.begin() + i + FRAME);

      // Checksum
      if (!ld06_checksum_ok47(f.data())) continue;

      // Angles (centi-degrees)
      int sa_cent = int(f[4])  | (int(f[5])  << 8);
      int ea_cent = int(f[42]) | (int(f[43]) << 8);
      int span    = ea_cent - sa_cent; if (span < 0) span += 36000;

      constexpr int N = POINTS_PER_FRAME;       // 12
      const int denom = (N > 1) ? (N - 1) : 1;
      const uint64_t t_ns = mono_ns();

      // 12 × (dist_lo, dist_hi, intensity) at [6..41]
      for (int k = 0; k < N; ++k) {
        const uint8_t dL = f[6 + 3*k + 0];
        const uint8_t dH = f[6 + 3*k + 1];
        const uint8_t I  = f[6 + 3*k + 2];
        uint32_t dmm = (uint32_t(dH) << 8) | uint32_t(dL);
        if (dmm == 0 || dmm > (uint32_t)max_range_mm_) continue;

        uint32_t a_cdeg = (uint32_t)(sa_cent + (span * k) / denom) % 36000;
        cb(a_cdeg, dmm, (uint32_t)I, t_ns);
        any = true;
      }
    }
    return any;
  };

  return parse_frames(on_point);
}

#endif // __linux__