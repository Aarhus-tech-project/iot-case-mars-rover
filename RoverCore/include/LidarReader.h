#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

class LidarReader {
public:
  using Callback = std::function<void(uint32_t angle_cdeg,
                                      uint32_t distance_mm,
                                      uint32_t intensity,
                                      uint64_t t_ns)>;

  // LD06/LD19 defaults: /dev/serial0 @ 230400, clamp >6000 mm
  explicit LidarReader(std::string port = "/dev/serial0", int baud = 230400, int max_range_mm = 6000);

  ~LidarReader();

  // Open/close the serial device
  bool open();
  void close();
  bool isOpen() const { return fd_ >= 0; }

  // Read + parse; invokes callback for each valid sample found.
  // Returns true if any samples were produced this call.
  bool pump(const Callback& on_point, int poll_timeout_ms = 10);

  // Optional tweaks
  void setPort(std::string port) { port_ = std::move(port); }
  void setBaud(int baud) { baud_ = baud; }
  void setMaxRangeMm(int mm) { max_range_mm_ = mm; }

private:
  static uint64_t mono_ns();
  static unsigned long baud_to_term_(int baud);
  bool configure_port_();

  int fd_ = -1;
  std::string port_;
  int baud_;
  int max_range_mm_;
  std::vector<uint8_t> buf_;
};