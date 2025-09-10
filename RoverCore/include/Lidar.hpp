#pragma once

#include <iostream>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

struct Lidar
{
    uint32_t angle_cdeg;
    uint32_t distance_mm;
    uint32_t intensity;
    uint64_t time_ns;
};

class LidarReader {
public:
  using Callback = std::function<void(uint32_t angle_cdeg,
                                      uint32_t distance_mm,
                                      uint32_t intensity,
                                      uint64_t t_ns)>;

  explicit LidarReader(std::string port = "/dev/serial0", int baud = 230400, int max_range_mm = 12000);

  ~LidarReader();

  bool open();
  void close();
  bool isOpen() const { return fd_ >= 0; }

  bool pump(const Callback& on_point, int poll_timeout_ms = 10);

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