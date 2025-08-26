#include "LidarReader.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <thread>

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

static std::string env_str(const char* k, const char* d){ const char* v=getenv(k); return v? v : d; }
static int env_int(const char* k, int d){ const char* v=getenv(k); return v? std::atoi(v) : d; }

int main(){
  std::signal(SIGINT, on_sigint);

  std::string port = env_str("LIDAR_PORT", "/dev/serial0");
  int baud        = env_int("LIDAR_BAUD", 230400);
  int max_mm      = env_int("MAX_RANGE_MM", 6000);

  LidarReader lr(port, baud, max_mm);
  if (!lr.open()) {
    std::fprintf(stderr, "[lidar] failed to open %s @%d\n", port.c_str(), baud);
    return 1;
  }
  std::fprintf(stderr, "[lidar] reading %s @%d (max=%dmm)\n", port.c_str(), baud, max_mm);

  uint64_t count = 0, last_log = 0;

  auto cb = [&](uint32_t angle_cdeg, uint32_t dist_mm, uint32_t intensity, uint64_t t_ns){
    // mono_ns angle_cdeg distance_mm intensity
    std::printf("%llu %u %u %u\n",
                (unsigned long long)t_ns, angle_cdeg, dist_mm, intensity);
    ++count;
  };

  while (g_run.load()){
    lr.pump(cb, /*poll_timeout_ms*/ 10); // poll handles waiting
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    if (std::chrono::duration_cast<std::chrono::seconds>(now).count() != (long)last_log){
      std::fprintf(stderr, "[lidar] samples=%llu\n", (unsigned long long)count);
      last_log = std::chrono::duration_cast<std::chrono::seconds>(now).count();
    }
  }

  lr.close();
  std::fprintf(stderr, "[lidar] bye\n");
  return 0;
}