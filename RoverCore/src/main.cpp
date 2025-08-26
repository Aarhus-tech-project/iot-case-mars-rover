#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <thread>
#include <cmath>

#include "LidarReader.h"
#include "OccupancyGrid.h"

#define GRID_WIDTH 1024
#define GRID_HEIGHT 1024
#define GRID_CELL_SIZE_M 0.05f

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

static std::string env_str(const char* k, const char* d){ const char* v=getenv(k); return v? v : d; }
static int env_int(const char* k, int d){ const char* v=getenv(k); return v? std::atoi(v) : d; }

void render_ascii(const OccupancyGrid<GRID_WIDTH, GRID_HEIGHT>& grid, int rover_gx, int rover_gy)
{
    // Hide cursor, clear, home
    std::fputs("\x1b[?25l\x1b[2J\x1b[H", stdout);

    // Optional border
    std::putchar('+');
    for (size_t x = 0; x < grid.width; ++x) std::putchar('-');
    std::puts("+");

    for (size_t y = 0; y < grid.height; ++y) {
        std::putchar('|');
        for (size_t x = 0; x < grid.width; ++x) {
            if ((int)x == rover_gx && (int)y == rover_gy) { std::putchar('R'); continue; }
            uint8_t v = grid.at(x, y);
            char c = (v == 0) ? '.' : (v < 64) ? ':' : (v < 128) ? 'o' : (v < 192) ? 'O' : '#';
            std::putchar(c);
        }
        std::puts("|");
    }

    std::putchar('+');
    for (size_t x = 0; x < grid.width; ++x) std::putchar('-');
    std::puts("+");

    std::fflush(stdout);
}

// map 0..255 to grayscale and write PGM
template <size_t W, size_t H>
bool save_grid_pgm(const OccupancyGrid<W,H>& grid, const char* path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    // P5 = binary graymap, W H, maxval 255
    f << "P5\n" << W << " " << H << "\n255\n";
    // top row first? If you want (0,0) bottom-left, reverse y here.
    for (size_t y = 0; y < H; ++y) {
        for (size_t x = 0; x < W; ++x) {
            uint8_t v = grid.at(x, y);
            f.put(static_cast<char>(v));
        }
    }
    return f.good();
}

struct Point {
    float x, y;
};

Point sample_from_origin(Point origin, float angle_deg, float distance_mm) {
    float rad = angle_deg * M_PI / 180.0;
    float x = origin.x + distance_mm / 1000.0f * std::cos(rad);
    float y = origin.y + distance_mm / 1000.0f * std::sin(rad);
    return {x, y};
}

int main(){
  OccupancyGrid<GRID_WIDTH, GRID_HEIGHT> grid(GRID_CELL_SIZE_M);

  float rover_x_m = GRID_WIDTH * GRID_CELL_SIZE_M / 2;
  float rover_y_m = GRID_HEIGHT * GRID_CELL_SIZE_M / 2;

  std::signal(SIGINT, on_sigint);

  std::string port = "/dev/serial0";
  int baud = 230400;
  int max_mm = 12000;

  LidarReader lr(port, baud, max_mm);
  if (!lr.open()) {
    std::fprintf(stderr, "[lidar] failed to open %s @%d\n", port.c_str(), baud);
    return 1;
  }
  std::fprintf(stderr, "[lidar] reading %s @%d (max=%dmm)\n", port.c_str(), baud, max_mm);

  uint64_t last_log = 0;

  auto last_draw = std::chrono::steady_clock::now();

  auto cb = [&](uint32_t angle_cdeg, uint32_t dist_mm, uint32_t intensity, uint64_t t_ns){
      // (optional) filter weak returns
      if (intensity < 5) return;

      Point samplePoint = sample_from_origin({rover_x_m, rover_y_m}, angle_cdeg / 100.0f, dist_mm);
      auto [gx, gy] = grid.worldToGrid(samplePoint.x, samplePoint.y);
      if (grid.inBounds(gx, gy)) grid.at(gx, gy) = 255;
  };

  while (g_run.load()){
      lr.pump(cb, 10); // poll waits; no extra sleep needed

      auto now = std::chrono::steady_clock::now();
      if (now - last_draw >= std::chrono::milliseconds(100)) { // ~10 FPS
          // rover grid coords (center)
          int rover_gx, rover_gy;
          std::tie(rover_gx, rover_gy) = grid.worldToGrid(rover_x_m, rover_y_m);
          //render_ascii(grid, rover_gx, rover_gy);
          last_draw = now;
      }
  }

  // on exit, show cursor again
  std::fputs("\x1b[?25h\n", stdout);

  if (!save_grid_pgm(grid, "/tmp/occgrid.pgm")) {
      std::fprintf(stderr, "[grid] failed to write /tmp/occgrid.pgm\n");
  } else {
      std::fprintf(stderr, "[grid] wrote /tmp/occgrid.pgm\n");
  }

  lr.close();
  return 0;
}