#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <thread>
#include <cmath>
#include <fstream>
#include <string>
#include <memory>
#include <grpcpp/grpcpp.h>
#include "telemetry.grpc.pb.h"

#include "LidarReader.h"
#include "OccupancyGrid.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using rover::v1::Telemetry;
using rover::v1::GridFrame;
using rover::v1::Ack;

#define HUB_ADDRESS "127.0.0.1:50051"

#define GRID_WIDTH 1024
#define GRID_HEIGHT 1024
#define GRID_CELL_SIZE_M 0.01f

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

static std::string env_str(const char* k, const char* d){ const char* v=getenv(k); return v? v : d; }
static int env_int(const char* k, int d){ const char* v=getenv(k); return v? std::atoi(v) : d; }

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

static inline Point sample_from_origin(Point origin, float angle_deg, float distance_mm) {
    float r = distance_mm * 0.001f;
    float theta_math_deg = 90.0f - angle_deg;
    float rad = theta_math_deg * float(M_PI) / 180.0f;
    float x = origin.x + r * std::cos(rad);
    float y = origin.y + r * std::sin(rad);
    return {x, y};
}

int main(){
    OccupancyGrid<GRID_WIDTH, GRID_HEIGHT> grid(GRID_CELL_SIZE_M);

    // grpc setup
    auto channel = grpc::CreateChannel(HUB_ADDRESS, grpc::InsecureChannelCredentials());
    std::unique_ptr<Telemetry::Stub> stub = Telemetry::NewStub(channel);

    ClientContext ctx;
    Ack ack;
    std::unique_ptr<grpc::ClientWriter<GridFrame>> writer(stub->PublishGrid(&ctx, &ack));
    if(!writer){
        fprintf(stderr, "[rover] failed to create writer\n");
        return 1;
    }

    // Lidar and SLAM setup
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

    auto cb = [&](uint32_t angle_cdeg, uint32_t dist_mm, uint32_t intensity, uint64_t t_ns){

        Point samplePoint = sample_from_origin({rover_x_m, rover_y_m}, angle_cdeg / 100.0f, dist_mm);
        auto [gx, gy] = grid.worldToGrid(samplePoint.x, samplePoint.y);
        if (grid.inBounds(gx, gy)) grid.at(gx, gy) += 8;

        auto [rgx, rgy] = grid.worldToGrid(rover_x_m, rover_y_m);
        int dx = std::abs(gx - rgx), sx = rgx < gx ? 1 : -1;
        int dy = -std::abs(gy - rgy), sy = rgy < gy ? 1 : -1;
        int err = dx + dy, e2;
        while (true) {
            if (grid.inBounds(rgx, rgy)) {
                uint8_t& v = grid.at(rgx, rgy);
                if (v > 0) v -= 2;
            }
            if (rgx == gx && rgy == gy) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; rgx += sx; }
            if (e2 <= dx) { err += dx; rgy += sy; }
        }
    };

    auto last_push = std::chrono::steady_clock::now();

    while (g_run.load()){
        lr.pump(cb, 10); 

        auto now = std::chrono::steady_clock::now();
        if (now - last_push >= std::chrono::seconds(5)) {
            GridFrame msg;
            msg.set_width(GRID_WIDTH);
            msg.set_height(GRID_HEIGHT);
            msg.set_cell_size_m(GRID_CELL_SIZE_M);
            msg.set_data(reinterpret_cast<const char*>(grid.data.data()), grid.data.size());
            if (!writer->Write(msg)) {
                std::fprintf(stderr, "[grpc] stream closed by server during Write()\n");
                break;
            }
            last_push = now;
            std::fprintf(stderr, "[grpc] pushed grid snapshot (%ux%u)\n", GRID_WIDTH, GRID_HEIGHT);
        }
    }

    if (!save_grid_pgm(grid, "/tmp/occgrid.pgm")) {
        std::fprintf(stderr, "[grid] failed to write /tmp/occgrid.pgm\n");
    } else {
        std::fprintf(stderr, "[grid] wrote /tmp/occgrid.pgm\n");
    }

    writer->WritesDone();
    Status st = writer->Finish();
    lr.close();
    return 0;
}