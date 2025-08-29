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
#include "Ray.h"
#include "Motors.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using rover::v1::Telemetry;
using rover::v1::GridFrame;
using rover::v1::Pose2D;
using rover::v1::Ack;

#define HUB_ADDRESS "172.31.0.110:50051"

#define LIDAR_SERIAL_PORT "/dev/serial0"
#define LIDAR_SERIAL_BAUD 230400
#define LIDAR_MAX_MM 12000

#define GRID_WIDTH 1024
#define GRID_HEIGHT 1024
#define GRID_CELL_SIZE_M 0.05f

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

static inline int32_t angDiffCdeg(int32_t a, int32_t b) {
    // shortest signed diff (-18000..18000]
    return (a - b + 54000) % 36000 - 18000;
}

int main() {
    Motors motors;

    motors.forward();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    motors.left();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    motors.right();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    motors.reverse();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    motors.stop();

    OccupancyGrid<GRID_WIDTH, GRID_HEIGHT> grid(GRID_CELL_SIZE_M);

    // grpc setup
    auto channel = grpc::CreateChannel(HUB_ADDRESS, grpc::InsecureChannelCredentials());
    std::unique_ptr<Telemetry::Stub> stub = Telemetry::NewStub(channel);

    grpc::ClientContext gridCtx;
    rover::v1::Ack gridAck;
    auto gridWriter = stub->PublishGrid(&gridCtx, &gridAck);

    grpc::ClientContext poseCtx;
    rover::v1::Ack poseAck;
    auto poseWriter = stub->PublishPose(&poseCtx, &poseAck);

    // Lidar and SLAM setup
    float rover_x_m = GRID_WIDTH * GRID_CELL_SIZE_M / 2;
    float rover_y_m = GRID_HEIGHT * GRID_CELL_SIZE_M / 2;
    float rover_theta = M_PI;

    std::signal(SIGINT, on_sigint);

    LidarReader lr(LIDAR_SERIAL_PORT, LIDAR_SERIAL_BAUD, LIDAR_MAX_MM);
    if (!lr.open()) {
        std::fprintf(stderr, "[lidar] failed to open %s @%d\n", LIDAR_SERIAL_PORT, LIDAR_SERIAL_BAUD);
        return 1;
    }
    std::fprintf(stderr, "[lidar] reading %s @%d (max=%dmm)\n", LIDAR_SERIAL_PORT, LIDAR_SERIAL_BAUD, LIDAR_MAX_MM);

    std::vector<Ray> buffer;
    std::vector<Ray> backBuffer;
    uint32_t last_angle_cdeg;
    auto cb = [&](uint32_t angle_cdeg, uint32_t dist_mm, uint32_t intensity, uint64_t t_ns) {
        if (angle_cdeg < last_angle_cdeg) {
            // bring backbuffer to frontbuffer
            // Clean backbuffer
            buffer = std::move(backBuffer);
        }

        Ray ray(rover_x_m, rover_y_m, angle_cdeg / 100.0f, dist_mm, t_ns);
        backBuffer.push_back(ray);

        last_angle_cdeg = angle_cdeg;
    };

    auto last_push = std::chrono::steady_clock::now();
    while (g_run.load()) {
        lr.pump(cb, 10); 

        auto now = std::chrono::steady_clock::now();
        if (now - last_push >= std::chrono::seconds(5)) {
            
            std::cout << "Buffer size: " << buffer.size() << std::endl;
            for (Ray& ray : buffer) {
                auto [gx, gy] = grid.worldToGrid(ray.point_x_m, ray.point_y_m);
                if (grid.inBounds(gx, gy)) grid.at(gx, gy) += 80;

                auto [rgx, rgy] = grid.worldToGrid(rover_x_m, rover_y_m);
                int dx = std::abs(gx - rgx), sx = rgx < gx ? 1 : -1;
                int dy = -std::abs(gy - rgy), sy = rgy < gy ? 1 : -1;
                int err = dx + dy, e2;
                while (true) {
                    if (grid.inBounds(rgx, rgy)) {
                        uint8_t& v = grid.at(rgx, rgy);
                        if (v > 0) v -= 20;
                    }
                    if (rgx == gx && rgy == gy) break;
                    e2 = 2 * err;
                    if (e2 >= dy) { err += dy; rgx += sx; }
                    if (e2 <= dx) { err += dx; rgy += sy; }
                }
            }

            // Push telemetry over GRPC
            GridFrame gridFrame;
            gridFrame.set_width(GRID_WIDTH);
            gridFrame.set_height(GRID_HEIGHT);
            gridFrame.set_cell_size_m(GRID_CELL_SIZE_M);
            gridFrame.set_data(reinterpret_cast<const char*>(grid.data.data()), grid.data.size());

            if (!gridWriter->Write(gridFrame)) {
                std::fprintf(stderr, "[grpc] stream closed by server during Write()\n");
                break;
            }

            Pose2D pose2D;
            pose2D.set_x_m(rover_x_m);
            pose2D.set_y_m(rover_y_m);
            pose2D.set_theta(rover_theta);

            if (!poseWriter->Write(pose2D)) {
                std::fprintf(stderr, "[grpc] stream closed by server during Write()\n");
                break;
            }

            last_push = now;
            std::fprintf(stderr, "[grpc] pushed grid snapshot (%ux%u)\n", GRID_WIDTH, GRID_HEIGHT);
        }
    }

    gridWriter->WritesDone();
    poseWriter->WritesDone();
    lr.close();
    return 0;
}