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
#include <vector>
#include <optional>
#include <grpcpp/grpcpp.h>
#include <grpcpp/support/channel_arguments.h>
#include "telemetry.grpc.pb.h"

#include "Config.hpp"
#include "Lidar.hpp"
#include "OccupancyGrid.hpp"
#include "Ray.hpp"
#include "Motors.hpp"
#include "CommandStreamClient.hpp"
#include "TelemetryStream.hpp"
#include "BNO055.hpp"

// Signal-safe global control flags and contexts
static std::atomic<bool> g_run{true};

// Signal handler for graceful shutdown
static void on_sigint(int) {
    static std::atomic<bool> sigint_handled{false};
    if (!sigint_handled.exchange(true)) {
        std::fprintf(stderr, "\n[signal] Caught signal, shutting down gracefully...\n");
        g_run.store(false);
    } else {
        std::fprintf(stderr, "\n[signal] Signal received again, forcing exit\n");
        std::exit(1);
    }
}

int main() {
    try {
        std::fprintf(stderr, "[main] Starting\n");
        OccupancyGrid<GRID_WIDTH, GRID_HEIGHT> grid(GRID_CELL_SIZE_M);

        // grpc setup
        grpc::ChannelArguments args;
        args.SetInt(GRPC_ARG_KEEPALIVE_TIME_MS, 30000);
        args.SetInt(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 10000);
        args.SetInt(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);
        auto channel = grpc::CreateCustomChannel(HUB_ADDRESS, grpc::InsecureChannelCredentials(), args);
        TelemetryStream telemetry(channel);

        Motors motors;
        CommandStreamClient cmd(channel, &motors);
        cmd.Start();

        // Register SIGINT and SIGTERM handlers
        std::signal(SIGINT, on_sigint);
        std::signal(SIGTERM, on_sigint);

        // Lidar setup
        float rover_x_m = GRID_WIDTH * GRID_CELL_SIZE_M / 2;
        float rover_y_m = GRID_HEIGHT * GRID_CELL_SIZE_M / 2;
        float rover_rot_deg = 0;

        LidarReader lr(LIDAR_SERIAL_PORT, LIDAR_SERIAL_BAUD, LIDAR_MAX_MM);
        if (!lr.open()) {
            std::fprintf(stderr, "[lidar] failed to open %s @%d\n", LIDAR_SERIAL_PORT, LIDAR_SERIAL_BAUD);
            return 1;
        }
        std::fprintf(stderr, "[lidar] reading %s @%d (max=%dmm)\n", LIDAR_SERIAL_PORT, LIDAR_SERIAL_BAUD, LIDAR_MAX_MM);

        BNO055 imu("/dev/i2c-1");
        if (!imu.begin(true, BNO055::MODE_NDOF)) {
            std::fprintf(stderr, "[imu] failed to initialize BNO055\n");
            //return 1;
        } else {
            std::fprintf(stderr, "[imu] BNO055 initialized\n");
        }

        std::vector<Lidar> buffer;
        std::vector<Lidar> backBuffer;
        uint32_t last_angle_cdeg = 0;
        auto cb = [&](Lidar lidar) {
            if (lidar.angle_cdeg < last_angle_cdeg) {
                buffer = std::move(backBuffer);
                backBuffer.clear();
            }

            backBuffer.push_back(lidar);
            last_angle_cdeg = lidar.angle_cdeg;
        };

        // Start Lidar thread
        std::thread lidar_thread([&]() {
            std::fprintf(stderr, "[lidar_thread] started\n");
            while (g_run.load()) {
                lr.pump(cb, 10);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            std::fprintf(stderr, "[lidar_thread] exiting\n");
        });

        auto last_push = std::chrono::steady_clock::now();
        while (g_run.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            auto now = std::chrono::steady_clock::now();
            if (now - last_push >= std::chrono::milliseconds(1000)) {
            if (!buffer.empty()) {
                std::printf("[main] got %zu lidar points\n", buffer.size());
                    LidarScan scan;
                    for (const Lidar& lidar : buffer) {
                        Ray ray(rover_x_m, rover_y_m, rover_rot_deg + lidar.angle_cdeg / 100.0f, lidar.distance_mm, lidar.time_ns);
                        grid.populateRayOnGrid(ray);

                        auto* p = scan.add_points();
                        p->set_x_m(ray.point_x_m);
                        p->set_y_m(ray.point_y_m);
                    }

                    telemetry.SendLidar(scan);
                }

                //telemetry.SendGrid(grid);
                //telemetry.SendPose(rover_x_m, rover_y_m, rover_rot_deg);

                BNO055::Sample imuSample = imu.readSample();

                telemetry.SendIMU(imuSample);

                last_push = now;
            }
        }
        std::fprintf(stderr, "[main] g_run is false, exiting main loop\n");
        g_run.store(false);

        if (lidar_thread.joinable()) {
            lidar_thread.join();
        }

        /*
        //Pose prior = {rover_x_m, rover_y_m, rover_rot_deg, 1.0f};
        auto result = TryLocalizeLidar(grid, buffer, std::nullopt);

        if (result) {
            std::printf("[mcl] FINAL pose=(%.2f,%.2f, %.1f°)  w=%.3f\n",
                        result->x, result->y, result->heading_deg, result->weight);
        } else {
            // tell the rest of your system we need more info (e.g., rotate-in-place scan, odom hint, etc.)
        }
        */

        lr.close();
        cmd.Stop();

        return 0;
    }
    catch (const std::exception& ex) {
        std::fprintf(stderr, "[main] exception: %s\n", ex.what());
        return 1;
    }
    catch (...) {
        std::fprintf(stderr, "[main] unknown exception\n");
        return 1;
    }
}