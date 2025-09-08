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

#include "Config.h"
#include "Lidar.h"
#include "OccupancyGrid.h"
#include "MonteCarloLocalization.h"
#include "Ray.h"
#include "Motors.h"
#include "CommandStreamClient.h"
#include "OrientationEstimate.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using rover::v1::Telemetry;
using rover::v1::GridFrame;
using rover::v1::Pose2D;
using rover::v1::LidarScan;
using rover::v1::Ack;

// Signal-safe global control flags and contexts
static std::atomic<bool> g_run{true};
static std::unique_ptr<grpc::ClientContext> g_gridCtx;
static std::unique_ptr<grpc::ClientContext> g_poseCtx;
static std::unique_ptr<grpc::ClientContext> g_lidarCtx;

// Signal handler for graceful shutdown
static void on_sigint(int) {
    static std::atomic<bool> sigint_handled{false};
    if (!sigint_handled.exchange(true)) {
        std::fprintf(stderr, "\n[signal] Caught signal, shutting down gracefully...\n");
        g_run.store(false);
        if (g_gridCtx) g_gridCtx->TryCancel();
        if (g_poseCtx) g_poseCtx->TryCancel();
        if (g_lidarCtx) g_lidarCtx->TryCancel();
    } else {
        std::fprintf(stderr, "\n[signal] Signal received again, forcing exit\n");
        std::exit(1);
    }
}

int main() {
    std::mutex stream_mutex;
    bool lidarStreamClosed = false;
    bool gridStreamClosed = false;
    bool poseStreamClosed = false;

    try {
        std::fprintf(stderr, "[main] Starting\n");
        OccupancyGrid<GRID_WIDTH, GRID_HEIGHT> grid(GRID_CELL_SIZE_M);
        MonteCarloLocalization<GRID_WIDTH, GRID_HEIGHT> mcl(grid);

        // grpc setup
        grpc::ChannelArguments args;
        args.SetInt(GRPC_ARG_KEEPALIVE_TIME_MS, 30000);
        args.SetInt(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 10000);
        args.SetInt(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);
        auto channel = grpc::CreateCustomChannel(HUB_ADDRESS, grpc::InsecureChannelCredentials(), args);

        Motors motors;
        CommandStreamClient cmd(channel, &motors);
        cmd.Start();

        std::unique_ptr<Telemetry::Stub> stub = Telemetry::NewStub(channel);

        // Register SIGINT and SIGTERM handlers
        std::signal(SIGINT, on_sigint);
        std::signal(SIGTERM, on_sigint);

        // Setup GRPC streaming contexts
        g_gridCtx = std::make_unique<grpc::ClientContext>();
        g_poseCtx = std::make_unique<grpc::ClientContext>();
        g_lidarCtx = std::make_unique<grpc::ClientContext>();

        auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(10);
        g_gridCtx->set_deadline(deadline);
        g_poseCtx->set_deadline(deadline);
        g_lidarCtx->set_deadline(deadline);

        Ack gridAck, poseAck, lidarAck;
        auto gridWriter = stub->PublishGrid(g_gridCtx.get(), &gridAck);
        auto poseWriter = stub->PublishPose(g_poseCtx.get(), &poseAck);
        auto lidarWriter = stub->PublishLidar(g_lidarCtx.get(), &lidarAck);

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

        std::vector<Lidar> buffer;
        std::vector<Lidar> backBuffer;
        uint32_t last_angle_cdeg = 0;

        auto cb = [&](uint32_t angle_cdeg, uint32_t dist_mm, uint32_t intensity, uint64_t t_ns) {
            if (angle_cdeg < last_angle_cdeg) {
                buffer = std::move(backBuffer);
                backBuffer.clear();
            }

            Lidar lidar = { angle_cdeg, dist_mm, intensity, t_ns };
            backBuffer.push_back(lidar);
            last_angle_cdeg = angle_cdeg;
        };

        // Start Lidar thread
        std::thread lidar_thread([&]() {
            std::fprintf(stderr, "[lidar_thread] started\n");
            while (g_run.load()) {
                lr.pump(cb, 10);
                // Optional: add a small sleep here to avoid hogging CPU
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            std::fprintf(stderr, "[lidar_thread] exiting\n");
        });

        auto last_push = std::chrono::steady_clock::now();
        bool first = true;

        while (g_run.load()) {
            auto now = std::chrono::steady_clock::now();
            if (now - last_push >= std::chrono::milliseconds(1000)) {
                if (!buffer.empty()) {
                    if (first) {
                        first = false;
                        OrientationEstimate oe = EstimateHeadingFromScan(buffer, 3.0f, 0.30f, 0.02f, 12, LIDAR_MAX_MM);
                        rover_rot_deg = oe.snapped_up_deg;
                        std::printf("[slam] lidar points %zu, initial heading %.1f° (used %d segments)\n", buffer.size(), rover_rot_deg, oe.used_segments);
                    }

                    LidarScan scan;
                    for (const Lidar& lidar : buffer) {
                        Ray ray(rover_x_m, rover_y_m, rover_rot_deg + lidar.angle_cdeg / 100.0f, lidar.distance_mm, lidar.time_ns);
                        grid.populateRayOnGrid(ray);

                        auto* p = scan.add_points();
                        p->set_x_m(ray.point_x_m);
                        p->set_y_m(ray.point_y_m);
                    }

                    {
                        std::lock_guard<std::mutex> lock(stream_mutex);
                        if (!lidarWriter->Write(scan)) {
                            std::fprintf(stderr, "[grpc] lidar stream closed by server\n");
                            lidarStreamClosed = true;
                            break;
                        }
                    }
                }

                GridFrame gridFrame;
                gridFrame.set_width(GRID_WIDTH);
                gridFrame.set_height(GRID_HEIGHT);
                gridFrame.set_cell_size_m(GRID_CELL_SIZE_M);
                gridFrame.set_data(reinterpret_cast<const char*>(grid.data()), grid.size());

                {
                    std::lock_guard<std::mutex> lock(stream_mutex);
                    if (!gridWriter->Write(gridFrame)) {
                        std::fprintf(stderr, "[grpc] grid stream closed by server\n");
                        gridStreamClosed = true;
                        break;
                    }
                }

                Pose2D pose2D;
                pose2D.set_x_m(rover_x_m);
                pose2D.set_y_m(rover_y_m);
                pose2D.set_rot_deg(rover_rot_deg);

                {
                    std::lock_guard<std::mutex> lock(stream_mutex);
                    if (!poseWriter->Write(pose2D)) {
                        std::fprintf(stderr, "[grpc] pose stream closed by server\n");
                        poseStreamClosed = true;
                        break;
                    }
                }

                last_push = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::fprintf(stderr, "[main] g_run is false, exiting main loop\n");

        g_run.store(false);

        // Ensure gRPC calls are cancelled
        if (g_lidarCtx) g_lidarCtx->TryCancel();
        if (g_gridCtx) g_gridCtx->TryCancel();
        if (g_poseCtx) g_poseCtx->TryCancel();

        std::fprintf(stderr, "[main] Joining lidar thread...\n");
        lidar_thread.join();
        std::fprintf(stderr, "[main] Lidar thread joined.\n");

        // Shutdown gRPC writers safely
        {
            std::lock_guard<std::mutex> lock(stream_mutex);

            if (lidarWriter && !lidarStreamClosed) {
                std::fprintf(stderr, "[grpc] Shutting down lidar stream...\n");
                lidarWriter->WritesDone();
                Status status = lidarWriter->Finish();
                std::fprintf(stderr, "[grpc] lidar stream finished: %s\n", status.ok() ? "OK" : status.error_message().c_str());
            }

            if (gridWriter && !gridStreamClosed) {
                std::fprintf(stderr, "[grpc] Shutting down grid stream...\n");
                gridWriter->WritesDone();
                Status status = gridWriter->Finish();
                std::fprintf(stderr, "[grpc] grid stream finished: %s\n", status.ok() ? "OK" : status.error_message().c_str());
            }

            if (poseWriter && !poseStreamClosed) {
                std::fprintf(stderr, "[grpc] Shutting down pose stream...\n");
                poseWriter->WritesDone();
                Status status = poseWriter->Finish();
                std::fprintf(stderr, "[grpc] pose stream finished: %s\n", status.ok() ? "OK" : status.error_message().c_str());
            }
        }

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