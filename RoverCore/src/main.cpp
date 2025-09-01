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
#include "telemetry.grpc.pb.h"

#include "Config.h"
#include "Lidar.h"
#include "OccupancyGrid.h"
#include "MonteCarloLocalization.h"
#include "Ray.h"
#include "Motors.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using rover::v1::Telemetry;
using rover::v1::GridFrame;
using rover::v1::Pose2D;
using rover::v1::LidarScan;
using rover::v1::Ack;

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

// -------- angle helpers --------
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
constexpr float DEG2RAD   = float(M_PI) / 180.0f;
constexpr float CDEG2RAD  = float(M_PI) / 18000.0f; // centidegree -> rad

// If your LIDAR’s 0° isn’t straight-ahead in rover frame, set this.
// Or put it in Config.h and include here.
constexpr float LIDAR_MOUNT_YAW_RAD = 0.0f; // e.g. +90° -> 90*DEG2RAD

int main() {
    using GridT = OccupancyGrid<GRID_WIDTH, GRID_HEIGHT>;
    GridT grid(GRID_CELL_SIZE_M);

    MonteCarloLocalization<GRID_WIDTH, GRID_HEIGHT> mcl(grid, MLC_NUM_PARTICLES);
    mcl.ray_step = 8;
    mcl.z_hit = 0.9f; mcl.z_rand = 0.1f;

    // gRPC
    auto channel = grpc::CreateChannel(HUB_ADDRESS, grpc::InsecureChannelCredentials());
    std::unique_ptr<Telemetry::Stub> stub = Telemetry::NewStub(channel);

    grpc::ClientContext gridCtx;  Ack gridAck;  auto gridWriter  = stub->PublishGrid(&gridCtx,  &gridAck);
    grpc::ClientContext poseCtx;  Ack poseAck;  auto poseWriter  = stub->PublishPose(&poseCtx,  &poseAck);
    grpc::ClientContext lidarCtx; Ack lidarAck; auto lidarWriter = stub->PublishLidar(&lidarCtx, &lidarAck);

    // World frame: +X right, +Y up. In that convention, 0 rad = +X (east), π/2 = +Y (north).
    float rover_x_m = GRID_WIDTH  * GRID_CELL_SIZE_M * 0.5f;
    float rover_y_m = GRID_HEIGHT * GRID_CELL_SIZE_M * 0.5f;
    float rover_theta = float(M_PI) * 0.5f; // facing "north"/up

    std::signal(SIGINT, on_sigint);

    LidarReader lr(LIDAR_SERIAL_PORT, LIDAR_SERIAL_BAUD, LIDAR_MAX_MM);
    if (!lr.open()) {
        std::fprintf(stderr, "[lidar] failed to open %s @%d\n", LIDAR_SERIAL_PORT, LIDAR_SERIAL_BAUD);
        return 1;
    }
    std::fprintf(stderr, "[lidar] reading %s @%d (max=%dmm)\n", LIDAR_SERIAL_PORT, LIDAR_SERIAL_BAUD, LIDAR_MAX_MM);

    // Full-rev buffering
    std::vector<Lidar> buffer, backBuffer;
    uint32_t last_angle_cdeg = 0;
    bool have_last = false;

    auto cb = [&](uint32_t angle_cdeg, uint32_t dist_mm, uint32_t intensity, uint64_t t_ns) {
        // Detect wrap-around -> new sweep ready
        if (have_last && angle_cdeg < last_angle_cdeg) {
            buffer = std::move(backBuffer);
            backBuffer.clear();
        }
        backBuffer.push_back({ angle_cdeg, dist_mm, intensity, t_ns });
        last_angle_cdeg = angle_cdeg;
        have_last = true;
    };

    auto last_push = std::chrono::steady_clock::now();
    uint64_t last_ts = 0;

    while (g_run.load()) {
        lr.pump(cb, 10);

        auto now = std::chrono::steady_clock::now();
        if (now - last_push >= std::chrono::milliseconds(100)) { // ~10 Hz
            if (!buffer.empty()) {
                // dt from LIDAR timestamps
                const uint64_t t_now_ns = buffer.back().time_ns;
                float dt = (last_ts == 0) ? 0.1f : float(t_now_ns - last_ts) * 1e-9f;
                if (dt <= 0) dt = 1e-3f;

                // --- 1) MCL (use lidar in sensor frame; class handles rotation per particle) ---
                mcl.iterate(buffer, std::nullopt, dt);
                const auto est = mcl.estimateCached();
                rover_x_m = est.x; rover_y_m = est.y; rover_theta = est.theta;

                // --- 2) Populate grid using WORLD angle for each beam ---
                LidarScan scan;
                for (const Lidar& li : buffer) {
                    // Convert beam angle to world:
                    //   world_angle = rover_theta + mount_yaw + lidar_angle(rad)
                    const float beam_rad_world = rover_theta + LIDAR_MOUNT_YAW_RAD + (li.angle_cdeg * CDEG2RAD);

                    // If your Ray expects CENTIDEGREES instead of radians, do:
                    //   const float beam_cdeg_world = (rover_theta + LIDAR_MOUNT_YAW_RAD) * (18000.0f/float(M_PI)) + li.angle_cdeg;
                    //   Ray ray(rover_x_m, rover_y_m, beam_cdeg_world, li.distance_mm, li.time_ns);

                    Ray ray(rover_x_m, rover_y_m, beam_rad_world, li.distance_mm, li.time_ns);
                    grid.populateRayOnGrid(ray);

                    auto* p = scan.add_points();
                    p->set_x_m(ray.point_x_m);
                    p->set_y_m(ray.point_y_m);
                }

                // Publish
                if (!lidarWriter->Write(scan)) { std::fprintf(stderr, "[grpc] LIDAR stream closed\n"); break; }

                GridFrame gf;
                gf.set_width(GRID_WIDTH);
                gf.set_height(GRID_HEIGHT);
                gf.set_cell_size_m(GRID_CELL_SIZE_M);
                gf.set_data(reinterpret_cast<const char*>(grid.data()), grid.size());
                if (!gridWriter->Write(gf)) { std::fprintf(stderr, "[grpc] GRID stream closed\n"); break; }

                Pose2D pose2D;
                pose2D.set_x_m(rover_x_m);
                pose2D.set_y_m(rover_y_m);
                pose2D.set_theta(rover_theta);
                if (!poseWriter->Write(pose2D)) { std::fprintf(stderr, "[grpc] POSE stream closed\n"); break; }

                last_ts = t_now_ns;
            }

            last_push = now;
            std::fprintf(stderr, "[grpc] pushed snapshot (%ux%u), pts=%zu\n",
                         GRID_WIDTH, GRID_HEIGHT, buffer.size());
        }
    }

    gridWriter->WritesDone();  gridWriter->Finish();
    poseWriter->WritesDone();  poseWriter->Finish();
    lidarWriter->WritesDone(); lidarWriter->Finish();
    lr.close();
    return 0;
}