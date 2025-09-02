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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// radians → centidegrees (0..36000 wrap optional)
constexpr float RAD2CDEG = 18000.0f / float(M_PI);

// If your lidar’s 0° isn’t straight ahead, adjust this (radians).

int main() {
    /*
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
    */

    OccupancyGrid<GRID_WIDTH, GRID_HEIGHT> grid(GRID_CELL_SIZE_M);
    MonteCarloLocalization<GRID_WIDTH, GRID_HEIGHT> mcl(grid, MLC_NUM_PARTICLES);

    // grpc setup
    auto channel = grpc::CreateChannel(HUB_ADDRESS, grpc::InsecureChannelCredentials());
    std::unique_ptr<Telemetry::Stub> stub = Telemetry::NewStub(channel);

    grpc::ClientContext gridCtx;
    rover::v1::Ack gridAck;
    auto gridWriter = stub->PublishGrid(&gridCtx, &gridAck);

    grpc::ClientContext poseCtx;
    rover::v1::Ack poseAck;
    auto poseWriter = stub->PublishPose(&poseCtx, &poseAck);

    grpc::ClientContext lidarCtx;
    rover::v1::Ack lidarAck;
    auto lidarWriter = stub->PublishLidar(&lidarCtx, &lidarAck);

    // Lidar and SLAM setup
    float rover_x_m = GRID_WIDTH * GRID_CELL_SIZE_M / 2;
    float rover_y_m = GRID_HEIGHT * GRID_CELL_SIZE_M / 2;
    float rover_theta = M_PI_2;

    std::signal(SIGINT, on_sigint);

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
        }

        Lidar lidar = { angle_cdeg, dist_mm, intensity, t_ns };
        backBuffer.push_back(lidar);

        last_angle_cdeg = angle_cdeg;
    };

    auto last_push = std::chrono::steady_clock::now();
    uint64_t last_ts = 0;
    uint32_t stamp_count = 0;
    uint32_t WARMUP_STAMPS = 50;
    while (g_run.load()) {
        lr.pump(cb, 10); 

        auto now = std::chrono::steady_clock::now();
        if (now - last_push >= std::chrono::seconds(1)) {
            
            if (!buffer.empty()) {

                if (stamp_count >= WARMUP_STAMPS) {
                    uint64_t t_now_ns = buffer.back().time_ns;
                    float dt = (last_ts == 0) ? 0.1f : float(t_now_ns - last_ts) * 1e-9f;
                    if (dt <= 0) dt = 1e-3f;

                    // ---------- MCL FIRST: get pose estimate ----------
                    // No odometry yet? Pass nullopt. If you have odom deltas, pass them here.
                    mcl.iterate(buffer, std::nullopt, dt);
                    auto est = mcl.estimateCached();
                    rover_x_m   = est.x;
                    rover_y_m   = est.y;
                    rover_theta = est.theta;

                    last_ts = t_now_ns;

                    // Do some debugging output
                    std::fprintf(stderr, "[slam] pose: x=%.2fm y=%.2fm θ=%.2f° neff=%.2f%% particles=%u\n",
                                 rover_x_m, rover_y_m, rover_theta * 180.f / float(M_PI),
                                 mcl.neffFraction() * 100.f, unsigned(MLC_NUM_PARTICLES));
                }
                else {
                    std::fprintf(stderr, "[slam] warming up MCL (%u/%u)\n", stamp_count, WARMUP_STAMPS);
                }

                LidarScan scan;
                for (Lidar& lidar : buffer) {
                    Ray ray(rover_x_m, rover_y_m, lidar.angle_cdeg / 100.0f, lidar.distance_mm, lidar.time_ns);

                    grid.populateRayOnGrid(ray);   

                    auto* p = scan.add_points();
                    p->set_x_m(ray.point_x_m);
                    p->set_y_m(ray.point_y_m);
                }

                stamp_count++;

                if (!lidarWriter->Write(scan)) {
                    std::fprintf(stderr, "[grpc] stream closed by server during Write()\n");
                    break;
                }
            }

            // Push telemetry over GRPC
            GridFrame gridFrame;
            gridFrame.set_width(GRID_WIDTH);
            gridFrame.set_height(GRID_HEIGHT);
            gridFrame.set_cell_size_m(GRID_CELL_SIZE_M);
            gridFrame.set_data(reinterpret_cast<const char*>(grid.data()), grid.size());

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
        }
    }

    gridWriter->WritesDone();
    gridWriter->Finish();
    poseWriter->WritesDone();
    poseWriter->Finish();
    lidarWriter->WritesDone();
    lidarWriter->Finish();
    lr.close();
    return 0;
}