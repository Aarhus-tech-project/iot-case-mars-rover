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

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

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
    MonteCarloLocalization<GRID_WIDTH, GRID_HEIGHT> mcl(grid);

    // grpc setup
    grpc::ChannelArguments args;
    args.SetInt(GRPC_ARG_KEEPALIVE_TIME_MS, 30000);
    args.SetInt(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 10000);
    args.SetInt(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);

    auto channel = grpc::CreateCustomChannel(HUB_ADDRESS, grpc::InsecureChannelCredentials(), args);

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
    float rover_rot_deg = 0;

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
    bool first = true;
    while (g_run.load()) {
        lr.pump(cb, 10); 

        auto now = std::chrono::steady_clock::now();
        if (now - last_push >= std::chrono::milliseconds(1000)) {
            
            if (!buffer.empty()) {
                if (first) {
                    first = false;

                    OrientationEstimate oe = EstimateHeadingFromScan(buffer, 3.0f, 0.30f, 0.02f, 12, LIDAR_MAX_MM);
                    rover_rot_deg = oe.heading_up_deg;
                    std::printf("[slam] initial heading %.1f° (used %d segments)\n", rover_rot_deg, oe.used_segments);
                }

                LidarScan scan;
                for (Lidar& lidar : buffer) {
                    Ray ray(rover_x_m, rover_y_m, rover_rot_deg + lidar.angle_cdeg / 100.0f, lidar.distance_mm, lidar.time_ns);

                    grid.populateRayOnGrid(ray);   

                    auto* p = scan.add_points();
                    p->set_x_m(ray.point_x_m);
                    p->set_y_m(ray.point_y_m);
                }

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
            pose2D.set_rot_deg(rover_rot_deg);

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