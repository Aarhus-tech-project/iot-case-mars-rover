#pragma once

#include <iostream>
#include <grpcpp/grpcpp.h>
#include "telemetry.grpc.pb.h"

#include "Config.hpp"
#include "OccupancyGrid.hpp"
#include "BNO055.hpp"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using rover::v1::Telemetry;
using rover::v1::GridFrame;
using rover::v1::Pose2D;
using rover::v1::LidarScan;
using rover::v1::IMUSample;
using rover::v1::Ack;

class TelemetryStream
{
private:
    std::unique_ptr<Telemetry::Stub> stub;

    std::unique_ptr<grpc::ClientContext> g_gridCtx;
    std::unique_ptr<grpc::ClientContext> g_poseCtx;
    std::unique_ptr<grpc::ClientContext> g_lidarCtx;
    std::unique_ptr<grpc::ClientContext> g_imuCtx;

    Ack gridAck, poseAck, lidarAck, imuAck;

    std::unique_ptr<grpc::ClientWriter<GridFrame>> gridWriter;
    std::unique_ptr<grpc::ClientWriter<Pose2D>> poseWriter;
    std::unique_ptr<grpc::ClientWriter<LidarScan>> lidarWriter;
    std::unique_ptr<grpc::ClientWriter<IMUSample>> imuWriter;

public:
    TelemetryStream(std::shared_ptr<grpc::Channel> channel);
    ~TelemetryStream();

    bool SendGrid(const OccupancyGrid<GRID_WIDTH, GRID_HEIGHT>& grid);
    bool SendPose(float x_m, float y_m, float rot_deg);
    bool SendLidar(const LidarScan& scan);
    bool SendIMU(const BNO055::Sample& sample);
};