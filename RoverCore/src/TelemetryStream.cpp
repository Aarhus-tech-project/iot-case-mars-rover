#include "TelemetryStream.hpp"

TelemetryStream::TelemetryStream(std::shared_ptr<grpc::Channel> channel)
{
    stub = Telemetry::NewStub(channel);

    g_gridCtx = std::make_unique<grpc::ClientContext>();
    g_poseCtx = std::make_unique<grpc::ClientContext>();
    g_lidarCtx = std::make_unique<grpc::ClientContext>();
    
    gridWriter = stub->PublishGrid(g_gridCtx.get(), &gridAck);
    poseWriter = stub->PublishPose(g_poseCtx.get(), &poseAck);
    lidarWriter = stub->PublishLidar(g_lidarCtx.get(), &lidarAck);
}

TelemetryStream::~TelemetryStream()
{
    if (gridWriter) gridWriter->WritesDone();
    if (poseWriter) poseWriter->WritesDone();
    if (lidarWriter) lidarWriter->WritesDone();

    if (g_gridCtx) g_gridCtx->TryCancel();
    if (g_poseCtx) g_poseCtx->TryCancel();
    if (g_lidarCtx) g_lidarCtx->TryCancel();
}

bool TelemetryStream::SendGrid(const OccupancyGrid<GRID_WIDTH, GRID_HEIGHT>& grid)
{
    GridFrame gridFrame;
    gridFrame.set_width(GRID_WIDTH);
    gridFrame.set_height(GRID_HEIGHT);
    gridFrame.set_cell_size_m(GRID_CELL_SIZE_M);
    gridFrame.set_data(reinterpret_cast<const char*>(grid.data()), grid.size());

    if (!gridWriter->Write(gridFrame)) {
        std::fprintf(stderr, "[grpc] grid stream closed by server\n");
        return false;
    }

    return true;
}

bool TelemetryStream::SendPose(float x_m, float y_m, float rot_deg)
{
    Pose2D pose2D;
    pose2D.set_x_m(x_m);
    pose2D.set_y_m(y_m);
    pose2D.set_rot_deg(rot_deg);

    if (!poseWriter->Write(pose2D)) {
        std::fprintf(stderr, "[grpc] pose stream closed by server\n");
        return false;
    }

    return true;
}

bool TelemetryStream::SendLidar(const LidarScan& scan)
{
    if (!lidarWriter->Write(scan)) {
        std::fprintf(stderr, "[grpc] lidar stream closed by server\n");
        return false;
    }

    return true;
}