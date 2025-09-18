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

bool TelemetryStream::SendIMU(const BNO055::Sample& sample)
{
    IMUSample imuSample;
    imuSample.set_w(sample.w);
    imuSample.set_x(sample.x);
    imuSample.set_y(sample.y);
    imuSample.set_z(sample.z);
    imuSample.set_ax(sample.ax);
    imuSample.set_ay(sample.ay);
    imuSample.set_az(sample.az);
    imuSample.set_gx(sample.gx);
    imuSample.set_gy(sample.gy);
    imuSample.set_gz(sample.gz);
    imuSample.set_mx(sample.mx);
    imuSample.set_my(sample.my);
    imuSample.set_mz(sample.mz);
    imuSample.set_ex(sample.eX);
    imuSample.set_ey(sample.eY);
    imuSample.set_ez(sample.eZ);
    imuSample.set_lx(sample.lx);
    imuSample.set_ly(sample.ly);
    imuSample.set_lz(sample.lz);
    imuSample.set_tx(sample.tx);
    imuSample.set_calib_sys(sample.calib_sys);
    imuSample.set_calib_gyro(sample.calib_gyro);
    imuSample.set_calib_accel(sample.calib_accel);
    imuSample.set_calib_mag(sample.calib_mag);
    imuSample.set_time_ns(sample.time_ns);

    if (!imuWriter->Write(imuSample)) {
        std::fprintf(stderr, "[grpc] imu stream closed by server\n");
        return false;
    }

    return true;
}