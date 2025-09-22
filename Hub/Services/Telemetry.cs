// TelemetryService.cs
using Grpc.Core;
using Microsoft.AspNetCore.SignalR;
using Rover.V1;
using Hub.Models;
using Hub.Data;

namespace HubServer.Services;

public sealed class TelemetryService : Telemetry.TelemetryBase
{
    private readonly GridState _state;
    private readonly IHubContext<TelemetryHub, ITelemetryClient> _hub;
    private readonly HubDbContext _db;

    public TelemetryService(GridState state, IHubContext<TelemetryHub, ITelemetryClient> hub, HubDbContext db)
    { _state = state; _hub = hub; _db = db; }

    public override async Task<Ack> PublishGrid(IAsyncStreamReader<GridFrame> requestStream, ServerCallContext context)
    {
        ulong n = 0;
        await foreach (var m in requestStream.ReadAllAsync(context.CancellationToken))
        {
            var bytes = m.Data.ToByteArray();
            _state.Update((int)m.Width, (int)m.Height, m.CellSizeM, bytes);
            n++;

            Console.WriteLine($"[Grid] rx frame #{n} {m.Width}x{m.Height} cell={m.CellSizeM:F3} t={m.TMonoNs}");

            var url = $"/grid.png";
            await _hub.Clients.All.Grid(new GridMetaDto(
                (int)m.Width, (int)m.Height, m.CellSizeM, n, m.TMonoNs, url));

            Console.WriteLine($"[Grid] broadcast seq={n} url={url}");
        }
        return new Ack { Received = n };
    }

    public override async Task<Ack> PublishPose(IAsyncStreamReader<Pose2D> requestStream, ServerCallContext context)
    {
        ulong n = 0;
        await foreach (var m in requestStream.ReadAllAsync(context.CancellationToken))
        {
            n++;
            await _hub.Clients.All.Pose(new PoseDto(m.XM, m.YM, m.RotDeg, n));
        }
        return new Ack { Received = n };
    }

    public override async Task<Ack> PublishLidar(IAsyncStreamReader<LidarScan> requestStream, ServerCallContext context)
    {
        ulong n = 0;
        await foreach (var m in requestStream.ReadAllAsync(context.CancellationToken))
        {
            n++;
            var pts = new List<LidarPointDto>(m.Points.Count);
            foreach (var p in m.Points) pts.Add(new LidarPointDto(p.XM, p.YM));
            await _hub.Clients.All.Lidar(new LidarDto(pts, n));
        }
        return new Ack { Received = n };
    }

    public override async Task<Ack> PublishIMU(IAsyncStreamReader<IMUSample> requestStream, ServerCallContext context)
    {
        ulong n = 0;

        var lastSave = DateTime.UtcNow;
        var lastBroadcast = DateTime.UtcNow;

        await foreach (var m in requestStream.ReadAllAsync(context.CancellationToken))
        {
            n++;

            var entity = new ImuSample
            {
                ReceivedAt = DateTime.UtcNow,
                RoverTimeNs = m.TimeNs,

                QuaternionW = m.W,
                QuaternionX = m.X,
                QuaternionY = m.Y,
                QuaternionZ = m.Z,

                AccelX = m.Ax,
                AccelY = m.Ay,
                AccelZ = m.Az,

                GyroX = m.Gx,
                GyroY = m.Gy,
                GyroZ = m.Gz,

                MagX = m.Mx,
                MagY = m.My,
                MagZ = m.Mz,

                Heading = m.EX,
                Roll = m.EY,
                Pitch = m.EZ,

                LinearAccelX = m.Lx,
                LinearAccelY = m.Ly,
                LinearAccelZ = m.Lz,

                TemperatureC = m.Tx,

                CalibSys = m.CalibSys,
                CalibGyro = m.CalibGyro,
                CalibAccel = m.CalibAccel,
                CalibMag = m.CalibMag
            };

            var now = DateTime.UtcNow;

            // Save to DB
            if ((now - lastSave).TotalSeconds >= 30)
            {
                _db.ImuSamples.Add(entity);
                await _db.SaveChangesAsync();
                lastSave = now;
                Console.WriteLine($"[IMU] Saved sample at {now}");
            }

            // Broadcast to frontend
            if ((now - lastBroadcast).TotalSeconds >= 5)
            {
                var dto = new ImuDto(
                    m.TimeNs,
                    m.W, m.X, m.Y, m.Z,
                    m.Ax, m.Ay, m.Az,
                    m.Gx, m.Gy, m.Gz,
                    m.Mx, m.My, m.Mz,
                    m.EX, m.EY, m.EZ,
                    m.Lx, m.Ly, m.Lz,
                    m.Tx,
                    m.CalibSys, m.CalibGyro, m.CalibAccel, m.CalibMag
                );

                await _hub.Clients.All.Imu(dto);
                lastBroadcast = now;
                Console.WriteLine($"[IMU] Broadcast to clients at {now}");
            }
        }

        Console.WriteLine($"[IMU] Stream finished, received {n} samples");
        return new Ack { Received = n };
    }
}