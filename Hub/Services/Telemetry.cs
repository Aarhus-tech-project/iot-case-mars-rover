// TelemetryService.cs
using Grpc.Core;
using Microsoft.AspNetCore.SignalR;
using Rover.V1;

namespace HubServer.Services;

public sealed class TelemetryService : Telemetry.TelemetryBase
{
    private readonly GridState _state;
    private readonly IHubContext<TelemetryHub, ITelemetryClient> _hub;

    public TelemetryService(GridState state, IHubContext<TelemetryHub, ITelemetryClient> hub)
    { _state = state; _hub = hub; }

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
            await _hub.Clients.All.Pose(new PoseDto(m.XM, m.YM, m.Theta, n));
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
}