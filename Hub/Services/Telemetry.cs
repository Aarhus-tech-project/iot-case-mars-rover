using Grpc.Core;
using Rover.V1;

namespace HubServer.Services;

public sealed class TelemetryService : Telemetry.TelemetryBase
{
    private readonly GridState _state;
    public TelemetryService(GridState state) => _state = state;

    public override async Task<Ack> PublishGrid(IAsyncStreamReader<GridFrame> requestStream, ServerCallContext context)
    {
        ulong n = 0;
        await foreach (var m in requestStream.ReadAllAsync(context.CancellationToken))
        {
            var bytes = m.Data.ToByteArray(); // should be width*height
            _state.Update((int)m.Width, (int)m.Height, m.CellSizeM, bytes);
            n++;
        }
        return new Ack { Received = n };
    }
}