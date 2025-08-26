using Grpc.Core;
using Rover.V1;

namespace HubServer.Services;

public sealed class TelemetryService : Telemetry.TelemetryBase
{
    public override async Task<Ack> PublishGrid(IAsyncStreamReader<GridFrame> requestStream, ServerCallContext context)
    {
        ulong n = 0;
        await foreach (var m in requestStream.ReadAllAsync())
        {
            int w = (int)m.Width, h = (int)m.Height;
            var bytes = m.Data.ToByteArray(); // length should be w*h
            // Example: copy into 2D byte[,] grid if you want
            var grid = new byte[h, w];
            for (int y = 0; y < h; y++)
            {
                Buffer.BlockCopy(bytes, y * w, grid, y * w, w);
            }

            // TODO: store/display; m.CellSizeM gives resolution
            n++;
        }

        return new Ack { Received = n };
    }
}