using Grpc.Core;
using Rover.V1;

namespace HubServer.Services;

public sealed class TelemetryService : Telemetry.TelemetryBase
{
    public override async Task<Ack> PublishSamples(
        IAsyncStreamReader<Sample> requestStream,
        ServerCallContext context)
    {
        ulong count = 0;
        var sw = System.Diagnostics.Stopwatch.StartNew();

        await foreach (var s in requestStream.ReadAllAsync(context.CancellationToken))
        {
            Console.WriteLine($"angle={s.AngleCdeg/100.0:F2}°  dist={s.DistanceMm}mm  I={s.Intensity}");
        }

        sw.Stop();
        var secs = Math.Max(1e-6, sw.Elapsed.TotalSeconds);
        Console.WriteLine($"[hub] stream closed: {count} samples in {secs:F2}s  (~{count/secs:F0}/s)");

        return new Ack { Received = count };
    }
}