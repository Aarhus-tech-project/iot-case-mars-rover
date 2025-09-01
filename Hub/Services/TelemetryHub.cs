using Microsoft.AspNetCore.SignalR;

public interface ITelemetryClient
{
    Task Grid(GridMetaDto dto);
    Task Pose(PoseDto dto);
    Task Lidar(LidarDto dto);
}

public sealed class TelemetryHub : Hub<ITelemetryClient> { }

public record GridMetaDto(int Width, int Height, float CellSizeM, ulong Seq, ulong TMonoNs, string Url);
public record PoseDto(float XM, float YM, float Theta, ulong Seq);
public record LidarPointDto(float XM, float YM);
public record LidarDto(IReadOnlyList<LidarPointDto> Points, ulong Seq);