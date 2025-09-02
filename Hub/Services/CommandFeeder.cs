using HubServer.Services;

public sealed class ConsoleFeeder : BackgroundService
{
    private readonly IRoverCommandHub _hub;
    public ConsoleFeeder(IRoverCommandHub hub) => _hub = hub;

    protected override async Task ExecuteAsync(CancellationToken ct)
    {
        Console.WriteLine("Type commands to send to rover (1:1):");
        while (!ct.IsCancellationRequested)
        {
            var line = await Task.Run(Console.ReadLine, ct);
            if (string.IsNullOrWhiteSpace(line)) continue;

            try
            {
                var reply = await _hub.SendAsync(line.Trim(), TimeSpan.FromSeconds(5), ct);
                Console.WriteLine($"[CMD] final: {reply.Status} - {reply.Message}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[CMD] error: {ex.Message}");
            }
        }
    }
}