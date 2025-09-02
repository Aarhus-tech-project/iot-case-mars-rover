// CommandLineServer.cs
using System;
using System.Threading;
using System.Threading.Tasks;
using Grpc.Core;
using Rover.V1; // generated from your .proto

namespace HubServer.Services
{
    /// <summary>
    /// Singleton hub that lets the rest of your server code do:
    ///   await _hub.SendAsync("ROT 90");
    /// It only supports ONE outstanding command at a time (strict 1:1).
    /// </summary>
    public interface IRoverCommandHub
    {
        bool IsConnected { get; }
        Task<TextReply> SendAsync(string text, TimeSpan? timeout = null, CancellationToken ct = default);
        // Internal: bound by the gRPC service when a rover connects.
        void Attach(Func<TextCommand, CancellationToken, Task<TextReply>> sendCore, string peerInfo);
        void Detach(string reason);
    }

    public sealed class RoverCommandHub : IRoverCommandHub
    {
        private readonly SemaphoreSlim _serial = new(1, 1); // enforce 1-at-a-time
        private Func<TextCommand, CancellationToken, Task<TextReply>>? _sendCore;
        private volatile string _peer = "";

        public bool IsConnected => _sendCore is not null;

        public void Attach(Func<TextCommand, CancellationToken, Task<TextReply>> sendCore, string peerInfo)
        {
            _sendCore = sendCore ?? throw new ArgumentNullException(nameof(sendCore));
            _peer = peerInfo;
            Console.WriteLine($"[CMD HUB] attached to rover: {_peer}");
        }

        public void Detach(string reason)
        {
            _sendCore = null;
            Console.WriteLine($"[CMD HUB] detached ({reason})");
        }

        public async Task<TextReply> SendAsync(string text, TimeSpan? timeout = null, CancellationToken ct = default)
        {
            var core = _sendCore ?? throw new InvalidOperationException("Rover not connected.");
            await _serial.WaitAsync(ct).ConfigureAwait(false);
            try
            {
                using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
                if (timeout is { } t) cts.CancelAfter(t);

                var cmd = new TextCommand
                {
                    Text     = text ?? string.Empty,
                    TMonoNs  = (ulong)DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1_000_000UL
                };
                // 1 command -> 1 reply
                return await core(cmd, cts.Token).ConfigureAwait(false);
            }
            finally
            {
                _serial.Release();
            }
        }
    }

    /// <summary>
    /// gRPC service implementation for the rover-initiated command stream.
    /// From server POV:
    ///   requestStream: TextReply (rover -> server)
    ///   responseStream: TextCommand (server -> rover)
    /// </summary>
    public sealed class CommandLineService : CommandLine.CommandLineBase
    {
        private readonly IRoverCommandHub _hub;
        public CommandLineService(IRoverCommandHub hub) => _hub = hub;

        public override async Task Stream(
            IAsyncStreamReader<TextReply> requestStream,
            IServerStreamWriter<TextCommand> responseStream,
            ServerCallContext context)
        {
            var peer = context.Peer; // or get a rover-id from metadata
            Console.WriteLine($"[CMD] rover connected: {peer}");

            // Bind the active stream to the hub. The delegate:
            //  - writes ONE TextCommand to rover
            //  - awaits ONE TextReply from rover
            //  - returns the reply
            _hub.Attach(async (cmd, ct) =>
            {
                // Send the command down the response stream
                await responseStream.WriteAsync(cmd).ConfigureAwait(false);
                Console.WriteLine($"[CMD] sent: {cmd.Text}");

                // Wait for exactly one reply from rover
                // MoveNext honors cancellation via ct
                if (await requestStream.MoveNext(ct).ConfigureAwait(false))
                {
                    var r = requestStream.Current;
                    Console.WriteLine($"[CMD] reply: {r.Status} - {r.Message}");
                    return r;
                }

                throw new RpcException(new Status(StatusCode.Unavailable, "Rover disconnected before reply."));
            }, peer);

            // Park here until the client cancels/disconnects.
            await WaitUntilCancelled(context.CancellationToken);

            _hub.Detach("rover disconnected");
            Console.WriteLine($"[CMD] rover disconnected: {peer}");
        }

        private static Task WaitUntilCancelled(CancellationToken ct)
        {
            var tcs = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
            ct.Register(() => tcs.TrySetResult());
            return tcs.Task;
        }
    }
}