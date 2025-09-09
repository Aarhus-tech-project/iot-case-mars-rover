// CommandHub.cs
using HubServer.Services;
using Microsoft.AspNetCore.SignalR;

public interface ICommandClient
{
    Task CommandReply(string status, string message);
}

public class CommandHub : Hub<ICommandClient>
{
    private readonly IRoverCommandHub _hub;

    public CommandHub(IRoverCommandHub hub)
    {
        _hub = hub;
    }

    public async Task SendCommand(string text)
    {
        if (!_hub.IsConnected)
        {
            await Clients.Caller.CommandReply("error", "Rover not connected");
            return;
        }

        try
        {
            var reply = await _hub.SendAsync(text, TimeSpan.FromSeconds(5));
            await Clients.Caller.CommandReply(reply.Status, reply.Message);
        }
        catch (Exception ex)
        {
            await Clients.Caller.CommandReply("error", ex.Message);
        }
    }
}