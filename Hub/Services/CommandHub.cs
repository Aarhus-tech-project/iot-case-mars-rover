// CommandHub.cs
using HubServer.Services;
using Microsoft.AspNetCore.SignalR;
using Hub.Data;
using Hub.Models;

public interface ICommandClient
{
    Task CommandReply(string status, string message);
}

public class CommandHub : Hub<ICommandClient>
{
    private readonly IRoverCommandHub _hub;
    private readonly HubDbContext _db;

    public CommandHub(IRoverCommandHub hub, HubDbContext db)
    {
        _hub = hub;
        _db = db;
    }

    public async Task SendCommand(string text)
    {
        var command = new CommandMessage
        {
            CommandText = text,
            StatusId = 1, // Pending
            CommandSentAt = DateTime.UtcNow
        };

        _db.CommandMessages.Add(command);
        await _db.SaveChangesAsync();

        if (!_hub.IsConnected)
        {
            command.StatusId = 4; // Failed
            command.ReplyText = "Rover not connected";
            await _db.SaveChangesAsync();
            
            await Clients.Caller.CommandReply("error", "Rover not connected");
            return;
        }

        try
        {
            // Mark as Sent
            command.StatusId = 2; // Sent
            await _db.SaveChangesAsync();

            // Send to rover and wait for reply
            var reply = await _hub.SendAsync(text, TimeSpan.FromSeconds(5));

            // Update with reply
            command.ReplyText = reply.Message;
            command.ReplyReceivedAt = DateTime.UtcNow;
            command.StatusId = 3; // Received

            await _db.SaveChangesAsync();

            await Clients.Caller.CommandReply("received", reply.Message);
        }
        catch (Exception ex)
        {
            command.StatusId = 4; // Failed
            command.ReplyText = ex.Message;
            await _db.SaveChangesAsync();

            await Clients.Caller.CommandReply("error", ex.Message);
        }
    }
}