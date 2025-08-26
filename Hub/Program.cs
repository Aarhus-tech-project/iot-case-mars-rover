// Program.cs
using HubServer.Services;
using Microsoft.AspNetCore.Server.Kestrel.Core;

var builder = WebApplication.CreateBuilder(args);
builder.Services.AddSingleton<GridState>();
builder.Services.AddGrpc();

var app = builder.Build();
app.MapGrpcService<TelemetryService>();

app.MapGet("/map.png", (GridState state) =>
{
    var png = state.EncodePng(flipY: true);
    return png.Length == 0
        ? Results.NotFound("no grid yet")
        : Results.File(png, "image/png");
});

app.MapGet("/", () => "HubServer running");
app.Run();