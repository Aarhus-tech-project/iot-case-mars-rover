using HubServer.Services;

var builder = WebApplication.CreateBuilder(args);

builder.Services.AddGrpc();
builder.Services.AddSignalR();
builder.Services.AddSingleton<GridState>();

builder.Services.AddCors(o => o.AddDefaultPolicy(p =>
    p.WithOrigins("http://localhost:5173")
     .AllowAnyHeader()
     .AllowAnyMethod()
     .AllowCredentials()));

var app = builder.Build();

app.UseCors();

app.MapGrpcService<TelemetryService>();
app.MapHub<TelemetryHub>("/telemetryHub");

app.MapGet("/grid.png", (GridState state) =>
{
    var png = state.EncodePng(flipY: true);
    return png.Length == 0 ? Results.NotFound("no grid yet")
                           : Results.File(png, "image/png");
});

app.MapGet("/", () => "HubServer running");
app.Run();