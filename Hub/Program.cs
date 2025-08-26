using Grpc.Net.Compression;
using HubServer.Services;

var builder = WebApplication.CreateBuilder(args);
builder.Services.AddGrpc();

var app = builder.Build();

app.MapGrpcService<TelemetryService>();
app.MapGet("/", () => "HubServer gRPC is running (HTTP/2 on 50051)");

app.Run();