// CommandStreamClient.h
#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <cstdio>

#include <grpcpp/grpcpp.h>
#include "telemetry.grpc.pb.h"  // generated from your proto

// Proto (server POV):
// service CommandLine {s
//   // Rover opens this call (client).
//   // Rover -> Server: TextReply   (request stream)
//   // Server -> Rover: TextCommand (response stream)
//   rpc Stream (stream TextReply) returns (stream TextCommand);
// }
//
// TextCommand: string cmd_id, string text, uint64 t_mono_ns (optional)
// TextReply:   string cmd_id, string status, string message, uint64 t_mono_ns

class CommandStreamClient {
public:
    explicit CommandStreamClient(std::shared_ptr<grpc::Channel> channel,
                                 std::string rover_id = "rover-01")
        : channel_(std::move(channel)), rover_id_(std::move(rover_id)) {}

    ~CommandStreamClient() { Stop(); }

    void Start() {
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true)) return;
        worker_ = std::thread([this]{ loop(); });
    }

    void Stop() {
        bool expected = true;
        if (!running_.compare_exchange_strong(expected, false)) return;
        cancel_.store(true);
        if (worker_.joinable()) worker_.join();
        cancel_.store(false);
    }

private:
    static uint64_t mono_ns() {
        using namespace std::chrono;
        return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    }

    void loop() {
        using namespace std::chrono_literals;
        rover::v1::CommandLine::Stub stub(channel_);

        int backoff_ms = 250;
        const int backoff_cap = 5000;

        while (running_.load()) {
            grpc::ClientContext ctx;
            ctx.AddMetadata("x-rover-id", rover_id_);      // optional identifier
            ctx.set_wait_for_ready(true);                  // wait for server

            auto stream = stub.Stream(&ctx);               // ClientReaderWriter<TextReply, TextCommand>
            if (!stream) {
                std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
                backoff_ms = std::min(backoff_ms * 2, backoff_cap);
                continue;
            }

            std::puts("[CMD] stream opened");
            backoff_ms = 250;

            rover::v1::TextCommand cmd;
            while (running_.load()) {
                if (!stream->Read(&cmd)) {
                    std::puts("[CMD] stream read closed by server");
                    break;
                }

                const std::string text = cmd.text();

                // Print the received command
                std::printf("[CMD] text=\"%s\"\n", text.c_str());

                // Send a dumb default reply
                rover::v1::TextReply r;
                r.set_status("OK");
                r.set_message("ack");
                r.set_t_mono_ns(mono_ns());

                if (!stream->Write(r)) {
                    std::puts("[CMD] stream write failed (server closed?)");
                    break;
                }
            }

            grpc::Status s = stream->Finish();
            std::printf("[CMD] stream finished: %s\n", s.ok() ? "OK" : s.error_message().c_str());

            if (!running_.load()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
            backoff_ms = std::min(backoff_ms * 2, backoff_cap);
        }

        std::puts("[CMD] loop exit");
    }

private:
    std::shared_ptr<grpc::Channel> channel_;
    std::string rover_id_;
    std::atomic<bool> running_{false};
    std::atomic<bool> cancel_{false};
    std::thread worker_;
};