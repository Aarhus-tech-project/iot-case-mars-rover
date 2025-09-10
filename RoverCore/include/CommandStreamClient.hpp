#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <sstream>
#include <iostream>
#include <cstdio>

#include <grpcpp/grpcpp.h>
#include "telemetry.grpc.pb.h"
#include "Motors.hpp"

class CommandStreamClient {
private:
    std::shared_ptr<grpc::Channel> channel_;
    Motors* motors_;
    std::string rover_id_;
    std::atomic<bool> running_{false};
    std::atomic<bool> cancel_{false};
    std::thread worker_;

    static uint64_t mono_ns();

    void loop();
    rover::v1::TextReply handleCommand(const std::string& text);

    void moveDistance(int cm, bool forward);
    void rotateAngle(int deg);

public:
    CommandStreamClient(std::shared_ptr<grpc::Channel> channel, Motors* motors, std::string rover_id = "rover-01");
    ~CommandStreamClient();

    void Start();
    void Stop();
};
