#include "CommandStreamClient.hpp"

CommandStreamClient::CommandStreamClient(std::shared_ptr<grpc::Channel> channel,
                    Motors* motors,
                    std::string rover_id)
    : channel_(std::move(channel)), motors_(motors), rover_id_(std::move(rover_id)) {}

CommandStreamClient::~CommandStreamClient() { Stop(); }

void CommandStreamClient::Start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) return;
    worker_ = std::thread([this]{ loop(); });
}

void CommandStreamClient::Stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) return;
    cancel_.store(true);
    if (worker_.joinable()) worker_.join();
    cancel_.store(false);
}

uint64_t CommandStreamClient::mono_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

void CommandStreamClient::loop() {
    using namespace std::chrono_literals;
    rover::v1::CommandLine::Stub stub(channel_);

    int backoff_ms = 250;
    const int backoff_cap = 5000;

    while (running_.load()) {
        grpc::ClientContext ctx;
        ctx.AddMetadata("x-rover-id", rover_id_);
        ctx.set_wait_for_ready(true);

        auto stream = stub.Stream(&ctx); // ClientReaderWriter<TextReply, TextCommand>
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
            std::printf("[CMD] received: \"%s\"\n", text.c_str());

            rover::v1::TextReply reply = handleCommand(text);
            reply.set_t_mono_ns(mono_ns());

            if (!stream->Write(reply)) {
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

rover::v1::TextReply CommandStreamClient::handleCommand(const std::string& text) {
    rover::v1::TextReply reply;
    reply.set_status("OK");

    std::istringstream iss(text);
    std::string cmd;
    int value = 0;
    iss >> cmd >> value;

    // Convert command to uppercase or lowercase (e.g., lowercase here)
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), [](unsigned char c) {
        return std::tolower(c);
    });

    // Now compare with lowercase versions of commands
    if (cmd == "forward") {
        moveDistance(value, true);
        reply.set_message("Moved " + std::to_string(value) + " cm forward");
    } else if (cmd == "reverse") {
        moveDistance(value, false);
        reply.set_message("Moved " + std::to_string(value) + " cm backward");
    } else if (cmd == "rot") {
        rotateAngle(value);
        reply.set_message("Rotated " + std::to_string(value) + " degrees");
    } else if (cmd == "stop") {
        motors_->stop();
        reply.set_message("Stopped");
    } else {
        reply.set_status("ERROR");
        reply.set_message("Unknown command: " + text);
    }

    return reply;
}

void CommandStreamClient::moveDistance(int cm, bool forward) {
    if (forward) motors_->forward();
    else motors_->reverse();

    std::this_thread::sleep_for(std::chrono::milliseconds(cm * 100)); // crude timing
    motors_->stop();
}

void CommandStreamClient::rotateAngle(int deg) {
    int ms = std::abs(deg) * 10; // crude timing
    if (deg > 0) motors_->left();
    else motors_->right();
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    motors_->stop();
}