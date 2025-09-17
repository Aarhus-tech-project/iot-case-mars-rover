#pragma once

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>

#include "BNO055.hpp"

class IMUPositionEstimation {
private:
    BNO055& imu;
    std::atomic<bool> running;
    std::thread workerThread;

    float bias_ax {0.f}, bias_ay {0.f};
    bool  bias_ready {false};
    float acc_deadband {0.03f};
    float vel_leak_per_s {0.15f}; 

    void updatePosition();

public:
    float x_m {0.f};
    float y_m {0.f};
    float rot_deg {0.f};

    float velocity_x_mps {0.f};
    float velocity_y_mps {0.f};
    float angular_dps {0.f};

    float update_interval_ms {50.f};

    IMUPositionEstimation(BNO055& imu);
    ~IMUPositionEstimation();

    void Start(float init_x_m = 0.f, float init_y_m = 0.f, float init_rot_deg = 0.f);
    void Stop();
};

IMUPositionEstimation::IMUPositionEstimation(BNO055& imu) : imu(imu), running(false) {
    imu.begin(true, BNO055::MODE_NDOF);
}

IMUPositionEstimation::~IMUPositionEstimation() {
    Stop();
    imu.close();
}

void IMUPositionEstimation::Start(float init_x_m, float init_y_m, float init_rot_deg) {
    if (running) return;
    running = true;

    x_m = init_x_m;
    y_m = init_y_m;
    rot_deg = init_rot_deg;

    workerThread = std::thread(&IMUPositionEstimation::updatePosition, this);
}

void IMUPositionEstimation::Stop() {
    if (!running) return;
    running = false;
    if (workerThread.joinable()) {
        workerThread.join();
    }
}

void IMUPositionEstimation::updatePosition() {
    using clock = std::chrono::steady_clock;
    auto t_prev = clock::now();

    // simple on-the-fly bias estimation (first ~1s)
    int   bias_n = 0;
    float bias_sum_x = 0.f, bias_sum_y = 0.f;

    // keep last yaw for angular rate
    float last_rot = rot_deg;

    while (running) {
        auto t_now = clock::now();
        float dt = std::chrono::duration<float>(t_now - t_prev).count();
        t_prev = t_now;
        if (dt <= 0.f) dt = update_interval_ms / 1000.f; // fallback

        BNO055::Vec3  linAcc{};
        BNO055::Euler euler{};
        if (imu.readLinearAccel(linAcc) && imu.readEuler(euler)) {
            // --- learn accel bias for ~1s assuming initial stillness ---
            if (!bias_ready) {
                bias_sum_x += linAcc.x;
                bias_sum_y += linAcc.y;
                if (++bias_n >= 40) { // ~40 * 25ms ≈ 1s
                    bias_ax = bias_sum_x / bias_n;
                    bias_ay = bias_sum_y / bias_n;
                    bias_ready = true;
                }
            }

            // subtract bias; work in m/s^2 (no cm scaling)
            float ax = linAcc.x - (bias_ready ? bias_ax : 0.f);
            float ay = linAcc.y - (bias_ready ? bias_ay : 0.f);

            // deadband small noise
            if (std::fabs(ax) < acc_deadband) ax = 0.f;
            if (std::fabs(ay) < acc_deadband) ay = 0.f;

            // rotate sensor-frame accel to world X/Y using heading (yaw)
            float yaw = euler.heading_deg * float(M_PI) / 180.f;
            float cy = std::cos(yaw), sy = std::sin(yaw);
            float ax_w =  cy * ax - sy * ay;
            float ay_w =  sy * ax + cy * ay;

            // integrate velocity (m/s) and position (m)
            // zero-velocity update if very quiet (both accel ~0 and tiny yaw change)
            float drot = euler.heading_deg - last_rot;
            if (drot > 180.f) drot -= 360.f; else if (drot < -180.f) drot += 360.f;
            float ang_rate = drot / dt;  // deg/s

            bool still = (std::fabs(ax_w) < 0.02f && std::fabs(ay_w) < 0.02f &&
                          std::fabs(ang_rate) < 1.0f);

            if (still) {
                // strong bleed when we believe we are stationary
                float strong = std::pow(1.f - 0.8f, dt);
                velocity_x_mps *= strong;
                velocity_y_mps *= strong;
            } else {
                velocity_x_mps += ax_w * dt;
                velocity_y_mps += ay_w * dt;
            }

            // gentle continuous leak to prevent long-term drift
            float leak = std::pow(1.f - vel_leak_per_s, dt);
            velocity_x_mps *= leak;
            velocity_y_mps *= leak;

            // positions
            x_m += velocity_x_mps * dt;
            y_m += velocity_y_mps * dt;

            // angular rate & heading
            angular_dps = ang_rate;
            rot_deg = euler.heading_deg;
            last_rot = rot_deg;
        } else {
            std::fprintf(stderr, "[imu] Warning: Failed to read IMU data\n");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(update_interval_ms)));
    }
}

