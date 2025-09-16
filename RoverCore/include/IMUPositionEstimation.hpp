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

    void updatePosition();

public:
    float x_m {0.f};
    float y_m {0.f};
    float rot_deg {0.f};

    float velocity_x_mps {0.f};
    float velocity_y_mps {0.f};
    float velocity_mps {0.f};
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
    if (workerThread.joinable()) workerThread.join();
}

void IMUPositionEstimation::updatePosition() {
    while (running) {
        BNO055::Vec3 linAcc;
        BNO055::Euler euler;
        if (imu.readLinearAccel(linAcc) && imu.readEuler(euler)) {
            // Convert acceleration from m/s² to cm/s² for finer granularity
            float ax = linAcc.x * 100.f; // cm/s²
            float ay = linAcc.y * 100.f; // cm/s²

            // Update velocities (cm/s)
            velocity_x_mps += ax * (update_interval_ms / 1000.f);
            velocity_y_mps += ay * (update_interval_ms / 1000.f);

            // Compute overall velocity magnitude (m/s)
            velocity_mps = std::sqrt(velocity_x_mps * velocity_x_mps + velocity_y_mps * velocity_y_mps) / 100.f;

            // Update orientation
            float new_rot_deg = euler.heading_deg;
            float delta_rot = new_rot_deg - rot_deg;
            if (delta_rot > 180.f) delta_rot -= 360.f;
            else if (delta_rot < -180.f) delta_rot += 360.f;
            angular_dps = delta_rot / (update_interval_ms / 1000.f);
            rot_deg = new_rot_deg;

            // Update positions (m)
            x_m += (velocity_x_mps / 100.f) * (update_interval_ms / 1000.f);
            y_m += (velocity_y_mps / 100.f) * (update_interval_ms / 1000.f);
        } else {
            std::fprintf(stderr, "[imu] Warning: Failed to read IMU data\n");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(update_interval_ms)));
    }
}

