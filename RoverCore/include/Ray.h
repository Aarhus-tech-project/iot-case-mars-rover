#pragma once
#include <cmath>
#include <cstdint>

class Ray {
public:
    float origin_x_m;
    float origin_y_m;

    float angle_deg;
    float direction_x;
    float direction_y;

    float distance_mm;

    float point_x_m;
    float point_y_m;

    uint64_t time_ns;

    Ray(float _origin_x_m, float _origin_y_m, float _angle_deg, float _distance_mm, uint64_t _time_ns);
    ~Ray();
};

inline Ray::Ray(float _origin_x_m, float _origin_y_m, float _angle_deg, float _distance_mm, uint64_t _time_ns) {
    origin_x_m  = _origin_x_m;
    origin_y_m  = _origin_y_m;
    angle_deg   = _angle_deg;
    distance_mm = _distance_mm;
    time_ns     = _time_ns;

    // your original math: theta_math = 90 - lidar_angle
    float r_m = _distance_mm * 0.001f;
    float theta_math_deg = 90.0f - _angle_deg;
    float rad = theta_math_deg * float(M_PI) / 180.0f;

    direction_x = std::cos(rad);
    direction_y = std::sin(rad);

    point_x_m = _origin_x_m + r_m * direction_x;
    point_y_m = _origin_y_m + r_m * direction_y;
}

inline Ray::~Ray() {}