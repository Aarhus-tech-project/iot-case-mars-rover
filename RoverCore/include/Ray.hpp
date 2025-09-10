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
};