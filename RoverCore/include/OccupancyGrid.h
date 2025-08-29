#pragma once

#include <array>
#include <cstdint>
#include <utility>

#include "Ray.h"

template <size_t WIDTH, size_t HEIGHT>
struct OccupancyGrid {
    static constexpr size_t width  = WIDTH;
    static constexpr size_t height = HEIGHT;

    float cell_size_m;
    std::array<uint8_t, WIDTH * HEIGHT> data{};

    explicit OccupancyGrid(float cell_size_meters) : cell_size_m(cell_size_meters) {}

    bool inBounds(int x, int y) const {
        return x >= 0 && y >= 0 && x < static_cast<int>(WIDTH) && y < static_cast<int>(HEIGHT);
    }
    bool inBounds(size_t x, size_t y) const {
        return x < WIDTH && y < HEIGHT;
    }

    uint8_t& at(size_t x, size_t y) { return data[y * WIDTH + x]; }
    const uint8_t& at(size_t x, size_t y) const { return data[y * WIDTH + x]; }

    std::pair<int,int> worldToGrid(float x_m, float y_m) const {
        return { static_cast<int>(x_m / cell_size_m),
                 static_cast<int>(y_m / cell_size_m) };
    }
    std::pair<float,float> gridToWorld(int gx, int gy) const {
        return { (gx + 0.5f) * cell_size_m, (gy + 0.5f) * cell_size_m };
    }

    void populateRayOnGrid(const Ray& ray) {
        auto [gx, gy] = worldToGrid(ray.point_x_m, ray.point_y_m);
        if (inBounds(gx, gy)) at(gx, gy) += 80;

        auto [rgx, rgy] = worldToGrid(ray.origin_x_m, ray.origin_y_m);
        int dx = std::abs(gx - rgx), sx = rgx < gx ? 1 : -1;
        int dy = -std::abs(gy - rgy), sy = rgy < gy ? 1 : -1;
        int err = dx + dy, e2;
        while (true) {
            if (inBounds(rgx, rgy)) {
                uint8_t& v = at(rgx, rgy);
                if (v > 0) v -= 20;
            }
            if (rgx == gx && rgy == gy) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; rgx += sx; }
            if (e2 <= dx) { err += dx; rgy += sy; }
        }
    }
};