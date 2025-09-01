#pragma once
#include <array>
#include <cstdint>
#include <cmath>
#include <utility>
#include <type_traits>
#include "Ray.h"

template <size_t WIDTH, size_t HEIGHT>
class OccupancyGrid {
public:
    static_assert(WIDTH > 0 && HEIGHT > 0, "Grid dimensions must be positive");
    static constexpr size_t width  = WIDTH;
    static constexpr size_t height = HEIGHT;
    using value_type = uint8_t;

    explicit OccupancyGrid(float cell_size_meters) noexcept
        : cell_size_m_(cell_size_meters), data_{} {}

    // -------- queries --------
    constexpr float cellSizeM() const noexcept { return cell_size_m_; }
    constexpr size_t size() const noexcept { return WIDTH * HEIGHT; }
    constexpr size_t stride() const noexcept { return WIDTH; }

    // bounds
    static constexpr bool inBounds(int x, int y) noexcept {
        return x >= 0 && y >= 0 && x < int(WIDTH) && y < int(HEIGHT);
    }
    static constexpr bool inBounds(size_t x, size_t y) noexcept {
        return x < WIDTH && y < HEIGHT;
    }

    // read-only access
    const value_type& at(size_t x, size_t y) const noexcept { return data_[y * WIDTH + x]; }
    const value_type* data() const noexcept { return data_.data(); }

    // mutable access (explicit)
    value_type& at(size_t x, size_t y) noexcept { return data_[y * WIDTH + x]; }
    value_type* data() noexcept { return data_.data(); }

    void clear() noexcept { data_.fill(0); }
    void fill(value_type v) noexcept { data_.fill(v); }

    // world <-> grid
    std::pair<int,int> worldToGrid(float x_m, float y_m) const noexcept {
        // floor is robust for negatives; use trunc if you know coords are non-negative
        int gx = int(std::floor(double(x_m) / double(cell_size_m_)));
        int gy = int(std::floor(double(y_m) / double(cell_size_m_)));
        return { gx, gy };
    }
    std::pair<float,float> gridToWorld(int gx, int gy) const noexcept {
        return { (gx + 0.5f) * cell_size_m_, (gy + 0.5f) * cell_size_m_ };
    }

    // -------- fast ray update: clipped + pointer-walk, saturating updates --------
    void populateRayOnGrid(const Ray& ray) noexcept {
        // Convert to grid coords
        auto [gx, gy]   = worldToGrid(ray.point_x_m,  ray.point_y_m);   // hit
        auto [rgx, rgy] = worldToGrid(ray.origin_x_m, ray.origin_y_m);  // origin

        // Clip once; if fully outside, nothing to do
        int x0 = rgx, y0 = rgy, x1 = gx, y1 = gy;
        if (!clipLineToGrid(x0, y0, x1, y1)) return;

        // Bresenham on the clipped segment using pointer steps
        int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy;

        // Start pointer at (x0,y0)
        value_type* p = data_.data() + size_t(y0) * WIDTH + x0;
        const int stepX = sx;
        const int stepY = sy * int(WIDTH);

        while (true) {
            // free space: subtract 40 with saturation
            *p = sat_sub(*p, 40);

            if (x0 == x1 && y0 == y1) break;
            int e2 = err << 1;
            if (e2 >= dy) { err += dy; x0 += sx; p += stepX; }
            if (e2 <= dx) { err += dx; y0 += sy; p += stepY; }
        }

        // Occupied hit cell: +80 with saturation (prefer original hit if inside)
        if (inBounds(gx, gy)) {
            value_type& h = at(size_t(gx), size_t(gy));
            h = sat_add(h, 80);
        } else {
            value_type& h = at(size_t(x1), size_t(y1));
            h = sat_add(h, 80);
        }
    }

private:
    // Saturating ops
    static inline value_type sat_add(value_type v, int add) noexcept {
        int s = int(v) + add;
        return value_type(s > 255 ? 255 : (s < 0 ? 0 : s));
    }
    static inline value_type sat_sub(value_type v, int sub) noexcept {
        int s = int(v) - sub;
        return value_type(s < 0 ? 0 : s);
    }

    // Cohen–Sutherland clip to [0..W-1]x[0..H-1]
    static inline bool clipLineToGrid(int& x0, int& y0, int& x1, int& y1) noexcept {
        auto out = [](int x, int y) noexcept {
            int c = 0;
            if (x < 0)             c |= 1;
            else if (x >= int(WIDTH))  c |= 2;
            if (y < 0)             c |= 4;
            else if (y >= int(HEIGHT)) c |= 8;
            return c;
        };
        int c0 = out(x0,y0), c1 = out(x1,y1);
        while (true) {
            if (!(c0|c1)) return true;   // both inside
            if (c0 & c1)  return false;  // trivially outside

            int co = c0 ? c0 : c1;
            long long x=0, y=0;
            if ((co & 1) || (co & 2)) { // left/right
                int x_edge = (co & 1) ? 0 : int(WIDTH) - 1;
                y = y0 + (long long)(y1 - y0) * (x_edge - x0) / (x1 - x0);
                x = x_edge;
            } else {                    // bottom/top
                int y_edge = (co & 4) ? 0 : int(HEIGHT) - 1;
                x = x0 + (long long)(x1 - x0) * (y_edge - y0) / (y1 - y0);
                y = y_edge;
            }
            if (co == c0) { x0 = int(x); y0 = int(y); c0 = out(x0,y0); }
            else           { x1 = int(x); y1 = int(y); c1 = out(x1,y1); }
        }
    }

    float cell_size_m_;
    std::array<value_type, WIDTH * HEIGHT> data_;
};