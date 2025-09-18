#pragma once
#include <array>
#include <cstdint>
#include <cmath>
#include <utility>
#include <type_traits>
#include "Ray.hpp"

struct RayResult {
    float distance_m;      // distance traveled (clamped to max_range_m)
    int   cell_x, cell_y;  // last cell entered (if relevant)
    bool  hit;             // true if hit an occupied cell
    bool  out_of_bounds;   // true if left the grid
    bool  started_oob;     // true if start was outside grid
};

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
    
    void populateRayOnGrid(const Ray& ray) noexcept {
        // Convert to grid coords
        auto [gx, gy]   = worldToGrid(ray.point_x_m,  ray.point_y_m);   // hit
        auto [rgx, rgy] = worldToGrid(ray.origin_x_m, ray.origin_y_m);  // origin

        // Clip once; if fully outside, nothing to do
        int x0 = rgx, y0 = rgy, x1 = gx, y1 = gy;
        if (!clipLineToGrid(x0, y0, x1, y1)) return;

        // --- Tunables (INDOOR) ---
        constexpr float L_FREE_LAMBDA_M = 0.40f;  // free space falloff (~37% at 0.4 m)
        constexpr float L_HIT_LAMBDA_M  = 1.80f;  // hit falloff (~37% at 1.8 m)
        constexpr int   FREE_BASE       = 40;     // max decrement near sensor
        constexpr int   HIT_BASE        = 80;     // max increment for close hits
        constexpr int   FREE_FLOOR      = 1;      // minimum decrement per free cell
        constexpr int   HIT_FLOOR       = 2;      // minimum increment at hit

        // Precompute world-space total range (meters)
        const float dxw = ray.point_x_m - ray.origin_x_m;
        const float dyw = ray.point_y_m - ray.origin_y_m;
        const float total_len_m = std::max(1e-6f, std::sqrt(dxw*dxw + dyw*dyw));

        // Bresenham on the clipped segment using pointer steps
        int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy;

        // Start pointer at (x0,y0)
        value_type* p = data_.data() + size_t(y0) * WIDTH + x0;
        const int stepX = sx;
        const int stepY = sy * int(WIDTH);

        // For a quick along-ray parameter, count grid steps vs. max(|dx|,|dy|)
        const int steps_max = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
        int steps_done = 0;

        while (true) {
            // Param t in [0..1] along the ray (approximate)
            float t = (steps_max > 0) ? float(steps_done) / float(steps_max) : 0.0f;
            float d_here_m = t * total_len_m;

            // Free-space weight decays with distance
            float w_free = std::exp(-d_here_m / L_FREE_LAMBDA_M);
            int   decr   = std::max(FREE_FLOOR, int(std::lround(FREE_BASE * w_free)));
            *p = sat_sub(*p, decr);

            if (x0 == x1 && y0 == y1) break;
            int e2 = err << 1;
            if (e2 >= dy) { err += dy; x0 += sx; p += stepX; }
            if (e2 <= dx) { err += dx; y0 += sy; p += stepY; }
            ++steps_done;
        }

        // Occupied hit cell: increment with distance falloff based on total range
        {
            float w_hit = std::exp(-total_len_m / L_HIT_LAMBDA_M);
            int   incr  = std::max(HIT_FLOOR, int(std::lround(HIT_BASE * w_hit)));

            if (inBounds(gx, gy)) {
                value_type& h = at(size_t(gx), size_t(gy));
                h = sat_add(h, incr);
            } else {
                value_type& h = at(size_t(x1), size_t(y1));
                h = sat_add(h, incr);
            }
        }
    }

    RayResult RayCastOnGrid(float x, float y, float angle_deg, float max_range_m) const noexcept {
        RayResult rr{max_range_m, -1, -1, false, false, false};

        auto inb = [this](int gx, int gy){ return inBounds(gx, gy); };

        // Early OOB check using your existing integer mapping
        auto [gx0, gy0] = worldToGrid(x, y);
        if (!inb(gx0, gy0)) {
            rr.started_oob = true;
            rr.distance_m  = 0.f;
            return rr;
        }

        // ---- Direction in STANDARD math frame ----
        const float rad = angle_deg * float(M_PI) / 180.0f;   // <— no 180°-angle flip
        float dx = std::cos(rad);
        float dy = std::sin(rad);

        if (std::abs(dx) < 1e-9f && std::abs(dy) < 1e-9f) {
            // No movement → no hit, at max range
            return rr;
        }

        // ---- Map world -> continuous grid coordinates ----
        const float cs = cell_size_m_;

        // Continuous world-to-grid in math frame (y up).
        float gx_f = x / cs;
        float gy_f = y / cs;

        // If your grid rows increase downward (typical image-style),
        // flip Y exactly once for DDA space.
        constexpr bool Y_DOWN_IMAGE = true;
        if constexpr (Y_DOWN_IMAGE) {
            // H is the grid height (in cells)
            const int H = int(HEIGHT);
            gy_f = float(H) - 1.0f - gy_f;   // y_up -> y_down index space
            dy   = -dy;                      // flip direction to match y-down
        }

        // Integer starting cell (in grid index space used by at()/inBounds())
        int ix = int(std::floor(gx_f));
        int iy = int(std::floor(gy_f));

        // Safety: make sure integer start matches earlier OOB check
        if (!inb(ix, iy)) {
            rr.started_oob = true;
            rr.distance_m  = 0.f;
            return rr;
        }

        // If starting cell is occupied → hit at zero
        if (at(size_t(ix), size_t(iy)) >= 128) {
            rr.hit = true; rr.distance_m = 0.f; rr.cell_x = ix; rr.cell_y = iy;
            return rr;
        }

        // ---- Standard grid DDA ----
        auto nextBoundary = [](float r, int i, int step){
            return (step > 0) ? (float(i) + 1.0f - r) : (r - float(i));
        };

        const int stepx = (dx > 0.f) ? 1 : -1;
        const int stepy = (dy > 0.f) ? 1 : -1;

        const float absdx = std::abs(dx);
        const float absdy = std::abs(dy);

        float tMaxX   = (dx != 0.f) ? nextBoundary(gx_f, ix, stepx) / absdx : std::numeric_limits<float>::infinity();
        float tMaxY   = (dy != 0.f) ? nextBoundary(gy_f, iy, stepy) / absdy : std::numeric_limits<float>::infinity();
        float tDeltaX = (dx != 0.f) ? (1.f / absdx) : std::numeric_limits<float>::infinity();
        float tDeltaY = (dy != 0.f) ? (1.f / absdy) : std::numeric_limits<float>::infinity();

        const float tMaxCells = max_range_m / cs;
        float t = 0.f;

        while (t <= tMaxCells) {
            if (tMaxX < tMaxY) {
                ix += stepx; t = tMaxX; tMaxX += tDeltaX;
            } else {
                iy += stepy; t = tMaxY; tMaxY += tDeltaY;
            }

            if (!inb(ix, iy)) {
                rr.out_of_bounds = true;
                rr.distance_m = std::min(max_range_m, t * cs);
                return rr;
            }
            if (at(size_t(ix), size_t(iy)) >= 128) {
                rr.hit = true;
                rr.distance_m = std::min(max_range_m, t * cs);
                rr.cell_x = ix; rr.cell_y = iy;
                return rr;
            }
        }

        // No hit within max range, stayed in-bounds
        rr.distance_m = max_range_m; // hit=false, out_of_bounds=false
        return rr;
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