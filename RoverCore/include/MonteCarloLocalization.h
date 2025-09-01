#pragma once

#include <iostream>
#include <cmath>
#include <random>
#include <vector>

#include "Lidar.h"
#include "OccupancyGrid.h"

struct OdomDelta {
    float dx_m   {0.f};
    float dy_m   {0.f};
    float dtheta {0.f};
    // Optional variances; if 0, defaults are used.
    float q_xx {0.f}, q_yy {0.f}, q_tt {0.f};
};

// ---- Particle ----
struct Particle {
    float x {0.f};       // meters, world frame
    float y {0.f};
    float theta {0.f};   // radians [-pi, pi]
    float weight {1.f};  // normalized later
};

template <size_t WIDTH, size_t HEIGHT>
class MonteCarloLocalization {
public:
    using GridT = OccupancyGrid<WIDTH, HEIGHT>;

    explicit MonteCarloLocalization(const GridT& grid,
                                    size_t num_particles = 100)
        : grid_(grid)
        , N_(std::max<size_t>(50, num_particles))
        , particles_(N_)
        , rng_(std::random_device{}())
    {
        initUniform();
    }

    // Main step: z = lidar points; u = optional odometry delta; dt in seconds
    void iterate(const std::vector<Lidar>& z,
                 std::optional<OdomDelta> u,
                 float dt_seconds)
    {
        predict(std::move(u), dt_seconds);
        updateWeights(z);
        normalizeWeights();
        resampleIfNeeded();
        updateEstimate();
    }

    // Convenience: if you sometimes have absolute odom pose, we compute a delta
    // from last estimate and call the delta-based iterate().
    void iterateWithAbsolute(const std::vector<Lidar>& z,
                             float abs_x_m, float abs_y_m, float abs_theta,
                             float dt_seconds)
    {
        // delta in world
        float dxw = abs_x_m - est_.x;
        float dyw = abs_y_m - est_.y;
        float dth = wrapAngle(abs_theta - est_.theta);

        // rotate into previous heading frame
        float c = std::cos(est_.theta), s = std::sin(est_.theta);
        OdomDelta u;
        u.dx_m   =  c*dxw + s*dyw;
        u.dy_m   = -s*dxw + c*dyw;
        u.dtheta =  dth;
        iterate(z, u, dt_seconds);
    }

    // Reinit particles uniformly over map
    void initUniform() {
        std::uniform_real_distribution<float> xdist(0.f, GRID_WIDTH  * grid_.cellSizeM());
        std::uniform_real_distribution<float> ydist(0.f, GRID_HEIGHT * grid_.cellSizeM());
        std::uniform_real_distribution<float> tdist(-float(M_PI), float(M_PI));
        for (auto& p : particles_) {
            p.x = xdist(rng_);
            p.y = ydist(rng_);
            p.theta = tdist(rng_);
            p.weight = 1.f / float(N_);
        }
        est_ = estimate(); // initial rough
    }

    // Current best estimate
    Particle estimate() const {
        // Weighted mean in SE2
        float sx=0.f, sy=0.f, sc=0.f, ss=0.f, sw=0.f;
        for (const auto& p : particles_) {
            sx += p.weight * p.x;
            sy += p.weight * p.y;
            sc += p.weight * std::cos(p.theta);
            ss += p.weight * std::sin(p.theta);
            sw += p.weight;
        }
        Particle e{};
        if (sw <= 0.f) sw = 1.f;
        e.x = sx / sw; e.y = sy / sw;
        e.theta = std::atan2(ss, sc);
        e.weight = 1.f;
        return e;
    }

    const std::vector<Particle>& particles() const noexcept { return particles_; }
    const Particle& estimateCached() const noexcept { return est_; }

    // Tuning knobs (public so you can tweak quickly)
    float sigma_dx_base  = 0.02f;       // m / sqrt(s)
    float sigma_dy_base  = 0.02f;       // m / sqrt(s)
    float sigma_dth_base = 0.01f;       // rad / sqrt(s)

    float z_hit   = 0.9f;               // occupancy hit weight
    float z_rand  = 0.1f;               // random measurement floor
    size_t ray_step = 8;                // downsample lidar for speed

    float neff_frac_threshold = 0.5f;   // resample if Neff < 0.5*N

private:
    static inline float wrapAngle(float a) {
        while (a >  M_PI) a -= 2.f*float(M_PI);
        while (a < -M_PI) a += 2.f*float(M_PI);
        return a;
    }

    void predict(std::optional<OdomDelta> u, float dt) {
        const float sdt = std::sqrt(std::max(0.f, dt));
        std::normal_distribution<float> n01(0.f, 1.f);

        for (auto& p : particles_) {
            float dx=0, dy=0, dth=0;

            if (u) {
                float sdx  = (u->q_xx > 0.f) ? std::sqrt(u->q_xx) : (sigma_dx_base  * sdt);
                float sdy  = (u->q_yy > 0.f) ? std::sqrt(u->q_yy) : (sigma_dy_base  * sdt);
                float sdth = (u->q_tt > 0.f) ? std::sqrt(u->q_tt) : (sigma_dth_base * sdt);
                dx  = u->dx_m   + sdx  * n01(rng_);
                dy  = u->dy_m   + sdy  * n01(rng_);
                dth = u->dtheta + sdth * n01(rng_);
            } else {
                dx  = (sigma_dx_base  * sdt) * n01(rng_);
                dy  = (sigma_dy_base  * sdt) * n01(rng_);
                dth = (sigma_dth_base * sdt) * n01(rng_);
            }

            // compose in particle frame
            float c = std::cos(p.theta), s = std::sin(p.theta);
            p.x     += c*dx - s*dy;
            p.y     += s*dx + c*dy;
            p.theta  = wrapAngle(p.theta + dth);
        }
    }

    void updateWeights(const std::vector<Lidar>& z) {
        // simple endpoint-on-occupancy scoring (fast)
        // You can upgrade to an EDT-based likelihood later.
        const float mm_to_m = 0.001f;

        for (auto& p : particles_) {
            float w = 1e-6f;

            // downsample beams
            for (size_t i = 0; i < z.size(); i += std::max<size_t>(1, ray_step)) {
                const auto& b = z[i];
                if (b.distance_mm == 0) continue; // skip invalid

                const float ang = p.theta + (float(b.angle_cdeg) * 0.01f) * float(M_PI) / 180.f;
                const float r   = float(b.distance_mm) * mm_to_m;

                const float wx = p.x + std::cos(ang) * r;
                const float wy = p.y + std::sin(ang) * r;

                auto [gx, gy] = grid_.worldToGrid(wx, wy);
                if (!grid_.inBounds(gx, gy)) {
                    w += z_rand; // out of map → random weight
                } else {
                    float occ = grid_.at(size_t(gx), size_t(gy)) / 255.f;
                    w += z_rand + z_hit * occ;
                }
            }
            p.weight = w;
        }
    }

    void normalizeWeights() {
        float sum = 0.f;
        for (const auto& p : particles_) sum += p.weight;
        if (sum <= 0.f) {
            const float inv = 1.f / float(N_);
            for (auto& p : particles_) p.weight = inv;
            return;
        }
        for (auto& p : particles_) p.weight /= sum;
    }

    void resampleIfNeeded() {
        // Neff = 1 / sum(w^2)
        float sumsq = 0.f;
        for (const auto& p : particles_) sumsq += p.weight * p.weight;
        const float neff = (sumsq > 0.f) ? (1.f / sumsq) : 0.f;

        if (neff >= neff_frac_threshold * float(N_)) return;

        // Systematic (low-variance) resampling
        std::vector<Particle> newP(N_);
        std::vector<float> cdf(N_);
        cdf[0] = particles_[0].weight;
        for (size_t i = 1; i < N_; ++i) cdf[i] = cdf[i-1] + particles_[i].weight;

        std::uniform_real_distribution<float> u01(0.f, 1.f/float(N_));
        float r = u01(rng_);
        float step = 1.f / float(N_);
        size_t i = 0;
        for (size_t m = 0; m < N_; ++m) {
            float U = r + m * step;
            while (U > cdf[i]) ++i;
            newP[m] = particles_[i];
            newP[m].weight = 1.f / float(N_);
        }
        particles_.swap(newP);
    }

    void updateEstimate() { est_ = estimate(); }

private:
    const GridT& grid_;
    size_t N_;
    std::vector<Particle> particles_;
    Particle est_{};

    std::mt19937 rng_;
};