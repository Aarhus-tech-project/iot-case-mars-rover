#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include <optional>

#include "Config.h"
#include "Lidar.h"
#include "OccupancyGrid.h"

struct Particle {
    float x {0.f};
    float y {0.f};
    float heading_deg {0.f};
    float weight {1.f};
};

float WrapAddDeg(float a, float b) {
    float s = a + b;
    while (s < 0.f) s += 360.f;
    while (s >= 360.f) s -= 360.f;
    return s;
}

template <size_t WIDTH, size_t HEIGHT>
class MonteCarloLocalization {
public:
    OccupancyGrid<WIDTH, HEIGHT> grid;
    std::vector<Particle> particles;
    std::random_device rd;
    std::mt19937 gen{rd()};

    MonteCarloLocalization(OccupancyGrid<WIDTH, HEIGHT>& _grid) : grid(_grid) {
        Init();
    }

    void Init() {
        particles.clear();
        particles.reserve(MLC_NUM_PARTICLES);

        std::uniform_real_distribution<float> dist_x(0.f, WIDTH * grid.cellSizeM());
        std::uniform_real_distribution<float> dist_y(0.f, HEIGHT * grid.cellSizeM());

        std::uniform_real_distribution<float> dist_heading(0.f, 360.f);

        for (int i = 0; i < MLC_NUM_PARTICLES; ++i) {
            Particle p;
            p.x = dist_x(gen);
            p.y = dist_y(gen);
            p.heading_deg = dist_heading(gen);
            p.weight = 1.f / MLC_NUM_PARTICLES;
            particles.push_back(p);
        }
    }

    void Iterate(const std::vector<Lidar>& lidar) {
        if (particles.empty()) {
            Init();
            return;
        }

        std::vector<Particle> new_particles;
        new_particles.reserve(particles.size());
        std::vector<float> cumulative_weights;
        cumulative_weights.reserve(particles.size());
        float cum_sum = 0.f;
        for (const auto& p : particles) {
            cum_sum += p.weight;
            cumulative_weights.push_back(cum_sum);
        }
        std::uniform_real_distribution<float> dist(0.f, cum_sum);
        for (size_t i = 0; i < particles.size(); ++i) {
            float r = dist(gen);
            auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), r);
            size_t index = std::distance(cumulative_weights.begin(), it);
            if (index >= particles.size()) index = particles.size() - 1;
            new_particles.push_back(particles[index]);
        }
        particles = std::move(new_particles);
        std::normal_distribution<float> noise_pos(0.f, 0.05f);      // 5 cm position noise
        std::normal_distribution<float> noise_heading(0.f, 2.f);   // 2° heading noise
        for (auto& p : particles) {
            p.x += noise_pos(gen);
            p.y += noise_pos(gen);
            p.heading_deg = WrapAddDeg(p.heading_deg, noise_heading(gen));
        }

        // Evaluate particles with lidar data
        EvaluateParticles(lidar);
    }

    Particle GetOptimalRotation(const std::vector<Lidar>& lidar, Particle particle) {
        if (lidar.empty()) return particle;

        float best_heading = particle.heading_deg;
        float best_score = EvalutateParticle(lidar, particle);

        const int angle_step = 1; // degrees
        for (int i = 0; i < 360; i += angle_step) {
            Particle test_particle = particle;
            test_particle.heading_deg = WrapAddDeg(particle.heading_deg, i);
            float score = EvalutateParticle(lidar, test_particle);
            if (score > best_score) {
                best_score = score;
                best_heading = test_particle.heading_deg;
            }
        }

        particle.heading_deg = best_heading;
        return particle;   
    }

    void EvaluateParticles(const std::vector<Lidar>& lidar) {
        float total_weight = 0.f;
        for (auto& p : particles) {
            p.weight = EvalutateParticle(lidar, p);
            total_weight += p.weight;
        }
        if (total_weight > 0.f) {
            for (auto& p : particles) {
                p.weight /= total_weight;
            }
        } else {
            // All weights zero, reinitialize
            Init();
        }
    }

    float EvalutateParticle(const std::vector<Lidar>& lidar, const Particle& particle) {
        // meters, not mm
        const float Rmax_m       = LIDAR_MAX_MM * 0.001f;
        const float sigma_hit_m  = 0.10f;   // ~10 cm tolerance
        const float inv_2s2      = 1.0f / (2.0f * sigma_hit_m * sigma_hit_m);
        const float p_floor      = 1e-3f;   // prevents underflow → visible weight

        auto wrap = [](float d)->float {
            while (d >= 360.f) d -= 360.f;
            while (d <    0.f) d += 360.f;
            return d;
        };

        if (lidar.empty()) return 0.f;

        double sum = 0.0;
        int used = 0;

        for (const auto& ray : lidar) {
            if (ray.distance_mm == 0) continue;

            // measured distance in meters (clamped)
            const float meas_m = std::min(ray.distance_mm * 0.001f, Rmax_m);

            // world angle (0°=up, +CW)
            const float beam_deg  = ray.angle_cdeg * 0.01f;
            const float world_deg = wrap(particle.heading_deg + beam_deg);

            // predicted distance from the grid (meters)
            float pred_m = grid.RayCastOnGrid(particle.x, particle.y, world_deg, Rmax_m);
            if (!std::isfinite(pred_m) || pred_m < 0.f) pred_m = Rmax_m;
            pred_m = std::min(pred_m, Rmax_m);

            const bool meas_hit = (meas_m < Rmax_m);
            const bool pred_hit = (pred_m < Rmax_m);

            float s;
            if (meas_hit && pred_hit) {
                const float e = std::fabs(meas_m - pred_m);
                s = std::exp(-(e*e) * inv_2s2);
            } else if (meas_hit != pred_hit) {
                const float e = std::fabs((meas_hit ? meas_m : pred_m) - Rmax_m);
                const float sigma_miss_m = 0.20f;
                s = std::exp(-(e*e) / (2.f * sigma_miss_m * sigma_miss_m));
            } else {
                s = 0.25f;
            }

            if (!(s > 0.f)) s = 0.f;
            s = std::max(p_floor, s);

            sum += s;
            ++used;
        }

        if (used == 0) return 0.f;
        float score = float(sum / double(used));   // 0..1
        if (!std::isfinite(score)) score = 0.f;
        return score;
    }

    Particle GetMeanParticle() {
        if (particles.empty()) return {0.f, 0.f, 0.f, 0.f};

        float sum_x = 0.f;
        float sum_y = 0.f;
        float sum_sin = 0.f;
        float sum_cos = 0.f;
        float sum_w = 0.f;

        for (const auto& p : particles) {
            sum_x += p.x * p.weight;
            sum_y += p.y * p.weight;
            sum_sin += std::sin(p.heading_deg * M_PI / 180.0f) * p.weight;
            sum_cos += std::cos(p.heading_deg * M_PI / 180.0f) * p.weight;
            sum_w += p.weight;
        }

        if (sum_w == 0.f) return {0.f, 0.f, 0.f, 0.f};

        Particle mean;
        mean.x = sum_x / sum_w;
        mean.y = sum_y / sum_w;
        mean.heading_deg = std::atan2(sum_sin, sum_cos) * 180.0f / M_PI;
        mean.weight = sum_w / particles.size();
        return mean;
    }

    Particle GetBestParticle() {
        if (particles.empty()) return {0.f, 0.f, 0.f, 0.f};

        Particle best_particle = particles[0];
        for (const auto& p : particles) {
            if (p.weight > best_particle.weight) {
                best_particle = p;
            }
        }
        return best_particle;
    }

    std::pair<float,float> computeNeff() {
        if (particles.empty()) return {0.f, 0.f};

        double sum = 0.0, sumsq = 0.0;
        for (const auto& p : particles) {
            const double w = std::max(1e-12, double(p.weight)); // tiny floor
            sum   += w;
            sumsq += w * w;
        }
        if (sumsq <= 0.0) return {0.f, 0.f};

        const double Neff = (sum*sum) / sumsq;  // works for unnormalized too
        const double frac = Neff / double(particles.size());
        return { float(Neff), float(frac) };
    }

    float GetParticleConfidence() {
        if (particles.empty()) return 0.f;
        float max_w = 0.f;
        for (const auto& p : particles) {
            if (p.weight > max_w) max_w = p.weight;
        }
        return max_w;
    }
};