#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include <optional>
#include <limits>
#include <optional>

#include "Config.hpp"
#include "Lidar.hpp"
#include "OccupancyGrid.hpp"

struct Particle {
    float x {0.f};
    float y {0.f};
    float heading_deg {0.f};
    float weight {1.f};
};

// Cheaper wrap to [0,360)
inline float WrapDeg360(float d) {
    d = std::fmod(d, 360.f);
    if (d < 0.f) d += 360.f;
    return d;
}

inline float WrapAddDeg(float a, float b) { return WrapDeg360(a + b); }

template <size_t WIDTH, size_t HEIGHT>
class MonteCarloLocalization {
public:
    OccupancyGrid<WIDTH, HEIGHT> grid;
    std::vector<Particle> particles;

    std::mt19937 gen;

    // ---- Tunables (good defaults for a Pi) ----
    int   max_beams_eval      = 192;   // cap beams per particle; stratified coverage
    float sigma_pos_m         = 0.05f; // process noise pos
    float sigma_heading_deg   = 3.0f;  // process noise heading
    float p_floor             = 1e-4f; // likelihood floor
    float sigma_hit_m         = 0.10f; // sensor model (hit)
    float sigma_miss_m        = 0.20f; // sensor model (miss)
    float restart_conf_th     = 0.05f; // if best weight below this...
    float restart_neff_frac   = 0.20f; // ...and Neff/N below this...
    int   restart_patience    = 3;     // ...for this many frames → restart
    float elite_keep_frac     = 0.10f; // keep top 10% on restart
    float roughen_neff_frac   = 0.85f; // if Neff/N too high, add tiny jitter
    float roughen_pos_m       = 0.01f; // roughening amount
    float roughen_head_deg    = 1.0f;  // roughening amount

    // Internal state
    int low_conf_streak = 0;

    MonteCarloLocalization(OccupancyGrid<WIDTH, HEIGHT>& _grid)
        : grid(_grid), gen(std::random_device{}()) {
        StartUniform();
    }

    // ---- New: start centered around a pose (Gaussian cloud) ----
    void StartAt(float x_m, float y_m, float heading_deg,
                 float pos_sigma_m = 0.25f, float heading_sigma_deg = 10.f) {
        particles.clear();
        particles.reserve(MLC_NUM_PARTICLES);

        std::normal_distribution<float> nx(x_m, pos_sigma_m);
        std::normal_distribution<float> ny(y_m, pos_sigma_m);
        std::normal_distribution<float> nh(heading_deg, heading_sigma_deg);

        const float w0 = 1.f / float(MLC_NUM_PARTICLES);
        for (int i = 0; i < MLC_NUM_PARTICLES; ++i) {
            Particle p;
            p.x = nx(gen);
            p.y = ny(gen);
            p.heading_deg = WrapDeg360(nh(gen));
            p.weight = w0;
            particles.push_back(p);
        }
        low_conf_streak = 0;
    }

    // ---- New: explicit global start (uniform over the map) ----
    void StartUniform() { Init(); }

    void Init() {
        particles.clear();
        particles.reserve(MLC_NUM_PARTICLES);

        const float world_w = WIDTH  * grid.cellSizeM();
        const float world_h = HEIGHT * grid.cellSizeM();

        std::uniform_real_distribution<float> dist_x(0.f, world_w);
        std::uniform_real_distribution<float> dist_y(0.f, world_h);
        std::uniform_real_distribution<float> dist_heading(0.f, 360.f);

        const float w0 = 1.f / float(MLC_NUM_PARTICLES);
        for (int i = 0; i < MLC_NUM_PARTICLES; ++i) {
            Particle p;
            p.x = dist_x(gen);
            p.y = dist_y(gen);
            p.heading_deg = dist_heading(gen);
            p.weight = w0;
            particles.push_back(p);
        }
        low_conf_streak = 0;
    }

    void Iterate(const std::vector<Lidar>& lidar) {
        if (particles.empty()) { Init(); return; }
        if (lidar.empty())     { return; }

        // -------- Systematic resampling --------
        const size_t N = particles.size();
        float Wsum = 0.f;
        for (const auto& p : particles) Wsum += p.weight;
        if (!(Wsum > 0.f)) { Init(); return; }

        const float step = Wsum / float(N);
        std::uniform_real_distribution<float> uni(0.f, step);
        float u = uni(gen);

        std::vector<Particle> new_particles;
        new_particles.reserve(N);

        float c = particles[0].weight;
        size_t i = 0;
        for (size_t m = 0; m < N; ++m) {
            const float U = u + float(m) * step;
            while (U > c && (i + 1) < N) { ++i; c += particles[i].weight; }
            new_particles.push_back(particles[i]);
            new_particles.back().weight = 1.f / float(N);
        }
        particles.swap(new_particles);

        // -------- Motion noise / roughening --------
        std::normal_distribution<float> noise_pos(0.f, sigma_pos_m);
        std::normal_distribution<float> noise_heading(0.f, sigma_heading_deg);
        for (auto& p : particles) {
            p.x += noise_pos(gen);
            p.y += noise_pos(gen);
            p.heading_deg = WrapAddDeg(p.heading_deg, noise_heading(gen));
        }

        // -------- Evaluate --------
        EvaluateParticles(lidar);

        // Optional tiny roughening if Neff too high (prevent impoverishment)
        auto [Neff, frac] = computeNeff();
        if (frac > roughen_neff_frac) {
            std::normal_distribution<float> rpos(0.f, roughen_pos_m);
            std::normal_distribution<float> rhead(0.f, roughen_head_deg);
            for (auto& p : particles) {
                p.x += rpos(gen);
                p.y += rpos(gen);
                p.heading_deg = WrapAddDeg(p.heading_deg, rhead(gen));
            }
        }

        // -------- Auto-restart guard --------
        const float conf = GetParticleConfidence();
        if (conf < restart_conf_th && frac < restart_neff_frac) {
            if (++low_conf_streak >= restart_patience) {
                ReseedWithElitesAndNoise();
                low_conf_streak = 0;
            }
        } else {
            low_conf_streak = 0;
        }
    }

    // Optional: faster, coarse rotation snap around best (unchanged signature)
    Particle GetOptimalRotation(const std::vector<Lidar>& lidar, Particle particle) {
        if (lidar.empty()) return particle;

        float best_heading = particle.heading_deg;
        float best_score   = EvalutateParticle(lidar, particle);

        const int coarse_step = 8; // coarser for speed on Pi
        for (int a = 0; a < 360; a += coarse_step) {
            Particle t = particle;
            t.heading_deg = WrapAddDeg(particle.heading_deg, float(a));
            const float s = EvalutateParticle(lidar, t);
            if (s > best_score) { best_score = s; best_heading = t.heading_deg; }
        }
        // small refine
        for (int da = -3; da <= 3; ++da) {
            Particle t = particle;
            t.heading_deg = WrapAddDeg(best_heading, float(da));
            const float s = EvalutateParticle(lidar, t);
            if (s > best_score) { best_score = s; best_heading = t.heading_deg; }
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

        if (total_weight > 0.f && std::isfinite(total_weight)) {
            const float inv = 1.f / total_weight;
            for (auto& p : particles) p.weight *= inv;
        } else {
            Init();
        }
    }

    // NOTE: name preserved for compatibility
    float EvalutateParticle(const std::vector<Lidar>& lidar, const Particle& particle) {
        // Precompute constants
        const float Rmax_m = LIDAR_MAX_MM * 0.001f;
        const float inv_2s2_hit  = 1.0f / (2.0f * sigma_hit_m  * sigma_hit_m);
        const float inv_2s2_miss = 1.0f / (2.0f * sigma_miss_m * sigma_miss_m);

        const size_t M = lidar.size();
        if (M == 0) return 0.f;

        // ---- Stratified angular subsampling to avoid bias/false bottoms ----
        const int K = std::max(1, std::min(max_beams_eval, int(M)));
        double sum = 0.0;
        int used = 0;

        // Temperature annealing (sharpen as Neff rises)
        // alpha > 1 = sharper; < 1 = smoother
        float alpha = 1.0f;
        {
            auto [Neff, frac] = computeNeff();
            alpha = 0.9f + 0.6f * std::clamp(frac, 0.f, 1.f); // ~0.9 → 1.5
        }

        for (int k = 0; k < K; ++k) {
            // Evenly spread indices across full scan
            size_t i = (size_t)((k + 0.5f) * (float(M) / float(K)));
            if (i >= M) i = M - 1;
            const auto& ray = lidar[i];

            if (ray.distance_mm <= 0) continue;

            float meas_m = ray.distance_mm * 0.001f;
            if (meas_m > Rmax_m) meas_m = Rmax_m;

            const float beam_deg  = ray.angle_cdeg * 0.01f;
            const float world_deg = WrapAddDeg(particle.heading_deg, beam_deg);

            float pred_m = grid.RayCastOnGrid(particle.x, particle.y, world_deg, Rmax_m);
            if (!(pred_m >= 0.f && std::isfinite(pred_m))) pred_m = Rmax_m;
            if (pred_m > Rmax_m) pred_m = Rmax_m;

            const bool meas_hit = (meas_m < Rmax_m);
            const bool pred_hit = (pred_m < Rmax_m);

            float s;
            if (meas_hit && pred_hit) {
                const float e = std::fabs(meas_m - pred_m);
                s = std::exp(-(e * e) * inv_2s2_hit);
            } else if (meas_hit != pred_hit) {
                const float e = std::fabs((meas_hit ? meas_m : pred_m) - Rmax_m);
                s = std::exp(-(e * e) * inv_2s2_miss);
            } else {
                s = 0.25f;
            }

            // temperature & floor
            s = std::max(p_floor, s);
            if (alpha != 1.0f) s = std::pow(s, alpha);

            sum += double(s);
            ++used;
        }

        if (used == 0) return 0.f;
        float score = float(sum / double(used));
        if (!std::isfinite(score)) score = 0.f;
        return score;
    }

    Particle GetBestParticle() {
        if (particles.empty()) return {0.f, 0.f, 0.f, 0.f};
        Particle best = particles[0];
        for (const auto& p : particles) if (p.weight > best.weight) best = p;
        return best;
    }

    std::pair<float,float> computeNeff() {
        if (particles.empty()) return {0.f, 0.f};
        double sum = 0.0, sumsq = 0.0;
        for (const auto& p : particles) {
            const double w = std::max(1e-12, double(p.weight));
            sum   += w;
            sumsq += w * w;
        }
        if (!(sumsq > 0.0)) return {0.f, 0.f};
        const double Neff = (sum * sum) / sumsq;
        const double frac = Neff / double(particles.size());
        return { float(Neff), float(frac) };
    }

    float GetParticleConfidence() {
        if (particles.empty()) return 0.f;
        float max_w = 0.f;
        for (const auto& p : particles) if (p.weight > max_w) max_w = p.weight;
        return max_w;
    }

private:
    // ---- New: elite-keep restart to escape false bottoms quickly ----
    void ReseedWithElitesAndNoise() {
        if (particles.empty()) { Init(); return; }

        // sort by weight desc
        std::vector<size_t> idx(particles.size());
        for (size_t i = 0; i < idx.size(); ++i) idx[i] = i;
        std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b){
            return particles[a].weight > particles[b].weight;
        });

        const size_t N = particles.size();
        const size_t eliteN = std::max<size_t>(1, size_t(elite_keep_frac * N));

        std::vector<Particle> next;
        next.reserve(N);

        // keep elites (renormalize later)
        for (size_t e = 0; e < eliteN; ++e) {
            next.push_back(particles[idx[e]]);
        }

        // half around elites (Gaussian), half uniform global
        const size_t aroundN = (N - eliteN) / 2;
        const size_t uniformN = N - eliteN - aroundN;

        std::normal_distribution<float> npos(0.f, 0.5f);    // broader to escape
        std::normal_distribution<float> nhead(0.f, 15.f);

        const float world_w = WIDTH  * grid.cellSizeM();
        const float world_h = HEIGHT * grid.cellSizeM();
        std::uniform_real_distribution<float> ux(0.f, world_w);
        std::uniform_real_distribution<float> uy(0.f, world_h);
        std::uniform_real_distribution<float> uh(0.f, 360.f);

        // around elites
        for (size_t k = 0; k < aroundN; ++k) {
            const Particle& e = next[k % eliteN];
            Particle p;
            p.x = e.x + npos(gen);
            p.y = e.y + npos(gen);
            p.heading_deg = WrapAddDeg(e.heading_deg, nhead(gen));
            p.weight = 1.f; // temp
            next.push_back(p);
        }
        // uniform
        for (size_t k = 0; k < uniformN; ++k) {
            Particle p;
            p.x = ux(gen);
            p.y = uy(gen);
            p.heading_deg = uh(gen);
            p.weight = 1.f; // temp
            next.push_back(p);
        }

        // renormalize
        const float w0 = 1.f / float(N);
        for (auto& p : next) p.weight = w0;
        particles.swap(next);
    }
};

static constexpr float ACCEPT_W_THRESHOLD = 0.85f;  // your gate
static constexpr int   ATTEMPTS_MAX      = 5;       // how many restarts we'll try
static constexpr int   ITER_PER_CHUNK    = 8;       // iterations between checks
static constexpr int   CHUNKS_PER_ATTEMPT= 10;      // 8*10 = 80 iters per attempt

struct Pose {
    float x, y, heading_deg;
    float weight;
};

std::optional<Pose> TryLocalizeLidar(OccupancyGrid<GRID_WIDTH, GRID_HEIGHT>& grid,
                 const std::vector<Lidar>& scan,
                 // optional prior; set to std::nullopt if unknown
                 std::optional<Pose> prior = std::nullopt)
{
    MonteCarloLocalization<GRID_WIDTH, GRID_HEIGHT> mcl(grid);

    // Pi-friendly defaults (tweak if needed)
    mcl.max_beams_eval    = 128;  // 96–192, lower = faster
    mcl.sigma_pos_m       = 0.04f;
    mcl.sigma_heading_deg = 2.5f;
    mcl.p_floor           = 2e-4f; // helps avoid totally flat weights

    for (int attempt = 0; attempt < ATTEMPTS_MAX; ++attempt) {
        // ---- (Re)start strategy ----
        if (attempt == 0 && prior.has_value()) {
            mcl.StartAt(prior->x, prior->y, prior->heading_deg, 0.20f, 8.0f);
        } else if (attempt == 0) {
            mcl.StartUniform();
        } else {
            // Mix: keep some elites and broaden around them (already implemented in the class),
            // but also explicitly widen process noise to explore more on this attempt:
            mcl.sigma_pos_m       = 0.06f + 0.02f * attempt; // 0.06, 0.08, ...
            mcl.sigma_heading_deg = 4.0f  + 1.0f  * attempt; // 4°, 5°, ...
            // Full uniform reset is also fine:
            mcl.StartUniform();
        }

        float best_score_this_attempt = 0.f;
        Pose  best_pose_this_attempt  = {0,0,0,0};

        // ---- Work in chunks; check acceptance between chunks ----
        for (int chunk = 0; chunk < CHUNKS_PER_ATTEMPT; ++chunk) {
            for (int k = 0; k < ITER_PER_CHUNK; ++k) mcl.Iterate(scan);

            // Evaluate best + rotation-refined best
            Particle best = mcl.GetBestParticle();
            float bestW   = mcl.EvalutateParticle(scan, best);

            Particle bestRot = mcl.GetOptimalRotation(scan, best);
            float bestRotW   = mcl.EvalutateParticle(scan, bestRot);

            float score = std::max(bestW, bestRotW);
            if (score > best_score_this_attempt) {
                best_score_this_attempt = score;
                const Particle& p = (bestRotW >= bestW) ? bestRot : best;
                best_pose_this_attempt = { p.x, p.y, p.heading_deg, score };
            }

            // Accept?
            if (score >= ACCEPT_W_THRESHOLD) {
                std::printf("[mcl] ACCEPT attempt=%d chunk=%d  w=%.3f  pose=(%.2f,%.2f, %.1f°)\n",
                            attempt, chunk, score,
                            best_pose_this_attempt.x, best_pose_this_attempt.y, best_pose_this_attempt.heading_deg);
                return best_pose_this_attempt;
            }
        }

        // Not accepted this attempt — log the best we saw, then restart loop
        std::printf("[mcl] REJECT attempt=%d  best_w=%.3f  pose=(%.2f,%.2f, %.1f°) — reseeding\n",
                    attempt, best_score_this_attempt,
                    best_pose_this_attempt.x, best_pose_this_attempt.y, best_pose_this_attempt.heading_deg);
    }

    // All attempts failed the gate
    std::printf("[mcl] FAILED: no pose met threshold (w >= %.2f)\n", ACCEPT_W_THRESHOLD);
    return std::nullopt;
}