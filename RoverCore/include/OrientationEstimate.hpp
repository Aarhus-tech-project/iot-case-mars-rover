#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include <algorithm>

#include "Lidar.hpp"

struct OrientationEstimate {
    float heading_up_deg;      // dominant grid axis in your frame (0=up, +CW)
    float snapped_up_deg;      // snapped to nearest multiple of 90°
    float confidence;          // 0..1-ish (higher is better)
    int   used_segments;       // how many segments contributed
};

static inline float rad2deg(float r){ return r * 180.0f / float(M_PI); }
static inline float deg2rad(float d){ return d * float(M_PI) / 180.0f; }
static inline float wrapDeg(float d){
    while (d <   0.f) d += 360.f;
    while (d >= 360.f) d -= 360.f;
    return d;
}

// Convert your up/CW angle (deg) to math CCW radians (0 = +X, +CCW)
static inline float upCWdeg_to_mathRad(float upCW_deg) {
    return deg2rad(90.0f - upCW_deg);
}

// Main estimator
// target_step_deg: try to build segments roughly this far apart angularly (2..5° works well)
// max_gap_m: reject segments that jump across large gaps (doorways / missing returns)
// min_seg_m: reject tiny jitter segments (sensor noise)
OrientationEstimate EstimateHeadingFromScan(
    const std::vector<Lidar>& scan,
    float target_step_deg = 3.0f,
    float max_gap_m       = 0.30f,
    float min_seg_m       = 0.02f,
    int   max_neighbor    = 12,       // search at most this many points ahead for neighbor
    uint32_t max_range_mm = 0         // 0 = ignore; else drop hits beyond this range
){
    // 1) Filter + sort by angle (handle wrap)
    std::vector<Lidar> hits;
    hits.reserve(scan.size());
    for (const auto& b : scan) {
        if (b.distance_mm == 0) continue;
        if (max_range_mm && b.distance_mm > max_range_mm) continue;
        hits.push_back(b);
    }
    if (hits.size() < 8) {
        return {0.f, 0.f, 0.f, 0};
    }
    std::sort(hits.begin(), hits.end(),
              [](const Lidar& a, const Lidar& b){ return a.angle_cdeg < b.angle_cdeg; });

    // 2) Precompute XY in math frame (+X, +CCW)
    struct Pt { float x, y; float deg_upCW; float r_m; float I; };
    std::vector<Pt> pts;
    pts.reserve(hits.size());
    for (const auto& h : hits) {
        float r_m = 0.001f * h.distance_mm;
        float a_up = 0.01f * float(h.angle_cdeg);
        float a_rad = upCWdeg_to_mathRad(a_up);
        pts.push_back({ r_m * std::cos(a_rad), r_m * std::sin(a_rad), a_up, r_m, float(h.intensity) });
    }
    const int N = int(pts.size());
    if (N < 8) return {0.f, 0.f, 0.f, 0};

    // Helper: angular distance ahead (in degrees, handling wrap), and find a neighbor near target_step_deg
    auto ahead_deg = [](float a, float b){ // both in [0,360)
        float d = b - a;
        if (d < 0) d += 360.f;
        return d; // 0..360
    };

    auto find_neighbor = [&](int i)->int {
        // Walk forward up to max_neighbor points, pick the one whose angular delta is closest to target_step_deg
        float a0 = pts[i].deg_upCW;
        float bestDiff = 1e9f;
        int bestJ = -1;
        for (int k = 1; k <= max_neighbor && k < N; ++k) {
            int j = (i + k) % N;
            float d = ahead_deg(a0, pts[j].deg_upCW);
            float diff = std::fabs(d - target_step_deg);
            if (diff < bestDiff) { bestDiff = diff; bestJ = j; }
            if (d > target_step_deg && bestJ >= 0) break; // passed the target; early exit
        }
        if (bestJ < 0) bestJ = (i + 1) % N; // fallback to immediate neighbor
        return bestJ;
    };

    // 3) Accumulate 90°-period orientation mean using local segments
    double C = 0.0, S = 0.0;   // sums of w * cos(4a), w * sin(4a)
    double W = 0.0;            // total weight
    int used = 0;

    for (int i = 0; i < N; ++i) {
        int j = find_neighbor(i);
        if (j == i) continue;

        float dx = pts[j].x - pts[i].x;
        float dy = pts[j].y - pts[i].y;
        float seg = std::sqrt(dx*dx + dy*dy);
        if (seg < min_seg_m || seg > max_gap_m) continue;

        float a = std::atan2(dy, dx); // radians, math frame (+X, +CCW)

        // Weight: segment length * (average intensity+1) to favor strong, long edges
        float Iavg = 0.5f * (pts[i].I + pts[j].I);
        double w = double(seg) * (1.0 + double(Iavg) / 255.0);

        C += w * std::cos(4.0 * a);
        S += w * std::sin(4.0 * a);
        W += w;
        ++used;
    }

    if (used < 5 || W <= 0.0) {
        return {0.f, 0.f, 0.f, used};
    }

    // 4) Dominant axis in math frame: a* = (1/4) atan2(S, C)
    float a_star_rad = 0.25f * std::atan2((float)S, (float)C); // radians
    float a_star_deg_math = rad2deg(a_star_rad);                // 0°=+X, +CCW

    // Convert to your frame: 0°=up, +CW
    float heading_up = wrapDeg(90.0f - a_star_deg_math);

    // Snap to nearest multiple of 90° (still in your frame)
    float snapped_up = wrapDeg(std::round(heading_up / 90.0f) * 90.0f);

    // Confidence ~ normalized resultant length in 4θ-space (0..1-ish)
    // Not strictly bounded to 1 unless weights are normalized; this is a good heuristic.
    float R = float(std::hypot(C, S));
    float confidence = float(R / (W + 1e-9)); // larger -> stronger dominant orientation

    return { heading_up, snapped_up, confidence, used };
}