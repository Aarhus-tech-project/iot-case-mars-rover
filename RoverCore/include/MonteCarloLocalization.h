#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include <optional>

#include "Config.h"
#include "Lidar.h"
#include "OccupancyGrid.h"

struct OdomDelta {
    float dx_m   {0.f};
    float dy_m   {0.f};
    float ddeg   {0.f};
    float q_xx {0.f}, q_yy {0.f}, q_dd {0.f};
};

struct Particle {
    float x {0.f};
    float y {0.f};
    float heading_deg {0.f};
    float weight {1.f};
};

template <size_t WIDTH, size_t HEIGHT>
class MonteCarloLocalization {
public:
    MonteCarloLocalization(const OccupancyGrid<WIDTH, HEIGHT>& _grid) : grid(_grid) {
        Init();
    }

    void Init() {
        particles.clear();
        particles.reserve(MLC_NUM_PARTICLES);
    }

    void Iterate(std::vector<Lidar> lidar) {
         
    }

private:
    OccupancyGrid<WIDTH, HEIGHT> grid;
    std::vector<Particle> particles;
};