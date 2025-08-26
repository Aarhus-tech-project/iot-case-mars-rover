#include <array>
#include <cstdint>
#include <utility>

template <size_t WIDTH, size_t HEIGHT>
struct OccupancyGrid {
    static constexpr size_t width  = WIDTH;
    static constexpr size_t height = HEIGHT;

    float cell_size_m; // resolution in meters
    std::array<uint8_t, WIDTH * HEIGHT> data{};

    explicit OccupancyGrid(float cell_size_meters) : cell_size_m(cell_size_meters) {}

    // --- bounds checks ---
    bool inBounds(int x, int y) const {
        return x >= 0 && y >= 0 && x < static_cast<int>(WIDTH) && y < static_cast<int>(HEIGHT);
    }
    bool inBounds(size_t x, size_t y) const {
        return x < WIDTH && y < HEIGHT;
    }

    // --- accessors (unchecked) ---
    uint8_t& at(size_t x, size_t y) { return data[y * WIDTH + x]; }
    const uint8_t& at(size_t x, size_t y) const { return data[y * WIDTH + x]; }

    // world <-> grid
    std::pair<int,int> worldToGrid(float x_m, float y_m) const {
        return { static_cast<int>(x_m / cell_size_m),
                 static_cast<int>(y_m / cell_size_m) };
    }
    std::pair<float,float> gridToWorld(int gx, int gy) const {
        // center of the cell
        return { (gx + 0.5f) * cell_size_m, (gy + 0.5f) * cell_size_m };
    }
};