#include "BNO055.hpp"

#ifdef __linux__

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

BNO055::BNO055(const char* i2c_dev, int addr7) : fd_(-1), addr_(addr7) {
    std::snprintf(dev_, sizeof(dev_), "%s", i2c_dev);
}

BNO055::~BNO055() { close(); }

bool BNO055::begin(bool useExtCrystal, Mode mode) {
    if (!openBus_()) return false;

    // Read chip id (0xA0). Sometimes needs a few tries after power-up.
    uint8_t id = 0;
    for (int i=0;i<10;i++) {
    if (read8_(REG_CHIP_ID, id) && id == 0xA0) break;
    sleepMs_(50);
    if (i==9) { fprintf(stderr, "BNO055: wrong chip id (0x%02x)\n", id); return false; }
    }

    // Enter CONFIG to change settings
    if (!setMode(MODE_CONFIG)) return false;

    // Normal power
    if (!write8_(REG_PWR_MODE, 0x00)) return false;
    sleepMs_(10);

    // Page 0
    if (!write8_(REG_PAGE_ID, 0x00)) return false;

    // Units: accel m/s^2, gyro dps, euler deg, temp C, orientation Android=0
    if (!write8_(REG_UNIT_SEL, 0x00)) return false;

    // Optional: ext crystal (only if your board actually has it; many CJMCU don’t)
    if (useExtCrystal) {
    uint8_t trig=0;
    if (!read8_(REG_SYS_TRIGGER, trig)) return false;
    trig |= 0x80; // CLK_SEL
    if (!write8_(REG_SYS_TRIGGER, trig)) return false;
    sleepMs_(10);
    }

    // Fusion mode
    if (!setMode(mode)) return false;
    sleepMs_(50);
    return true;
}

void BNO055::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool BNO055::setMode(Mode m) {
    if (!write8_(REG_OPR_MODE, static_cast<uint8_t>(m))) return false;
    // mode switch timing (datasheet ~7–19ms)
    sleepMs_(25);
    return true;
}

// ---- reads ----
bool BNO055::readCalib(Calib& c) {
    uint8_t v=0;
    if (!read8_(REG_CALIB_STAT, v)) return false;
    c.sys   = (v >> 6) & 0x03;
    c.gyro  = (v >> 4) & 0x03;
    c.accel = (v >> 2) & 0x03;
    c.mag   = (v >> 0) & 0x03;
    return true;
}

bool BNO055::readTempC(int8_t& tC) {
    uint8_t v=0;
    if (!read8_(REG_TEMP, v)) return false;
    tC = static_cast<int8_t>(v); // signed per datasheet
    return true;
}

bool BNO055::readEuler(Euler& e) {
    uint8_t raw[6];
    if (!readN_(REG_EUL_HEADING_LSB, raw, 6)) return false;
    auto rd16 = [](const uint8_t* p)->int16_t { return (int16_t)(p[0] | (p[1]<<8)); };
    // 1 LSB = 1/16 degree
    constexpr float k = 1.0f/16.0f;
    e.heading_deg = rd16(&raw[0]) * k;
    e.roll_deg    = rd16(&raw[2]) * k;
    e.pitch_deg   = rd16(&raw[4]) * k;
    return true;
}

bool BNO055::readQuaternion(float& w, float& x, float& y, float& z) {
    uint8_t raw[8];
    if (!readN_(REG_QUA_W_LSB, raw, 8)) return false;
    auto rd16 = [](const uint8_t* p)->int16_t { return (int16_t)(p[0] | (p[1]<<8)); };
    constexpr float k = 1.0f/16384.0f;
    w = rd16(&raw[0]) * k;
    x = rd16(&raw[2]) * k;
    y = rd16(&raw[4]) * k;
    z = rd16(&raw[6]) * k;
    return true;
}

bool BNO055::readAccel(Vec3& a) {       // m/s^2
    uint8_t raw[6];
    if (!readN_(REG_ACC_DATA_X_LSB, raw, 6)) return false;
    auto rd16 = [](const uint8_t* p)->int16_t { return (int16_t)(p[0] | (p[1]<<8)); };
    // 1 LSB = 1 mg = 0.00981 m/s^2 when UNIT_SEL=0x00
    constexpr float k = 0.00981f;
    a.x = rd16(&raw[0]) * k;
    a.y = rd16(&raw[2]) * k;
    a.z = rd16(&raw[4]) * k;
    return true;
}

bool BNO055::readGyro(Vec3& g) {        // dps
    uint8_t raw[6];
    if (!readN_(REG_GYR_DATA_X_LSB, raw, 6)) return false;
    auto rd16 = [](const uint8_t* p)->int16_t { return (int16_t)(p[0] | (p[1]<<8)); };
    // 1 LSB = 1/16 dps when UNIT_SEL=0x00
    constexpr float k = 1.0f/16.0f;
    g.x = rd16(&raw[0]) * k;
    g.y = rd16(&raw[2]) * k;
    g.z = rd16(&raw[4]) * k;
    return true;
}

bool BNO055::readLinearAccel(Vec3& v) { // m/s^2, gravity removed
    uint8_t raw[6];
    if (!readN_(REG_LIA_DATA_X_LSB, raw, 6)) return false;
    auto rd16 = [](const uint8_t* p)->int16_t { return (int16_t)(p[0] | (p[1]<<8)); };
    constexpr float k = 0.00981f;
    v.x = rd16(&raw[0]) * k;
    v.y = rd16(&raw[2]) * k;
    v.z = rd16(&raw[4]) * k;
    return true;
}

bool BNO055::readGravity(Vec3& v) {     // m/s^2 gravity vector
    uint8_t raw[6];
    if (!readN_(REG_GRV_DATA_X_LSB, raw, 6)) return false;
    auto rd16 = [](const uint8_t* p)->int16_t { return (int16_t)(p[0] | (p[1]<<8)); };
    constexpr float k = 0.00981f;
    v.x = rd16(&raw[0]) * k;
    v.y = rd16(&raw[2]) * k;
    v.z = rd16(&raw[4]) * k;
    return true;
}

BNO055::Sample BNO055::readSample() {
    Sample s{}; // zero-initialize

    // Timestamp first (monotonic)
    using namespace std::chrono;
    s.time_ns = (uint64_t)duration_cast<nanoseconds>(
                    steady_clock::now().time_since_epoch()).count();

    // Quaternion
    {
        float w=0, x=0, y=0, z=0;
        if (readQuaternion(w,x,y,z)) { s.w=w; s.x=x; s.y=y; s.z=z; }
        else { s.w = s.x = s.y = s.z = std::numeric_limits<float>::quiet_NaN(); }
    }

    // Acceleration (m/s^2)
    {
        Vec3 a{};
        if (readAccel(a)) { s.ax=a.x; s.ay=a.y; s.az=a.z; }
        else { s.ax = s.ay = s.az = std::numeric_limits<float>::quiet_NaN(); }
    }

    // Gyro (dps)
    {
        Vec3 g{};
        if (readGyro(g)) { s.gx=g.x; s.gy=g.y; s.gz=g.z; }
        else { s.gx = s.gy = s.gz = std::numeric_limits<float>::quiet_NaN(); }
    }

    // Magnetometer (uT) — 1 LSB = 1/16 uT when UNIT_SEL=0x00
    {
        uint8_t raw[6];
        if (readN_(REG_MAG_DATA_X_LSB, raw, 6)) {
            auto rd16 = [](const uint8_t* p)->int16_t { return (int16_t)(p[0] | (p[1] << 8)); };
            constexpr float k = 1.0f / 16.0f;
            s.mx = rd16(&raw[0]) * k;
            s.my = rd16(&raw[2]) * k;
            s.mz = rd16(&raw[4]) * k;
        } else {
            s.mx = s.my = s.mz = std::numeric_limits<float>::quiet_NaN();
        }
    }

    // Euler (deg) — heading, roll, pitch with 1 LSB = 1/16 deg
    {
        Euler e{};
        if (readEuler(e)) {
            s.eX = e.heading_deg;  // map as you prefer (heading/roll/pitch)
            s.eY = e.roll_deg;
            s.eZ = e.pitch_deg;
        } else {
            s.eX = s.eY = s.eZ = std::numeric_limits<float>::quiet_NaN();
        }
    }

    // Linear acceleration (m/s^2; gravity removed)
    {
        Vec3 v{};
        if (readLinearAccel(v)) { s.lx=v.x; s.ly=v.y; s.lz=v.z; }
        else { s.lx = s.ly = s.lz = std::numeric_limits<float>::quiet_NaN(); }
    }

    // Temperature (°C)
    {
        int8_t tC = 0;
        if (readTempC(tC)) s.tx = (float)tC;
        else s.tx = std::numeric_limits<float>::quiet_NaN();
    }

    // Calibration (0..3)
    {
        Calib c{};
        if (readCalib(c)) {
            s.calib_sys   = (uint8_t)c.sys;
            s.calib_gyro  = (uint8_t)c.gyro;
            s.calib_accel = (uint8_t)c.accel;
            s.calib_mag   = (uint8_t)c.mag;
        } else {
            s.calib_sys = s.calib_gyro = s.calib_accel = s.calib_mag = 0u;
        }
    }

    return s;
}

// Optional: soft reset (chip reboots, needs re-begin())
bool BNO055::reset() {
    if (!setMode(MODE_CONFIG)) return false;
    if (!write8_(REG_SYS_TRIGGER, 0x20)) return false; // RST_SYS
    sleepMs_(650);
    return true;
}


// ---- low-level ----
bool BNO055::openBus_() {
    if (fd_ >= 0) return true;
    fd_ = ::open(dev_, O_RDWR);
    if (fd_ < 0) { perror("open(i2c)"); return false; }
    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) { perror("ioctl(I2C_SLAVE)"); ::close(fd_); fd_=-1; return false; }
    return true;
}

bool BNO055::write8_(uint8_t reg, uint8_t val) {
    uint8_t buf[2]{reg, val};
    if (::write(fd_, buf, 2) != 2) { perror("i2c write"); return false; }
    return true;
}

bool BNO055::read8_(uint8_t reg, uint8_t& out) {
    if (::write(fd_, &reg, 1) != 1) { perror("i2c write(reg)"); return false; }
    if (::read(fd_, &out, 1) != 1)   { perror("i2c read8"); return false; }
    return true;
}

bool BNO055::readN_(uint8_t reg, uint8_t* dst, size_t n) {
    if (::write(fd_, &reg, 1) != 1) { perror("i2c write(reg)"); return false; }
    if (::read(fd_, dst, n) != (ssize_t)n) { perror("i2c readN"); return false; }
    return true;
}

void BNO055::sleepMs_(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

#endif