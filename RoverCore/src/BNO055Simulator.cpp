#include "BNO055.hpp"

#ifndef __linux__

BNO055::BNO055(const char* i2c_dev, int addr7) {
}

BNO055::~BNO055() {

}

bool BNO055::begin(bool useExtCrystal, Mode mode) {
    return true;
}

void BNO055::close() {

}

bool BNO055::setMode(Mode m) {
    return true;
}

// ---- reads ----
bool BNO055::readCalib(Calib& c) {
    return true;
}

bool BNO055::readTempC(int8_t& tC) {
    return true;
}

bool BNO055::readEuler(Euler& e) {
    return true;
}

bool BNO055::readQuaternion(float& w, float& x, float& y, float& z) {
    return true;
}

bool BNO055::readAccel(Vec3& a) {       // m/s^2
    return true;
}

bool BNO055::readGyro(Vec3& g) {        // dps
    return true;
}

bool BNO055::readLinearAccel(Vec3& v) { // m/s^2, gravity removed
    return true;
}

bool BNO055::readGravity(Vec3& v) {     // m/s^2 gravity vector
    return true;
}

BNO055::Sample BNO055::readSample() {
    BNO055::Sample s{};
    return s;
}

// Optional: soft reset (chip reboots, needs re-begin())
bool BNO055::reset() {
    return true;
}

// ---- low-level ----
bool BNO055::openBus_() {
    return true;
}

bool BNO055::write8_(uint8_t reg, uint8_t val) {
    return true;
}

bool BNO055::read8_(uint8_t reg, uint8_t& out) {
    return true;
}

bool BNO055::readN_(uint8_t reg, uint8_t* dst, size_t n) {
    return true;
}

void BNO055::sleepMs_(int ms) {
    
}

#endif // __linux__