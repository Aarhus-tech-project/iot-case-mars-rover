#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>

class BNO055 {
public:
  struct Sample
  {
    float w, x, y, z; // quaternion
    float ax, ay, az; // acceleration m/s²
    float gx, gy, gz; // gyro dps
    float mx, my, mz; // magnetometer uT
    float eX, eY, eZ; // Euler angles degrees
    float lx, ly, lz; // linear acceleration m/s²
    float tx;         // temperature °C
    uint8_t calib_sys, calib_gyro, calib_accel, calib_mag; // 0..3
    uint64_t time_ns; // monotonic timestamp
  };
  
  // ---- public enums/structs ----
  enum Mode : uint8_t {
    MODE_CONFIG = 0x00,
    MODE_ACCONLY = 0x01,
    MODE_MAGONLY = 0x02,
    MODE_GYRONLY = 0x03,
    MODE_ACCMAG  = 0x04,
    MODE_ACCGYRO = 0x05,
    MODE_MAGGYRO = 0x06,
    MODE_AMG     = 0x07,
    MODE_IMU     = 0x08,
    MODE_COMPASS = 0x09,
    MODE_M4G     = 0x0A,
    MODE_NDOF_FMC_OFF = 0x0B,
    MODE_NDOF    = 0x0C
  };

  struct Calib { uint8_t sys, gyro, accel, mag; };      // 0..3 each
  struct Euler { float heading_deg, roll_deg, pitch_deg; };
  struct Vec3  { float x, y, z; };

  // ---- lifecycle ----
  BNO055(const char* i2c_dev = "/dev/i2c-1", int addr7 = 0x29);
  ~BNO055();

  bool begin(bool useExtCrystal = false, Mode mode = MODE_NDOF);

  void close();

  bool setMode(Mode m);

  // ---- reads ----
  bool readCalib(Calib& c);

  bool readTempC(int8_t& tC);

  bool readEuler(Euler& e);

  bool readQuaternion(float& w, float& x, float& y, float& z);

  bool readAccel(Vec3& a);

  bool readGyro(Vec3& g);

  bool readLinearAccel(Vec3& v);

  bool readGravity(Vec3& v);

  BNO055::Sample readSample();

  // Optional: soft reset (chip reboots, needs re-begin())
  bool reset();
private:
  // ---- low-level ----
  bool openBus_();
  
  bool write8_(uint8_t reg, uint8_t val);

  bool read8_(uint8_t reg, uint8_t& out);

  bool readN_(uint8_t reg, uint8_t* dst, size_t n);

  static void sleepMs_(int ms);

  // ---- registers (page 0) ----
  static constexpr uint8_t REG_CHIP_ID         = 0x00;
  static constexpr uint8_t REG_PAGE_ID         = 0x07;
  static constexpr uint8_t REG_ACC_DATA_X_LSB  = 0x08;
  static constexpr uint8_t REG_MAG_DATA_X_LSB  = 0x0E;
  static constexpr uint8_t REG_GYR_DATA_X_LSB  = 0x14;
  static constexpr uint8_t REG_EUL_HEADING_LSB = 0x1A;
  static constexpr uint8_t REG_QUA_W_LSB       = 0x20;
  static constexpr uint8_t REG_LIA_DATA_X_LSB  = 0x28;
  static constexpr uint8_t REG_GRV_DATA_X_LSB  = 0x2E;
  static constexpr uint8_t REG_TEMP            = 0x34;
  static constexpr uint8_t REG_CALIB_STAT      = 0x35;
  static constexpr uint8_t REG_UNIT_SEL        = 0x3B;
  static constexpr uint8_t REG_OPR_MODE        = 0x3D;
  static constexpr uint8_t REG_PWR_MODE        = 0x3E;
  static constexpr uint8_t REG_SYS_TRIGGER     = 0x3F;

  int   fd_;
  int   addr_;
  char  dev_[32];
};