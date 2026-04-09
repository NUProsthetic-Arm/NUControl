#ifndef LSM6DSV_HPP
#define LSM6DSV_HPP

#include <Wire.h>
#include "lsm6dsv_reg.h"

#define I2C_SDA 18
#define I2C_SCL 19
#define LSM6DSV_ADDR 0x6A

/// \brief Struct for holding 3 axis acceleration data
struct accelerations
{
  /// \brief 3 axis acceleration data
  float x, y, z;
};

/// \brief Class abstraction of functions required to use LSM6DSV_IMU
class LSM6DSV_IMU
{
public:
  /// \brief Create empty initialization of LSM6DSV_IMU class
  LSM6DSV_IMU();

  /// \brief Initialize pins, check for heartbeat, and prepare device to collect data
  void init();

  /// \brief Get accelerations from sensor
  /// \return The acceleration reading as acceleration struct
  accelerations get_acc();

private:
  static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
  {
    Wire.beginTransmission(LSM6DSV_ADDR);
    Wire.write(reg);
    Wire.write(bufp, len);
    return Wire.endTransmission();
  }

  static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
  {
    Wire.beginTransmission(LSM6DSV_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(LSM6DSV_ADDR, len);
    for (uint16_t i = 0; i < len; i++) {
      bufp[i] = Wire.read();
    }
    return 0;
  }

  uint8_t whoami;
  stmdev_ctx_t dev_ctx;
  int16_t accel_raw[3];
  // float accel_mg[3];
};

#endif
