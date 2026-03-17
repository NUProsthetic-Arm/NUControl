#include "lsm6dsv.hpp"

LSM6DSV_IMU::LSM6DSV_IMU()
{    
}

void LSM6DSV_IMU::init()
{
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  
  /* Initialize driver context */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg  = platform_read;
  dev_ctx.handle    = NULL;
  
  lsm6dsv_device_id_get(&dev_ctx, &whoami);
  
  if (whoami != LSM6DSV_ID) {
      Serial.println("LSM6DSVTR not found!");
      while (1);
  }
  
  Serial.println("LSM6DSVTR connected");
  
  /* Reset device */
  lsm6dsv_sh_reset_set(&dev_ctx, PROPERTY_ENABLE);
  delay(100);
  lsm6dsv_sh_reset_set(&dev_ctx, &whoami);
  
  /* Enable Block Data Update */
  lsm6dsv_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  
  /* Set accelerometer:
      - ODR: 1.66 kHz
      - FS: ±2 g
  */
  lsm6dsv_xl_full_scale_set(&dev_ctx, LSM6DSV_2g);
  lsm6dsv_xl_data_rate_set(&dev_ctx, LSM6DSV_ODR_HA01_AT_2000Hz);
}

accelerations LSM6DSV_IMU::get_acc()
{
 lsm6dsv_acceleration_raw_get(&dev_ctx, accel_raw);
 return {lsm6dsv_from_fs2_to_mg(accel_raw[0]) / 1000.0f, lsm6dsv_from_fs2_to_mg(accel_raw[1]) / 1000.0f, lsm6dsv_from_fs2_to_mg(accel_raw[2]) / 1000.0f};
}

