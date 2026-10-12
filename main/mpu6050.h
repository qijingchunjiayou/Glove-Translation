#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define MPU6050_DEFAULT_ADDRESS 0x68

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temperature_c;
} mpu6050_data_t;

typedef struct {
    i2c_master_dev_handle_t device;
} mpu6050_handle_t;

esp_err_t mpu6050_init(mpu6050_handle_t *handle,
                       i2c_master_bus_handle_t bus_handle,
                       uint8_t address,
                       uint32_t scl_speed_hz);
esp_err_t mpu6050_read(mpu6050_handle_t *handle, mpu6050_data_t *data);
