#include "mpu6050.h"

#include <stddef.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75
#define MPU6050_TIMEOUT_MS        100

static const char *TAG = "mpu6050";

static esp_err_t write_register(mpu6050_handle_t *handle,
                                uint8_t reg, uint8_t value)
{
    const uint8_t packet[] = {reg, value};
    return i2c_master_transmit(handle->device, packet, sizeof(packet),
                               MPU6050_TIMEOUT_MS);
}

static esp_err_t read_registers(mpu6050_handle_t *handle, uint8_t reg,
                                uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(handle->device, &reg, 1, data, length,
                                       MPU6050_TIMEOUT_MS);
}

static int16_t read_be16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

esp_err_t mpu6050_init(mpu6050_handle_t *handle,
                       i2c_master_bus_handle_t bus_handle,
                       uint8_t address,
                       uint32_t scl_speed_hz)
{
    ESP_RETURN_ON_FALSE(handle != NULL && bus_handle != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "参数无效");

    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = scl_speed_hz,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_handle, &config,
                                                   &handle->device),
                        TAG, "添加 I2C 设备失败");

    uint8_t who_am_i = 0;
    ESP_RETURN_ON_ERROR(read_registers(handle, MPU6050_REG_WHO_AM_I,
                                        &who_am_i, 1),
                        TAG, "读取 WHO_AM_I 失败");
    /*
     * 0x68: MPU6050
     * 0x70: MPU6500/部分标称 MPU6050 的兼容模块
     * 两者本测试使用的采样寄存器和 ±2g/±250 dps 配置兼容。
     */
    ESP_RETURN_ON_FALSE(who_am_i == 0x68 || who_am_i == 0x70,
                        ESP_ERR_NOT_FOUND, TAG,
                        "不支持的 WHO_AM_I: 0x%02X", who_am_i);

    ESP_RETURN_ON_ERROR(write_register(handle, MPU6050_REG_PWR_MGMT_1, 0x00),
                        TAG, "唤醒失败");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(write_register(handle, MPU6050_REG_CONFIG, 0x03),
                        TAG, "配置低通滤波失败");
    ESP_RETURN_ON_ERROR(write_register(handle, MPU6050_REG_GYRO_CONFIG, 0x00),
                        TAG, "配置陀螺仪失败");
    ESP_RETURN_ON_ERROR(write_register(handle, MPU6050_REG_ACCEL_CONFIG, 0x00),
                        TAG, "配置加速度计失败");

    ESP_LOGI(TAG, "初始化成功，WHO_AM_I=0x%02X，量程: ±2g / ±250 dps",
             who_am_i);
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_handle_t *handle, mpu6050_data_t *data)
{
    ESP_RETURN_ON_FALSE(handle != NULL && handle->device != NULL && data != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "参数无效");

    uint8_t raw[14];
    ESP_RETURN_ON_ERROR(read_registers(handle, MPU6050_REG_ACCEL_XOUT_H,
                                        raw, sizeof(raw)),
                        TAG, "读取采样数据失败");

    data->accel_x_g = read_be16(&raw[0]) / 16384.0f;
    data->accel_y_g = read_be16(&raw[2]) / 16384.0f;
    data->accel_z_g = read_be16(&raw[4]) / 16384.0f;
    data->temperature_c = read_be16(&raw[6]) / 340.0f + 36.53f;
    data->gyro_x_dps = read_be16(&raw[8]) / 131.0f;
    data->gyro_y_dps = read_be16(&raw[10]) / 131.0f;
    data->gyro_z_dps = read_be16(&raw[12]) / 131.0f;
    return ESP_OK;
}
