#include <inttypes.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tca9548a.h"
#include "mpu6050.h"
#include "ssd1306.h"

/*
 * ESP32-S3 <-> TCA9548A test wiring
 *
 * GPIO8  -> SDA
 * GPIO9  -> SCL
 * 3V3    -> VIN/VCC
 * GND    -> GND
 * A0/A1/A2 -> GND (device address 0x70)
 * RESET  -> 3V3 (the module normally already pulls this pin high)
 *
 * Change only these definitions if your board uses different pins.
 */
#define TEST_I2C_SDA_GPIO       GPIO_NUM_8
#define TEST_I2C_SCL_GPIO       GPIO_NUM_9
#define TEST_I2C_PORT           I2C_NUM_0
#define TEST_I2C_FREQ_HZ        100000
#define TEST_PROBE_TIMEOUT_MS   20
#define MPU6050_MUX_CHANNEL     0
#define SSD1306_MUX_CHANNEL     1
#define DISPLAY_REFRESH_MS      250

static const char *TAG = "tca9548a_test";

static esp_err_t create_i2c_bus(i2c_master_bus_handle_t *bus_handle)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = TEST_I2C_PORT,
        .sda_io_num = TEST_I2C_SDA_GPIO,
        .scl_io_num = TEST_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    return i2c_new_master_bus(&bus_config, bus_handle);
}

static void scan_selected_channel(i2c_master_bus_handle_t bus_handle,
                                  uint8_t channel)
{
    unsigned int found = 0;

    ESP_LOGI(TAG, "扫描 TCA9548A 通道 %" PRIu8 "...", channel);
    for (uint8_t address = 0x08; address <= 0x77; ++address) {
        /* TCA9548A 本身位于上游总线，不属于任何下游通道。 */
        if (address == TCA9548A_DEFAULT_ADDRESS) {
            continue;
        }

        if (i2c_master_probe(bus_handle, address, TEST_PROBE_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGI(TAG, "  通道 %" PRIu8 " 发现设备: 0x%02X", channel, address);
            ++found;
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "  通道 %" PRIu8 " 未发现下游 I2C 设备", channel);
    } else {
        ESP_LOGI(TAG, "  通道 %" PRIu8 " 共发现 %u 个设备", channel, found);
    }
}

void app_main(void)
{
    i2c_master_bus_handle_t bus_handle = NULL;
    static tca9548a_handle_t mux;
    static mpu6050_handle_t mpu;
    static ssd1306_handle_t display;

    ESP_LOGI(TAG, "ESP32-S3 TCA9548A 单模块测试开始");
    ESP_LOGI(TAG, "接线: SDA=GPIO%d, SCL=GPIO%d, 频率=%d Hz",
             TEST_I2C_SDA_GPIO, TEST_I2C_SCL_GPIO, TEST_I2C_FREQ_HZ);

    ESP_ERROR_CHECK(create_i2c_bus(&bus_handle));

    esp_err_t ret = tca9548a_init(&mux, bus_handle,
                                  TCA9548A_DEFAULT_ADDRESS,
                                  TEST_I2C_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,
                 "未检测到 TCA9548A (0x%02X)，请检查供电、地址和 SDA/SCL 接线",
                 TCA9548A_DEFAULT_ADDRESS);
        return;
    }

    for (uint8_t channel = 0; channel < TCA9548A_CHANNEL_COUNT; ++channel) {
        ret = tca9548a_select_channel(&mux, channel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "选择通道 %" PRIu8 " 失败: %s",
                     channel, esp_err_to_name(ret));
            continue;
        }
        scan_selected_channel(bus_handle, channel);
    }

    ESP_ERROR_CHECK(tca9548a_select_channel(&mux, MPU6050_MUX_CHANNEL));
    ESP_ERROR_CHECK(mpu6050_init(&mpu, bus_handle, MPU6050_DEFAULT_ADDRESS,
                                 TEST_I2C_FREQ_HZ));

    ESP_ERROR_CHECK(tca9548a_select_channel(&mux, SSD1306_MUX_CHANNEL));
    ESP_ERROR_CHECK(ssd1306_init(&display, bus_handle, SSD1306_DEFAULT_ADDRESS,
                                 TEST_I2C_FREQ_HZ));

    ESP_LOGI(TAG, "开始持续采集 MPU6050 并刷新 OLED");
    while (true) {
        mpu6050_data_t data;
        ESP_ERROR_CHECK(tca9548a_select_channel(&mux, MPU6050_MUX_CHANNEL));
        esp_err_t read_result = mpu6050_read(&mpu, &data);
        if (read_result != ESP_OK) {
            ESP_LOGE(TAG, "MPU6050 读取失败: %s", esp_err_to_name(read_result));
            vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_MS));
            continue;
        }

        char line[22];
        ssd1306_clear(&display);
        ssd1306_draw_text(&display, 0, 0, "MPU6050 LIVE");
        snprintf(line, sizeof(line), "AX:%+.2f G", data.accel_x_g);
        ssd1306_draw_text(&display, 0, 1, line);
        snprintf(line, sizeof(line), "AY:%+.2f G", data.accel_y_g);
        ssd1306_draw_text(&display, 0, 2, line);
        snprintf(line, sizeof(line), "AZ:%+.2f G", data.accel_z_g);
        ssd1306_draw_text(&display, 0, 3, line);
        snprintf(line, sizeof(line), "GX:%+.1f D/S", data.gyro_x_dps);
        ssd1306_draw_text(&display, 0, 4, line);
        snprintf(line, sizeof(line), "GY:%+.1f D/S", data.gyro_y_dps);
        ssd1306_draw_text(&display, 0, 5, line);
        snprintf(line, sizeof(line), "GZ:%+.1f D/S", data.gyro_z_dps);
        ssd1306_draw_text(&display, 0, 6, line);
        snprintf(line, sizeof(line), "T:%.1f C", data.temperature_c);
        ssd1306_draw_text(&display, 0, 7, line);

        ESP_ERROR_CHECK(tca9548a_select_channel(&mux, SSD1306_MUX_CHANNEL));
        esp_err_t display_result = ssd1306_update(&display);
        if (display_result != ESP_OK) {
            ESP_LOGE(TAG, "OLED 刷新失败: %s", esp_err_to_name(display_result));
        }

        ESP_LOGI(TAG, "A[g]=%+.2f,%+.2f,%+.2f G[dps]=%+.1f,%+.1f,%+.1f T=%.1fC",
                 data.accel_x_g, data.accel_y_g, data.accel_z_g,
                 data.gyro_x_dps, data.gyro_y_dps, data.gyro_z_dps,
                 data.temperature_c);
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_MS));
    }

}
