idf_component_register(
    SRCS "main.c" "tca9548a.c" "mpu6050.c" "ssd1306.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c
)
