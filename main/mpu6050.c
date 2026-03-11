#include "mpu6050.h"
#include "driver/i2c.h"

#define MPU6050_ADDR 0x68

#define I2C_MASTER_SCL_IO           6          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           5          /*!< GPIO number used for I2C master data */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master device */
#define I2C_MASTER_FREQ_HZ          400000      /*!< I2C master clock frequency */

void i2c_master_init(void)
{
    esp_err_t err;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE("mpu6050", "i2c_param_config failed: %s", esp_err_to_name(err));
    }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE("mpu6050", "i2c_driver_install failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("mpu6050", "i2c master initialized on SDA=%d SCL=%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    }
}

// MPU6050 register addresses
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43

esp_err_t mpu6050_init(void) {
    uint8_t data[2] = {MPU6050_REG_PWR_MGMT_1, 0x00}; // wake up device

    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        MPU6050_ADDR,
        data,
        2,
        100 / portTICK_PERIOD_MS
    );
    if (err != ESP_OK) {
        ESP_LOGE("mpu6050", "wake-up write failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("mpu6050", "MPU6050 awakened at addr 0x%02x", MPU6050_ADDR);
    }
    return err;
}


// Read accelerometer and gyro registers. Each value is 16-bit big endian.
void mpu6050_read(int16_t* accel, int16_t* gyro) {
    // read 14 bytes starting from ACCEL_XOUT_H (accel 6, temp 2, gyro 6)
    uint8_t buf[14];
    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        MPU6050_ADDR,
        (uint8_t[]){MPU6050_REG_ACCEL_XOUT_H},
        1,
        buf,
        sizeof(buf),
        100 / portTICK_PERIOD_MS
    );
    if (err != ESP_OK) {
        ESP_LOGE("mpu6050", "read failed: %s", esp_err_to_name(err));
        return;
    }

    // convert to signed 16-bit
    for (int i = 0; i < 3; ++i) {
        accel[i] = (int16_t)((buf[2*i] << 8) | buf[2*i + 1]);
    }
    for (int i = 0; i < 3; ++i) {
        gyro[i] = (int16_t)((buf[8 + 2*i] << 8) | buf[8 + 2*i + 1]);
    }
}