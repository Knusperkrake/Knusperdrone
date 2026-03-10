#include "mpu6050.h"
#include "driver/i2c.h"

#define MPU6050_ADDR 0x68

#define I2C_MASTER_SCL_IO           6          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           5          /*!< GPIO number used for I2C master data */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master device */
#define I2C_MASTER_FREQ_HZ          400000      /*!< I2C master clock frequency */

void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void mpu6050_init() {
    uint8_t data[2] = {0x6b, 0x00};

    i2c_master_write_to_device(
        I2C_NUM_0,
        MPU6050_ADDR,
        data,
        2,
        100 / portTICK_PERIOD_MS
    );
}