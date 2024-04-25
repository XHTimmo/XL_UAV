#include <stdio.h>
#include "IIC.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "unity.h"

static const char *TAG = "IIC";

void i2c_bus_init(void)
{
    ESP_LOGI(TAG, "----------IIC BUS init begin----------");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    esp_err_t ret;
    ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_FAIL)
    {
        ESP_LOGI(TAG, "1 successful config IIC BUS");
        ESP_LOGI(TAG, "  IIC_PORT:%s SDA:GPIO_NUM_%d SCL:GPIO_NUM_%d",I2C_MASTER_NUM?"IIC_PORT_1":"IIC_PORT_0",I2C_MASTER_SDA_IO,I2C_MASTER_SCL_IO);
    }
    else
    {
        ESP_LOGE(TAG, "1 fail to config mpu6050");
    }
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_FAIL)
    {
        ESP_LOGI(TAG, "2 successful install IIC BUS");
    }
    else
    {
        ESP_LOGE(TAG, "2 fail to install mpu6050");
    }
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ESP_LOGI(TAG, "----------IIC BUS init begin----------\n");
}