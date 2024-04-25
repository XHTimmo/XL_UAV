#include <stdio.h>
#include "MPU6050.h"
#include <time.h>
#include <esp_log.h>
#include "../IIC/include/IIC.h"
#include <math.h>
#include "unity.h"

static const char *TAG = "MPU";
static void *mpu6050 = NULL;
typedef struct
{
    i2c_port_t bus;
    gpio_num_t int_pin;
    uint16_t dev_addr;
    uint32_t counter;
    float dt;
    struct timeval *timer;
} mpu6050_dev_t;

/***************************************************************************************************
*函数：static esp_err_t mpu6050_read(void* sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
*功能：MPU6050读函数
*参数：
        *sensor mpu6050句柄
        const uint8_t reg_start_addr 读目标的寄存器起始地址
        uint8_t *const data_buf 指向读取的数据存储的buf的指针
        const uint8_t data_len 读取的数据长度
*返回值：错误代码
*备注：
***************************************************************************************************/
static esp_err_t mpu6050_read(void *sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_READ, true);
    assert(ESP_OK == ret);
    ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}
/***************************************************************************************************
*函数：static esp_err_t mpu6050_write(void *sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
*功能：MPU6050读函数
*参数：
        *sensor mpu6050句柄
        const uint8_t reg_start_addr 写目标的寄存器起始地址
        uint8_t *const data_buf 指向写入的数据存储的buf的指针
        const uint8_t data_len 写入的数据长度
*返回值：错误代码
*备注：
***************************************************************************************************/
static esp_err_t mpu6050_write(void *sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static MPU6050_handle mpu6050_create(i2c_port_t port, const uint16_t dev_addr)
{
    mpu6050_dev_t *sensor = (mpu6050_dev_t *)calloc(1, sizeof(mpu6050_dev_t));
    sensor->bus = port;
    sensor->dev_addr = dev_addr << 1;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *)calloc(1, sizeof(struct timeval));
    return (MPU6050_handle)sensor;
}

static esp_err_t mpu6050_config(void *sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
{
    uint8_t config_regs[2] = {gyro_fs << 3, acce_fs << 3};
    return mpu6050_write(sensor, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
}

void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_IIC_ADDR);
    if (mpu6050 != NULL)
    {
        ESP_LOGI(TAG, "successful creat a mpu6050 handle");
    }
    else
    {
        ESP_LOGE(TAG, "fail creat a mpu6050 handle");
    }

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_FAIL)
    {
        ESP_LOGI(TAG, "successful config mpu6050");
        ESP_LOGI(TAG,"ACCE_FS_%d,GYRO_FS_%dDPS",(int)(pow(2,(ACCE_FS_4G+1))),(int)(250*pow(2,(ACCE_FS_4G))));
    }
    else
    {
        ESP_LOGE(TAG, "fail to config mpu6050");
    }

    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}