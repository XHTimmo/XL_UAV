#include <stdio.h>
#include "MPU6050.h"
#include <time.h>
#include <esp_log.h>
#include "../IIC/include/IIC.h"
#include <math.h>
#include "unity.h"

static const char *TAG = "MPU";
static void *mpu6050 = NULL;

static mpu6050_raw_data_t zero_acce = {
    .x = 0,
    .y = 0,
    .z = 0,
};
static mpu6050_raw_data_t zero_gyro = {
    .x = 0,
    .y = 0,
    .z = 0,
};

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

/***************************************************************************************************
 *函数：MPU6050_handle mpu6050_create(i2c_port_t port, const uint16_t dev_addr)
 *功能：创建MPU6050句柄
 *参数：
 *i2c_port_t port   MPU6050IIC端口
 *const uint16_t dev_addr   MPU6050IIC地址
 *返回值：指向MPU6050模块信息句柄的指针
 *备注：创建包含了MPU6050模块信息的句柄
 ***************************************************************************************************/
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

/***************************************************************************************************
 *函数：esp_err_t mpu6050_config(void *sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
 *功能：配置MPU6050量程
 *参数：
 *void *sensor  指向MPU6050句柄的指针
 *const mpu6050_acce_fs_t acce_fs   acce量程配置项
 *const mpu6050_gyro_fs_t gyro_fs   gyro量程配置项
 *返回值：错误值
 *备注：
 ***************************************************************************************************/
static esp_err_t mpu6050_config(void *sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
{
    uint8_t config_regs[2] = {gyro_fs << 3, acce_fs << 3};
    return mpu6050_write(sensor, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
}

/***************************************************************************************************
 *函数：esp_err_t mpu6050_wake_up(void *sensor)
 *功能：唤醒MPU6050
 *参数：
 *void *sensor指向MPU6050句柄的指针
 *返回值：无
 *备注：
 ***************************************************************************************************/
static esp_err_t mpu6050_wake_up(void *sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret)
    {
        return ret;
    }
    tmp &= (~BIT6);
    ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

/***************************************************************************************************
 *函数：esp_err_t mpu6050_get_deviceid(void *sensor, uint8_t *const deviceid)
 *功能：获取MPU6050ID
 *参数：
 *void *sensor  指向MPU6050句柄的指针
 *uint8_t *const deviceid   指向MPU6050设备ID的指针
 *返回值：错误值
 *备注：
 ***************************************************************************************************/
esp_err_t mpu6050_get_deviceid(void *sensor, uint8_t *const deviceid)
{
    return mpu6050_read(sensor, MPU6050_WHO_AM_I, deviceid, 1);
}

/***************************************************************************************************
 *函数：esp_err_t mpu6050_get_raw_acce(void * sensor, mpu6050_raw_acce_value_t *const raw_acce_value)
 *功能：获取ACCE的原始值
 *参数：
 *sensor mpu6050句柄
 *const raw_acce_value 指向加速度原始值的指针
 *返回值：错误代码
 *备注：获取ACCE加速的原始值
 ***************************************************************************************************/
esp_err_t mpu6050_get_raw_acce(void *sensor, mpu6050_raw_acce_value_t *const raw_acce_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

/***************************************************************************************************
 *函数：esp_err_t mpu6050_get_raw_gyro(void * sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value)
 *功能：获取GYRO的原始值
 *参数：
 *sensor mpu6050句柄
 *const raw_gyro_value 指向角加速度原始值的指针
 *返回值：错误代码
 *备注：获取GYRO角加速的原始值
 ***************************************************************************************************/
esp_err_t mpu6050_get_raw_gyro(void *sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd));

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));

    return ret;
}
static void mpu6050_get_raw_zero(void *sensor)
{
    esp_err_t ret;
    mpu6050_raw_acce_value_t zero_raw_acce_data;
    mpu6050_raw_gyro_value_t zero_raw_gyro_data;

    for (uint8_t i = 0; i < 200; i++)
    {
        ret = mpu6050_get_raw_acce(mpu6050, &zero_raw_acce_data);
        if (ret == ESP_OK)
        {
            // ESP_LOGI(TAG, "x_acce:%5d\ty_acce:%5d\tz_acce:%5d", zero_raw_acce_data.raw_acce_x, zero_raw_acce_data.raw_acce_y, zero_raw_acce_data.raw_acce_z);
            zero_acce.x += zero_raw_acce_data.raw_acce_x;
            zero_acce.y += zero_raw_acce_data.raw_acce_y;
            zero_acce.z += zero_raw_acce_data.raw_acce_z;
        }
        else
        {
            ESP_LOGW(TAG, "fail to get raw_acce");
        }
        ret = mpu6050_get_raw_gyro(mpu6050, &zero_raw_gyro_data);
        if (ret == ESP_OK)
        {
            // ESP_LOGI(TAG, "x_gyro:%5d\ty_gyro:%5d\tz_gyro:%5d", zero_raw_gyro_data.raw_gyro_x, zero_raw_gyro_data.raw_gyro_y, zero_raw_gyro_data.raw_gyro_z);
            zero_gyro.x += zero_raw_gyro_data.raw_gyro_x;
            zero_gyro.y += zero_raw_gyro_data.raw_gyro_y;
            zero_gyro.z += zero_raw_gyro_data.raw_gyro_z;
        }
        else
        {
            ESP_LOGW(TAG, "fail to get raw_gyro");
        }

        // vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    zero_acce.x /= 200;
    zero_acce.y /= 200;
    zero_acce.z /= 200;

    zero_gyro.x /= 200;
    zero_gyro.y /= 200;
    zero_gyro.z /= 200;
    ESP_LOGI(TAG, "5 successful get raw zero");
    ESP_LOGI(TAG, "  x_acce_zero:%5.5lf\ty_acce_zero:%5.5lf\tz_acce_zero:%5.5lf", zero_acce.x, zero_acce.y, zero_acce.z);
    ESP_LOGI(TAG, "  x_gyro_zero:%5.5lf\ty_gyro_zero:%5.5lf\tz_gyro_zero:%5.5lf", zero_gyro.x, zero_gyro.y, zero_gyro.z);
}
/***************************************************************************************************
 *函数：void i2c_sensor_mpu6050_init(void)
 *功能：初始化MPU6050
 *参数：
 *返回值：无
 *备注：包含创建MPU6050模块、配置量程、唤醒模块、获取零漂
 ***************************************************************************************************/
void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "----------Begin init MPU6050----------");
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_IIC_ADDR);
    if (mpu6050 != NULL)
    {
        ESP_LOGI(TAG, "1 successful creat a mpu6050 handle");
    }
    else
    {
        ESP_LOGE(TAG, "1 fail creat a mpu6050 handle");
    }

    ret = mpu6050_config(mpu6050, ACCE_FS_2G, GYRO_FS_500DPS);
    if (ret != ESP_FAIL)
    {
        ESP_LOGI(TAG, "2 successful config mpu6050");
        ESP_LOGI(TAG, "  ACCE_FS_%dG GYRO_FS_%dDPS", (int)(pow(2, (ACCE_FS_2G + 1))), (int)(250 * pow(2, (GYRO_FS_500DPS))));
    }
    else
    {
        ESP_LOGE(TAG, "2 fail to config mpu6050");
    }
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_FAIL)
    {
        ESP_LOGI(TAG, "3 successful wake up mpu6050");
    }
    else
    {
        ESP_LOGE(TAG, "3 fail to wake up mpu6050");
    }
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    uint8_t mpu6050_deviceid = 0;
    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    if (ret != ESP_FAIL)
    {
        ESP_LOGI(TAG, "4 successful get id from mpu6050");
        if (mpu6050_deviceid == MPU6050_WHO_AM_I_VAL)
        {
            ESP_LOGI(TAG, "  MPU6050 ID:%2.x", mpu6050_deviceid);
        }
        else
        {
            ESP_LOGW(TAG, "  MPU6050 ID:%2.x is not as MPU6050_WHO_AM_I_VAL:%2.x", mpu6050_deviceid, MPU6050_WHO_AM_I_VAL);
        }
    }
    else
    {
        ESP_LOGE(TAG, "4 fail to get id from mpu6050");
    }
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    mpu6050_get_raw_zero(mpu6050);

    ESP_LOGI(TAG, "----------End init MPU6050----------\n");
}

mpu6050_raw_data_return_t mpu6050_get_raw_data(void)
{
    esp_err_t ret;
    mpu6050_raw_data_return_t ret_data;
    mpu6050_raw_acce_value_t raw_acce_data;
    mpu6050_raw_gyro_value_t raw_gyro_data;
    ret = mpu6050_get_raw_acce(mpu6050, &raw_acce_data);
    ret = mpu6050_get_raw_gyro(mpu6050, &raw_gyro_data);
    ret_data.x1 = raw_acce_data.raw_acce_x-zero_acce.x;
    ret_data.y1 = raw_acce_data.raw_acce_y-zero_acce.y;
    ret_data.z1 = raw_acce_data.raw_acce_z-zero_acce.z;
    ret_data.x2 = raw_gyro_data.raw_gyro_x-zero_gyro.x;
    ret_data.y2 = raw_gyro_data.raw_gyro_y-zero_gyro.y;
    ret_data.z2 = raw_gyro_data.raw_gyro_z-zero_gyro.z;

    return ret_data;
}