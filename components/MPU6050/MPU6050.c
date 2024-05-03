// #include <stdio.h>
// #include "MPU6050.h"
// #include <time.h>
// #include <esp_log.h>
// #include "../IIC/include/IIC.h"
// #include <math.h>
// #include "unity.h"

// static const char *TAG = "MPU";
// MPU6050_handle mpu6050;
// static mpu6050_raw_data_t zero_acce = {
//     .x = 0,
//     .y = 0,
//     .z = 0,
// };
// static mpu6050_raw_data_t zero_gyro = {
//     .x = 0,
//     .y = 0,
//     .z = 0,
// };


// /***************************************************************************************************
// *函数：static esp_err_t mpu6050_read(void* sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
// *功能：MPU6050读函数
// *参数：
//         *sensor mpu6050句柄
//         const uint8_t reg_start_addr 读目标的寄存器起始地址
//         uint8_t *const data_buf 指向读取的数据存储的buf的指针
//         const uint8_t data_len 读取的数据长度
// *返回值：错误代码
// *备注：
// ***************************************************************************************************/
// static esp_err_t mpu6050_read(void *sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
// {
//     mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
//     esp_err_t ret;

//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     ret = i2c_master_start(cmd);
//     // printf("1");
//     assert(ESP_OK == ret);
//     ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
//     // printf("2");
//     assert(ESP_OK == ret);
//     ret = i2c_master_write_byte(cmd, reg_start_addr, true);
//     // printf("3");
//     assert(ESP_OK == ret);
//     ret = i2c_master_start(cmd);
//     // printf("4");
//     assert(ESP_OK == ret);
//     ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_READ, true);
//     // printf("5");
//     assert(ESP_OK == ret);
//     ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
//     // printf("6");
//     assert(ESP_OK == ret);
//     ret = i2c_master_stop(cmd);
//     // printf("7");
//     assert(ESP_OK == ret);
//     ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     // printf("%s\n",ret==0?"ESP_OK":"ESP_FAIL");
//     return ret;
// }
// /***************************************************************************************************
// *函数：static esp_err_t mpu6050_write(void *sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
// *功能：MPU6050读函数
// *参数：
//         *sensor mpu6050句柄
//         const uint8_t reg_start_addr 写目标的寄存器起始地址
//         uint8_t *const data_buf 指向写入的数据存储的buf的指针
//         const uint8_t data_len 写入的数据长度
// *返回值：错误代码
// *备注：
// ***************************************************************************************************/
// static esp_err_t mpu6050_write(void *sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
// {
//     mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
//     esp_err_t ret;

//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     ret = i2c_master_start(cmd);
//     assert(ESP_OK == ret);
//     ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
//     assert(ESP_OK == ret);
//     ret = i2c_master_write_byte(cmd, reg_start_addr, true);
//     assert(ESP_OK == ret);
//     ret = i2c_master_write(cmd, data_buf, data_len, true);
//     assert(ESP_OK == ret);
//     ret = i2c_master_stop(cmd);
//     assert(ESP_OK == ret);
//     ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);

//     return ret;
// }

// /***************************************************************************************************
//  *函数：MPU6050_handle mpu6050_create(i2c_port_t port, const uint16_t dev_addr)
//  *功能：创建MPU6050句柄
//  *参数：
//  *i2c_port_t port   MPU6050IIC端口
//  *const uint16_t dev_addr   MPU6050IIC地址
//  *返回值：指向MPU6050模块信息句柄的指针
//  *备注：创建包含了MPU6050模块信息的句柄
//  ***************************************************************************************************/
// static MPU6050_handle mpu6050_create(i2c_port_t port, const uint16_t dev_addr)
// {
//     mpu6050_dev_t *sensor = (mpu6050_dev_t *)calloc(1, sizeof(mpu6050_dev_t));
//     sensor->bus = port;
//     sensor->dev_addr = dev_addr << 1;
//     sensor->counter = 0;
//     sensor->dt = 0;
//     sensor->timer = (struct timeval *)calloc(1, sizeof(struct timeval));
//     return (MPU6050_handle)sensor;
// }

// /***************************************************************************************************
//  *函数：esp_err_t mpu6050_config(void *sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
//  *功能：配置MPU6050量程
//  *参数：
//  *void *sensor  指向MPU6050句柄的指针
//  *const mpu6050_acce_fs_t acce_fs   acce量程配置项
//  *const mpu6050_gyro_fs_t gyro_fs   gyro量程配置项
//  *返回值：错误值
//  *备注：
//  ***************************************************************************************************/
// static esp_err_t mpu6050_config(void *sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
// {
//     uint8_t config_regs[2] = {gyro_fs << 3, acce_fs << 3};
//     return mpu6050_write(sensor, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
// }

// /***************************************************************************************************
//  *函数：esp_err_t mpu6050_wake_up(void *sensor)
//  *功能：唤醒MPU6050
//  *参数：
//  *void *sensor指向MPU6050句柄的指针
//  *返回值：无
//  *备注：
//  ***************************************************************************************************/
// static esp_err_t mpu6050_wake_up(void *sensor)
// {
//     esp_err_t ret;
//     uint8_t tmp;
//     ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
//     if (ESP_OK != ret)
//     {
//         return ret;
//     }
//     tmp &= (~BIT6);
//     ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
//     return ret;
// }

// /***************************************************************************************************
//  *函数：esp_err_t mpu6050_get_deviceid(void *sensor, uint8_t *const deviceid)
//  *功能：获取MPU6050ID
//  *参数：
//  *void *sensor  指向MPU6050句柄的指针
//  *uint8_t *const deviceid   指向MPU6050设备ID的指针
//  *返回值：错误值
//  *备注：
//  ***************************************************************************************************/
// esp_err_t mpu6050_get_deviceid(void *sensor, uint8_t *const deviceid)
// {
//     return mpu6050_read(sensor, MPU6050_WHO_AM_I, deviceid, 1);
// }

// /***************************************************************************************************
//  *函数：esp_err_t mpu6050_get_raw_acce(void * sensor, mpu6050_raw_acce_value_t *const raw_acce_value)
//  *功能：获取ACCE的原始值
//  *参数：
//  *sensor mpu6050句柄
//  *const raw_acce_value 指向加速度原始值的指针
//  *返回值：错误代码
//  *备注：获取ACCE加速的原始值
//  ***************************************************************************************************/
// esp_err_t mpu6050_get_raw_acce(void *sensor, mpu6050_raw_acce_value_t *const raw_acce_value)
// {
//     uint8_t data_rd[6];
//     esp_err_t ret = mpu6050_read(sensor, MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

//     raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
//     raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
//     raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
//     return ret;
// }

// /***************************************************************************************************
//  *函数：esp_err_t mpu6050_get_raw_gyro(void * sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value)
//  *功能：获取GYRO的原始值
//  *参数：
//  *sensor mpu6050句柄
//  *const raw_gyro_value 指向角加速度原始值的指针
//  *返回值：错误代码
//  *备注：获取GYRO角加速的原始值
//  ***************************************************************************************************/
// esp_err_t mpu6050_get_raw_gyro(void *sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value)
// {
//     uint8_t data_rd[6];
//     esp_err_t ret = mpu6050_read(sensor, MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd));

//     raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
//     raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
//     raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));

//     return ret;
// }
// static void mpu6050_get_raw_zero(void *sensor)
// {
//     esp_err_t ret;
//     mpu6050_raw_acce_value_t zero_raw_acce_data;
//     mpu6050_raw_gyro_value_t zero_raw_gyro_data;

//     for (uint8_t i = 0; i < 200; i++)
//     {
//         ret = mpu6050_get_raw_acce(mpu6050, &zero_raw_acce_data);
//         if (ret == ESP_OK)
//         {
//             // ESP_LOGI(TAG, "x_acce:%5d\ty_acce:%5d\tz_acce:%5d", zero_raw_acce_data.raw_acce_x, zero_raw_acce_data.raw_acce_y, zero_raw_acce_data.raw_acce_z);
//             zero_acce.x += zero_raw_acce_data.raw_acce_x;
//             zero_acce.y += zero_raw_acce_data.raw_acce_y;
//             zero_acce.z += zero_raw_acce_data.raw_acce_z;
//         }
//         else
//         {
//             ESP_LOGW(TAG, "fail to get raw_acce");
//         }
//         ret = mpu6050_get_raw_gyro(mpu6050, &zero_raw_gyro_data);
//         if (ret == ESP_OK)
//         {
//             // ESP_LOGI(TAG, "x_gyro:%5d\ty_gyro:%5d\tz_gyro:%5d", zero_raw_gyro_data.raw_gyro_x, zero_raw_gyro_data.raw_gyro_y, zero_raw_gyro_data.raw_gyro_z);
//             zero_gyro.x += zero_raw_gyro_data.raw_gyro_x;
//             zero_gyro.y += zero_raw_gyro_data.raw_gyro_y;
//             zero_gyro.z += zero_raw_gyro_data.raw_gyro_z;
//         }
//         else
//         {
//             ESP_LOGW(TAG, "fail to get raw_gyro");
//         }

//         // vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
//     zero_acce.x /= 200;
//     zero_acce.y /= 200;
//     zero_acce.z /= 200;

//     zero_gyro.x /= 200;
//     zero_gyro.y /= 200;
//     zero_gyro.z /= 200;
//     ESP_LOGI(TAG, "5 successful get raw zero");
//     ESP_LOGI(TAG, "  x_acce_zero:%5.5lf\ty_acce_zero:%5.5lf\tz_acce_zero:%5.5lf", zero_acce.x, zero_acce.y, zero_acce.z);
//     ESP_LOGI(TAG, "  x_gyro_zero:%5.5lf\ty_gyro_zero:%5.5lf\tz_gyro_zero:%5.5lf", zero_gyro.x, zero_gyro.y, zero_gyro.z);
// }
// /***************************************************************************************************
//  *函数：void i2c_sensor_mpu6050_init(void)
//  *功能：初始化MPU6050
//  *参数：
//  *返回值：无
//  *备注：包含创建MPU6050模块、配置量程、唤醒模块、获取零漂
//  ***************************************************************************************************/
// void i2c_sensor_mpu6050_init(void)
// {
//     esp_err_t ret;
//     ESP_LOGI(TAG, "----------Begin init MPU6050----------");
//     mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_IIC_ADDR);
//     if (mpu6050 != NULL)
//     {
//         ESP_LOGI(TAG, "1 successful creat a mpu6050 handle");
//     }
//     else
//     {
//         ESP_LOGE(TAG, "1 fail creat a mpu6050 handle");
//     }


//     ret = mpu6050_config(mpu6050, ACCE_FS_2G, GYRO_FS_500DPS);
//     if (ret != ESP_FAIL)
//     {
//         ESP_LOGI(TAG, "2 successful config mpu6050");
//         ESP_LOGI(TAG, "  ACCE_FS_%dG GYRO_FS_%dDPS", (int)(pow(2, (ACCE_FS_2G + 1))), (int)(250 * pow(2, (GYRO_FS_500DPS))));
//     }
//     else
//     {
//         ESP_LOGE(TAG, "2 fail to config mpu6050");
//     }
//     TEST_ASSERT_EQUAL(ESP_OK, ret);

//     ret = mpu6050_wake_up(mpu6050);
//     if (ret != ESP_FAIL)
//     {
//         ESP_LOGI(TAG, "3 successful wake up mpu6050");
//     }
//     else
//     {
//         ESP_LOGE(TAG, "3 fail to wake up mpu6050");
//     }
//     TEST_ASSERT_EQUAL(ESP_OK, ret);

//     uint8_t mpu6050_deviceid= 0;
//     ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
//     if (ret != ESP_FAIL)
//     {
//         ESP_LOGI(TAG, "4 successful get id from mpu6050");
//         if (mpu6050_deviceid == MPU6050_WHO_AM_I_VAL)
//         {
//             ESP_LOGI(TAG, "  MPU6050 ID:%2.x", mpu6050_deviceid);
//         }
//         else
//         {
//             ESP_LOGW(TAG, "  MPU6050 ID:%2.x is not as MPU6050_WHO_AM_I_VAL:%2.x", mpu6050_deviceid, MPU6050_WHO_AM_I_VAL);
//         }
//     }
//     else
//     {
//         ESP_LOGE(TAG, "4 fail to get id from mpu6050");
//     }
//     TEST_ASSERT_EQUAL(ESP_OK, ret);

//     mpu6050_get_raw_zero(mpu6050);

//     ESP_LOGI(TAG, "----------End init MPU6050----------\n");
// }

// mpu6050_all_raw_data_t mpu6050_get_raw_data(void)
// {
//     esp_err_t ret;
//     mpu6050_all_raw_data_t ret_data;
//     mpu6050_raw_acce_value_t raw_acce_data;
//     mpu6050_raw_gyro_value_t raw_gyro_data;
//     ret = mpu6050_get_raw_acce(mpu6050, &raw_acce_data);
//     ret = mpu6050_get_raw_gyro(mpu6050, &raw_gyro_data);
//     ret_data.x1 = raw_acce_data.raw_acce_x-zero_acce.x;
//     ret_data.y1 = raw_acce_data.raw_acce_y-zero_acce.y;
//     ret_data.z1 = raw_acce_data.raw_acce_z-zero_acce.z;
//     ret_data.x2 = raw_gyro_data.raw_gyro_x-zero_gyro.x;
//     ret_data.y2 = raw_gyro_data.raw_gyro_y-zero_gyro.y;
//     ret_data.z2 = raw_gyro_data.raw_gyro_z-zero_gyro.z;

//     return ret_data;
// }





















#include <stdio.h>
#include "MPU6050.h"
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "../IIC/include/IIC.h"
#include "unity.h"
#include "esp_log.h"

#define MPU6050_GYRO_CONFIG 0x1Bu
#define MPU6050_ACCEL_CONFIG 0x1Cu
#define MPU6050_INTR_PIN_CFG 0x37u
#define MPU6050_INTR_ENABLE 0x38u
#define MPU6050_INTR_STATUS 0x3Au
#define MPU6050_ACCEL_XOUT_H 0x3Bu
#define MPU6050_GYRO_XOUT_H 0x43u
#define MPU6050_TEMP_XOUT_H 0x41u
#define MPU6050_PWR_MGMT_1 0x6Bu
#define MPU6050_WHO_AM_I 0x75u

#define ALPHA 0.3f              /*!< Weight of gyroscope */
#define RAD_TO_DEG 57.27272727f /*!< Radians to degrees */

static void *mpu6050 = NULL;

static const char *MPU6050TAG = "MPU6050";

mpu6050_acce_value_t acce = {
    .acce_x = 0,
    .acce_y = 0,
    .acce_z = 0};
mpu6050_gyro_value_t gyro = {
    .gyro_x = 0,
    .gyro_y = 0,
    .gyro_z = 0};
mpu6050_raw_acce_value_t raw_acce_zero = {
    .raw_acce_x = 0,
    .raw_acce_y = 0,
    .raw_acce_z = 0};
mpu6050_raw_gyro_value_t raw_gyro_zero = {
    .raw_gyro_x = 0,
    .raw_gyro_y = 0,
    .raw_gyro_z = 0};

typedef struct
{
    i2c_port_t bus;
    gpio_num_t int_pin;
    uint16_t dev_addr;
    uint32_t counter;
    float dt; /*!< delay time between two measurements, dt should be small (ms level) */
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
 *函数：void *mpu6050_create(i2c_port_t port, const uint16_t dev_addr)
 *功能：创建MPU6050句柄
 *参数：
 *i2c_port_t port   MPU6050IIC端口
 *const uint16_t dev_addr   MPU6050IIC地址
 *返回值：指向MPU6050模块信息句柄的指针
 *备注：创建包含了MPU6050模块信息的句柄
 ***************************************************************************************************/
void *mpu6050_create(i2c_port_t port, const uint16_t dev_addr)
{
    mpu6050_dev_t *sensor = (mpu6050_dev_t *)calloc(1, sizeof(mpu6050_dev_t));
    sensor->bus = port;
    sensor->dev_addr = dev_addr << 1;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *)calloc(1, sizeof(struct timeval));
    return (void *)sensor;
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

    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    uint8_t mpu6050_deviceid = 0;
    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    printf("%.2x\n", mpu6050_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");

    uint16_t i = 500;
    mpu6050_raw_acce_value_t raw1_acce;
    mpu6050_raw_gyro_value_t raw1_gyro;
    raw_acce_zero.raw_acce_x = 0;
    raw_acce_zero.raw_acce_y = 0;
    raw_acce_zero.raw_acce_z = 0;
    raw_gyro_zero.raw_gyro_x = 0;
    raw_gyro_zero.raw_gyro_y = 0;
    raw_gyro_zero.raw_gyro_z = 0;
    while (i--)
    {
        ret = mpu6050_get_raw_acce(mpu6050, &raw1_acce);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = mpu6050_get_raw_gyro(mpu6050, &raw1_gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        raw_acce_zero.raw_acce_x += raw1_acce.raw_acce_x;
        raw_acce_zero.raw_acce_y += raw1_acce.raw_acce_y;
        raw_acce_zero.raw_acce_z += raw1_acce.raw_acce_z - 8196;
        raw_gyro_zero.raw_gyro_x += raw1_gyro.raw_gyro_x;
        raw_gyro_zero.raw_gyro_y += raw1_gyro.raw_gyro_y;
        raw_gyro_zero.raw_gyro_z += raw1_gyro.raw_gyro_z;
    }
    raw_acce_zero.raw_acce_x /= 500;
    raw_acce_zero.raw_acce_y /= 500;
    raw_acce_zero.raw_acce_z /= 500;
    raw_gyro_zero.raw_gyro_x /= 500;
    raw_gyro_zero.raw_gyro_y /= 500;
    raw_gyro_zero.raw_gyro_z /= 500;

    ESP_LOGI(MPU6050TAG, "acc zero = %d %d %d\n", raw_acce_zero.raw_acce_x, raw_acce_zero.raw_acce_y, raw_acce_zero.raw_acce_z);
    ESP_LOGI(MPU6050TAG, "gyro zero = %d %d %d\n", raw_gyro_zero.raw_gyro_x, raw_gyro_zero.raw_gyro_y, raw_gyro_zero.raw_gyro_z);
}

/***************************************************************************************************
 *函数：esp_err_t mpu6050_wake_up(void *sensor)
 *功能：唤醒MPU6050
 *参数：
 *void *sensor指向MPU6050句柄的指针
 *返回值：无
 *备注：
 ***************************************************************************************************/
esp_err_t mpu6050_wake_up(void *sensor)
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
 *函数：esp_err_t mpu6050_config(void *sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
 *功能：配置MPU6050量程
 *参数：
 *void *sensor  指向MPU6050句柄的指针
 *const mpu6050_acce_fs_t acce_fs   acce量程配置项
 *const mpu6050_gyro_fs_t gyro_fs   gyro量程配置项
 *返回值：错误值
 *备注：
 ***************************************************************************************************/
esp_err_t mpu6050_config(void *sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
{
    uint8_t config_regs[2] = {gyro_fs << 3, acce_fs << 3};
    return mpu6050_write(sensor, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
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
 *函数：esp_err_t mpu6050_get_acce(void * sensor, mpu6050_acce_value_t *const acce_value)
 *功能：获取ACCE数据
 *参数：
 *sensor mpu6050句柄
 *const acce_value 指向加速度物理量值的指针
 *返回值：错误代码
 *备注：只获取除零漂的加速度并转换为实际的物理量
 ***************************************************************************************************/
esp_err_t mpu6050_get_acce(void *sensor, mpu6050_acce_value_t *const acce_value)
{
    esp_err_t ret;
    float acce_sensitivity;
    mpu6050_raw_acce_value_t raw_acce = {
        .raw_acce_x = 0,
        .raw_acce_y = 0,
        .raw_acce_z = 0};

    ret = mpu6050_get_acce_sensitivity(sensor, &acce_sensitivity);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = mpu6050_get_raw_acce(sensor, &raw_acce);
    if (ret != ESP_OK)
    {
        return ret;
    }

    acce_value->acce_x = (raw_acce.raw_acce_x - raw_acce_zero.raw_acce_x) / acce_sensitivity;
    acce_value->acce_y = (raw_acce.raw_acce_y - raw_acce_zero.raw_acce_y) / acce_sensitivity;
    acce_value->acce_z = (raw_acce.raw_acce_z - raw_acce_zero.raw_acce_z) / acce_sensitivity;
    return ESP_OK;
}
/***************************************************************************************************
 *函数：esp_err_t mpu6050_get_gyro(void * sensor, mpu6050_gyro_value_t *const gyro_value)
 *功能：获取GYRO数据
 *参数：
 *sensor mpu6050句柄
 *const gyro_value 指向角加速度值物理量的指针
 *返回值：错误代码
 *备注：只获取角加速度值除零漂并转换为实际的物理量
 ***************************************************************************************************/
esp_err_t mpu6050_get_gyro(void *sensor, mpu6050_gyro_value_t *const gyro_value)
{
    esp_err_t ret;
    float gyro_sensitivity;
    mpu6050_raw_gyro_value_t raw_gyro = {
        .raw_gyro_x = 0,
        .raw_gyro_y = 0,
        .raw_gyro_z = 0};

    ret = mpu6050_get_gyro_sensitivity(sensor, &gyro_sensitivity);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = mpu6050_get_raw_gyro(sensor, &raw_gyro);
    if (ret != ESP_OK)
    {
        return ret;
    }

    gyro_value->gyro_x = (raw_gyro.raw_gyro_x - raw_gyro_zero.raw_gyro_x) / gyro_sensitivity;
    gyro_value->gyro_y = (raw_gyro.raw_gyro_y - raw_gyro_zero.raw_gyro_y) / gyro_sensitivity;
    gyro_value->gyro_z = (raw_gyro.raw_gyro_z - raw_gyro_zero.raw_gyro_z) / gyro_sensitivity;
    return ESP_OK;
}

/***************************************************************************************************
 *函数：esp_err_t mpu6050_get_acce_sensitivity(void * sensor, float *const acce_sensitivity)
 *功能：获取ACCE的分度值
 *参数：
 *sensor mpu6050句柄
 *const acce_sensitivity 指向加速度分度值的指针
 *返回值：错误代码
 *备注：获取ACCE加速的分度值
 ***************************************************************************************************/
esp_err_t mpu6050_get_acce_sensitivity(void *sensor, float *const acce_sensitivity)
{
    esp_err_t ret;
    uint8_t acce_fs;
    ret = mpu6050_read(sensor, MPU6050_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs)
    {
    case ACCE_FS_2G:
        *acce_sensitivity = 16384;
        break;

    case ACCE_FS_4G:
        *acce_sensitivity = 8192;
        break;

    case ACCE_FS_8G:
        *acce_sensitivity = 4096;
        break;

    case ACCE_FS_16G:
        *acce_sensitivity = 2048;
        break;

    default:
        break;
    }
    return ret;
}

/***************************************************************************************************
 *函数：esp_err_t mpu6050_get_gyro_sensitivity(void * sensor, float *const gyro_sensitivity)
 *功能：获取GYRO的分度值
 *参数：
 *sensor mpu6050句柄
 *const gyro_sensitivity 指向加速度分度值的指针
 *返回值：错误代码
 *备注：获取GYRO加速的分度值
 ***************************************************************************************************/
esp_err_t mpu6050_get_gyro_sensitivity(void *sensor, float *const gyro_sensitivity)
{
    esp_err_t ret;
    uint8_t gyro_fs;
    ret = mpu6050_read(sensor, MPU6050_GYRO_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs)
    {
    case GYRO_FS_250DPS:
        *gyro_sensitivity = 131;
        break;

    case GYRO_FS_500DPS:
        *gyro_sensitivity = 65.5;
        break;

    case GYRO_FS_1000DPS:
        *gyro_sensitivity = 32.8;
        break;

    case GYRO_FS_2000DPS:
        *gyro_sensitivity = 16.4;
        break;

    default:
        break;
    }
    return ret;
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

esp_err_t mpu6050_complimentory_filter(void *sensor, const mpu6050_acce_value_t *const acce_value,
                                       const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle)
{
    float acce_angle[2];
    float gyro_angle[2];
    float gyro_rate[2];
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;

    sens->counter++;
    if (sens->counter == 1)
    {
        acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
        acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
        complimentary_angle->roll = acce_angle[0];
        complimentary_angle->pitch = acce_angle[1];
        gettimeofday(sens->timer, NULL);
        return ESP_OK;
    }

    struct timeval now, dt_t;
    gettimeofday(&now, NULL);
    timersub(&now, sens->timer, &dt_t);
    sens->dt = (float)(dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
    gettimeofday(sens->timer, NULL);

    acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
    acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);

    gyro_rate[0] = gyro_value->gyro_x;
    gyro_rate[1] = gyro_value->gyro_y;
    gyro_angle[0] = gyro_rate[0] * sens->dt;
    gyro_angle[1] = gyro_rate[1] * sens->dt;
    
    complimentary_angle->roll = (ALPHA * (complimentary_angle->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
    complimentary_angle->pitch = (ALPHA * (complimentary_angle->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);

    return ESP_OK;
}
/***************************************************************************************************
 *函数：esp_err_t IMU(complimentary_angle_t *complimentary_angle)
 *功能：获取俯仰角、横滚角、Z轴加速度
 *参数：
 *complimentary_angle_t *complimentary_angle 指向回传数据的指针（pitch、row、z_acce）
 *返回值：错误代码
 *备注：与main函数的接口函数，传递读取和解算出的数据
 ***************************************************************************************************/
esp_err_t IMU(complimentary_angle_t *complimentary_angle)
{

    esp_err_t ret;
    ret = mpu6050_get_acce(mpu6050, &acce);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    // ESP_LOGI(MPU6050TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

    ret = mpu6050_get_gyro(mpu6050, &gyro);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    // ESP_LOGI(MPU6050TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

    ret = mpu6050_complimentory_filter(mpu6050, &acce, &gyro, complimentary_angle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    complimentary_angle->accz = acce.acce_z;
    // ESP_LOGI(MPU6050TAG, "pitch:%.2f roll:%.2f \n", complimentary_angle->pitch,complimentary_angle->roll);

    return ret;
}