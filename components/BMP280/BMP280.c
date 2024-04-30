#include <stdio.h>
#include <string.h>
#include "BMP280.h"
#include <sys/time.h>
#include "../IIC/include/IIC.h"
#include "unity.h"
// defien reg addr
#define BMP280_RESET_REG 0xE0u
#define BMP280_CTRL_MEAS_REG 0xF4u
#define BMP280_CONFIG_REG 0xF5u
#define BMP280_DIG_ADDR 0x88u
#define BMP280_DATA_ADDR 0xF7u
// define reg val
#define BMP280_RESET_VAL 0xB6u
/*crtl_meas
    [7:5]--osrs_temperature
        001         --  16bit/0.0050
        010         --  17bit/0.0025
        011         --  18bit/0.0012
        100         --  19bit/0.0006
        101\others  --  20bit/0.0003
    [4:2]--osrs_pressure
        000
        001         --  16bit/2.62Pa
        010         --  17bit/1.31Pa
        011         --  18bit/0.66Pa
        100         --  19bit/0.33Pa
        101\others  --  20bit/0.16Pa
    [1:0]--mode
        00          --  sleep mode
        01/10       --  Forced mode
        11          --  Normal mode
*/
#define BMP280_CRTL_MEAS_VAL 0xFFu
/*
    [7:5]--time standby
        000         --  0.5ms
        001         --  62.5ms
        010         --  125ms
        011         --  250ms
        100         --  500ms
        101         --  1000ms
        110         --  2000ms
        111         --  4000ms
    [4:2]--filter
        000         --  off
        010         --  2
        011         --  5
        100         --  11
        101\others  --  22
    [1] --  NULL
    [0] --  3wire_spi_en
        0           --  unenable
        1           --  enable
*/
#define BMP280_CONFIG_VAL 0x00u

void *bmp280 = NULL;
static const char *BMPTAG = "BMP280";
#define BMP280_WHO_AM_I 0xD0u
typedef struct
{
    uint16_t T1;
    int16_t T2;
    int16_t T3;
    uint16_t P1;
    int16_t P2;
    int16_t P3;
    int16_t P4;
    int16_t P5;
    int16_t P6;
    int16_t P7;
    int16_t P8;
    int16_t P9;
} bmp280_dig_t;

bmp280_dig_t bmp280_dig = {
    .T1 = 0,
    .T2 = 0,
    .T3 = 0,
    .P1 = 0,
    .P2 = 0,
    .P3 = 0,
    .P4 = 0,
    .P5 = 0,
    .P6 = 0,
    .P7 = 0,
    .P8 = 0,
    .P9 = 0};

typedef struct
{
    i2c_port_t bus;
    gpio_num_t int_pin;
    uint16_t dev_addr;
    uint32_t counter;
    float dt; /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
} bmp280_dev_t;
/***************************************************************************************************
*函数：static esp_err_t bmp280_read(void *sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
*功能：bmp280读函数
*参数：
        *sensor bmp280句柄
        const uint8_t reg_start_addr 读目标的寄存器起始地址
        uint8_t *const data_buf 指向读的数据存储的buf的指针
        const uint8_t data_len 读入的数据长度
*返回值：错误代码
*备注：
***************************************************************************************************/
static esp_err_t bmp280_read(void *sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    bmp280_dev_t *sens = (bmp280_dev_t *)sensor;
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
*函数：static esp_err_t bmp280_write(void *sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
*功能：bmp280写函数
*参数：
        *sensor bmp280句柄
        const uint8_t reg_start_addr 写目标的寄存器起始地址
        uint8_t *const data_buf 指向写入的数据存储的buf的指针
        const uint8_t data_len 写入的数据长度
*返回值：错误代码
*备注：
***************************************************************************************************/
static esp_err_t bmp280_write(void *sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    bmp280_dev_t *sens = (bmp280_dev_t *)sensor;
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
*函数：static BMP280_handle bmp280_create(i2c_port_t port, const uint16_t dev_addr)
*功能：创建bmp280对象
*参数：
        port bmp280的IIC端口号
        dev_addr bmp280的IIC地址
        uint8_t *const data_buf 指向写入的数据存储的buf的指针
        const uint8_t data_len 写入的数据长度
*返回值：bmp280句柄
*备注：
***************************************************************************************************/
static BMP280_handle bmp280_create(i2c_port_t port, const uint16_t dev_addr)
{
    bmp280_dev_t *sensor = (bmp280_dev_t *)calloc(1, sizeof(bmp280_dev_t));
    sensor->bus = port;
    sensor->dev_addr = dev_addr << 1;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *)calloc(1, sizeof(struct timeval));
    return (BMP280_handle *)sensor;
}
/***************************************************************************************************
*函数：esp_err_t bmp280_get_deviceid(void *sensor, uint8_t *const deviceid)
*功能：获取BMP
*参数：
        *sensor bmp280的句柄
        deviceid bmp280的ID读取后的存储位置
*返回值：错误代码
*备注：
***************************************************************************************************/
esp_err_t bmp280_get_deviceid(void *sensor, uint8_t *const deviceid)
{
    return bmp280_read(sensor, BMP280_WHO_AM_I, deviceid, 1);
}
/***************************************************************************************************
*函数：esp_err_t bmp280_init(void *sensor)
*功能：BMP初始化
*参数：
*返回值：错误代码
*备注：
***************************************************************************************************/
esp_err_t bmp280_init(void *sensor)
{
    esp_err_t ret;

    uint8_t bmp280_raw_data[24];
    ret = bmp280_read(sensor, BMP280_DIG_ADDR, bmp280_raw_data, sizeof(bmp280_raw_data));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    bmp280_dig.T1 = (uint16_t)((bmp280_raw_data[1] << 8) | (bmp280_raw_data[0]));
    bmp280_dig.T2 = (int16_t)((bmp280_raw_data[3] << 8) | (bmp280_raw_data[2]));
    bmp280_dig.T3 = (int16_t)((bmp280_raw_data[5] << 8) | (bmp280_raw_data[4]));
    bmp280_dig.P1 = (uint16_t)((bmp280_raw_data[7] << 8) | (bmp280_raw_data[6]));
    bmp280_dig.P2 = (int16_t)((bmp280_raw_data[9] << 8) | (bmp280_raw_data[8]));
    bmp280_dig.P3 = (int16_t)((bmp280_raw_data[11] << 8) | (bmp280_raw_data[10]));
    bmp280_dig.P4 = (int16_t)((bmp280_raw_data[13] << 8) | (bmp280_raw_data[12]));
    bmp280_dig.P5 = (int16_t)((bmp280_raw_data[15] << 8) | (bmp280_raw_data[14]));
    bmp280_dig.P6 = (int16_t)((bmp280_raw_data[17] << 8) | (bmp280_raw_data[16]));
    bmp280_dig.P7 = (int16_t)((bmp280_raw_data[19] << 8) | (bmp280_raw_data[18]));
    bmp280_dig.P8 = (int16_t)((bmp280_raw_data[21] << 8) | (bmp280_raw_data[20]));
    bmp280_dig.P9 = (int16_t)((bmp280_raw_data[23] << 8) | (bmp280_raw_data[22]));

    uint8_t bmp280_write_temp = BMP280_RESET_VAL;
    ret = bmp280_write(sensor, BMP280_RESET_REG, &bmp280_write_temp, sizeof(bmp280_write_temp));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    bmp280_write_temp = BMP280_CRTL_MEAS_VAL;
    bmp280_write(sensor, BMP280_CTRL_MEAS_REG, &bmp280_write_temp, sizeof(bmp280_write_temp));
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    vTaskDelay(200 / portTICK_PERIOD_MS);
    return ret;
}

/***************************************************************************************************
*函数：esp_err_t bmp280_read_data(BMP280_DATA *bmp280_data)
*功能：BMP读取气压及温度数据
*参数：
        *bmp280_data 存取读取并解算的数据的结构体指针
*返回值：错误代码
*备注：
***************************************************************************************************/

esp_err_t bmp280_read_data(BMP280_DATA *bmp280_data)
{
    uint8_t raw_data[6];
    esp_err_t ret = bmp280_read(bmp280, BMP280_DATA_ADDR, raw_data, sizeof(raw_data));

    int32_t adc_P = (int32_t)((uint32_t)raw_data[0] << 12 | (uint32_t)raw_data[1] << 4 | (uint32_t)raw_data[2] >> 4);
    int32_t adc_T = (int32_t)((uint32_t)raw_data[3] << 12 | (uint32_t)raw_data[4] << 4 | (uint32_t)raw_data[5] >> 4);

    int32_t t_fine;
    double var1, var2, T, p;
    var1 = (((double)adc_T) / 16384.0 - ((double)bmp280_dig.T1) / 1024.0) * ((double)bmp280_dig.T2);
    var2 = ((((double)adc_T) / 131072.0 - ((double)bmp280_dig.T1) / 8192.0) *
            (((double)adc_T) / 131072.0 - ((double)bmp280_dig.T1) / 8192.0)) *
           ((double)bmp280_dig.T3);
    t_fine = (int32_t)(var1 + var2);
    T = (var1 + var2) / 5120.0;

    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)bmp280_dig.P6) / 32768.0;
    var2 = var2 + var1 * ((double)bmp280_dig.P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)bmp280_dig.P4) * 65536.0);
    var1 = (((double)bmp280_dig.P3) * var1 * var1 / 524288.0 + ((double)bmp280_dig.P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)bmp280_dig.P1);
    p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)bmp280_dig.P9) * p * p / 2147483648.0;
    var2 = p * ((double)bmp280_dig.P8) / 32768.0;
    p = p + (var1 + var2 + ((double)bmp280_dig.P7)) / 16.0;
    bmp280_data->P = p;
    bmp280_data->T = T;

    return ret;
}

/***************************************************************************************************
*函数：void i2c_sensor_bmp280_init(void)
*功能：BMP读取气压及温度数据
*参数：
        *bmp280_data 存取读取并解算的数据的结构体指针
*返回值：错误代码
*备注：
***************************************************************************************************/
void i2c_sensor_bmp280_init(void)
{
    esp_err_t ret;
    bmp280 = bmp280_create(I2C_MASTER_NUM, BMP280_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(bmp280, "bmp280 create returned NULL");
    uint8_t bmp280_deviceid;
    ret = bmp280_get_deviceid(bmp280, &bmp280_deviceid);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    printf("bmp280 Who Am I:%.2x\n", bmp280_deviceid);
    bmp280_init(bmp280);

    // TEST_ASSERT_EQUAL_UINT8_MESSAGE(BMP280_WHO_AM_I_VAL, bmp280_deviceid, "Who Am I register does not contain expected data");
    // bmp280_read_data();
}