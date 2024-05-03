// #ifndef _MPU6050_H_
// #define _MPU6050_H_

// #include "driver/i2c.h"
// #include "driver/gpio.h"

// #define MPU6050_IIC_ADDR 0x68u
// #define MPU6050_WHO_AM_I_VAL 0x68u

// #define MPU6050_GYRO_CONFIG 0x1Bu
// #define MPU6050_ACCEL_CONFIG 0x1Cu
// #define MPU6050_INTR_PIN_CFG 0x37u
// #define MPU6050_INTR_ENABLE 0x38u
// #define MPU6050_INTR_STATUS 0x3Au
// #define MPU6050_ACCEL_XOUT_H 0x3Bu
// #define MPU6050_GYRO_XOUT_H 0x43u
// #define MPU6050_TEMP_XOUT_H 0x41u
// #define MPU6050_PWR_MGMT_1 0x6Bu
// #define MPU6050_WHO_AM_I 0x75u



// typedef void * MPU6050_handle;

// typedef enum {
//     ACCE_FS_2G  = 0,     /*!< Accelerometer full scale range is +/- 2g */
//     ACCE_FS_4G  = 1,     /*!< Accelerometer full scale range is +/- 4g */
//     ACCE_FS_8G  = 2,     /*!< Accelerometer full scale range is +/- 8g */
//     ACCE_FS_16G = 3,     /*!< Accelerometer full scale range is +/- 16g */
// } mpu6050_acce_fs_t;

// typedef enum {
//     GYRO_FS_250DPS  = 0,     /*!< Gyroscope full scale range is +/- 250 degree per sencond */
//     GYRO_FS_500DPS  = 1,     /*!< Gyroscope full scale range is +/- 500 degree per sencond */
//     GYRO_FS_1000DPS = 2,     /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
//     GYRO_FS_2000DPS = 3,     /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
// } mpu6050_gyro_fs_t;

// typedef struct {
//     int16_t raw_acce_x;
//     int16_t raw_acce_y;
//     int16_t raw_acce_z;
// } mpu6050_raw_acce_value_t;

// typedef struct {
//     int16_t raw_gyro_x;
//     int16_t raw_gyro_y;
//     int16_t raw_gyro_z;
// } mpu6050_raw_gyro_value_t;


// typedef struct {
//     double x;
//     double y;
//     double z;
// } mpu6050_raw_data_t;

// typedef struct {
//     double x1;
//     double y1;
//     double z1;
//     double x2;
//     double y2;
//     double z2;
// } mpu6050_all_raw_data_t;

// typedef struct
// {
//     i2c_port_t bus;
//     gpio_num_t int_pin;
//     uint16_t dev_addr;
//     uint32_t counter;
//     float dt;
//     struct timeval *timer;
// } mpu6050_dev_t;

// extern MPU6050_handle mpu6050;

// static MPU6050_handle mpu6050_create(i2c_port_t port,const uint16_t dev_addr);
// static esp_err_t mpu6050_config(void *sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs);
// static esp_err_t mpu6050_wake_up(void * sensor);
// static esp_err_t mpu6050_get_deviceid(void* sensor, uint8_t *const deviceid);
// static esp_err_t mpu6050_read(void *sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len);
// static esp_err_t mpu6050_write(void *sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len);
// static esp_err_t mpu6050_get_raw_acce(void * sensor, mpu6050_raw_acce_value_t *const raw_acce_value);
// static esp_err_t mpu6050_get_raw_gyro(void * sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value);
// static void mpu6050_get_raw_zero(void * sensor);


// //public
// void i2c_sensor_mpu6050_init(void);
// mpu6050_all_raw_data_t mpu6050_get_raw_data(void);
// #endif


#include "driver/i2c.h"
#include "driver/gpio.h"

#define MPU6050_I2C_ADDRESS         0x68u /*!< I2C address with AD0 pin low */
#define MPU6050_WHO_AM_I_VAL        0x68u


typedef enum {
    ACCE_FS_2G  = 0,     /*!< Accelerometer full scale range is +/- 2g */
    ACCE_FS_4G  = 1,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_8G  = 2,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_16G = 3,     /*!< Accelerometer full scale range is +/- 16g */
} mpu6050_acce_fs_t;

typedef enum {
    GYRO_FS_250DPS  = 0,     /*!< Gyroscope full scale range is +/- 250 degree per sencond */
    GYRO_FS_500DPS  = 1,     /*!< Gyroscope full scale range is +/- 500 degree per sencond */
    GYRO_FS_1000DPS = 2,     /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
    GYRO_FS_2000DPS = 3,     /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
} mpu6050_gyro_fs_t;

typedef struct {
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} mpu6050_raw_acce_value_t;

typedef struct {
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} mpu6050_raw_gyro_value_t;

typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
} mpu6050_acce_value_t;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_gyro_value_t;

typedef struct {
    float temp;
} mpu6050_temp_value_t;

typedef struct {
    float roll;
    float pitch;
    float accz;
} complimentary_angle_t;


void *mpu6050_create(i2c_port_t port, const uint16_t dev_addr);

void i2c_sensor_mpu6050_init(void);

esp_err_t mpu6050_config(void *sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs);

esp_err_t mpu6050_wake_up(void * sensor);

esp_err_t mpu6050_get_deviceid(void* sensor, uint8_t *const deviceid);

esp_err_t mpu6050_get_acce(void * sensor, mpu6050_acce_value_t *const acce_value);

esp_err_t mpu6050_get_gyro(void * sensor, mpu6050_gyro_value_t *const gyro_value);

esp_err_t mpu6050_get_temp(void * sensor, mpu6050_temp_value_t *const temp_value);

esp_err_t mpu6050_get_acce_sensitivity(void * sensor, float *const acce_sensitivity);

esp_err_t mpu6050_get_gyro_sensitivity(void * sensor, float *const gyro_sensitivity);

esp_err_t mpu6050_get_raw_acce(void * sensor, mpu6050_raw_acce_value_t *const raw_acce_value);

esp_err_t mpu6050_get_raw_gyro(void * sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value);

esp_err_t mpu6050_complimentory_filter(void * sensor, const mpu6050_acce_value_t *const acce_value,
                                       const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle);

esp_err_t IMU(complimentary_angle_t *complimentary_angle);