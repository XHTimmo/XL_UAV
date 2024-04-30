#ifndef _BMP280_H_
#define _BMP280_H_
#include "driver/i2c.h"
#include "driver/gpio.h"

#define BMP280_I2C_ADDRESS          0x76u
#define BMP280_WHO_AM_I_VAL         0x58u

typedef struct {
    float T;
    float P;
} BMP280_DATA;

typedef void * BMP280_handle;
static BMP280_handle bmp280_create(i2c_port_t port,const uint16_t dev_addr);
static esp_err_t bmp280_get_deviceid(void* sensor, uint8_t *const deviceid);
static esp_err_t bmp280_init(void* sensor);
esp_err_t bmp280_read_data(BMP280_DATA * bmp280_data);
void i2c_sensor_bmp280_init(void);
#endif
