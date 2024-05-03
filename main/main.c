#include <main.h>
#include <IIC.h>
#include <MPU6050.h>
#include <IMU.h>
#include <freertos/FreeRTOS.h>
#include <BMP280.h>
#include "esp_log.h"
#include <MOTOR.h>
#include "driver/gpio.h"
complimentary_angle_t complimentary_angle = {
    .pitch = 0,
    .roll = 0,
    .accz = 0};
void MainInit(){
    i2c_bus_init();
    i2c_sensor_mpu6050_init();
    i2c_sensor_bmp280_init();
    MotorControl_Init();
    // Kalman_Init();
}
void showbanner(void){
    printf("+-+-+-+-+-+-+\n|X|L|_|U|A|V|\n+-+-+-+-+-+-+\n");
}
void app_main(void)
{
    showbanner();
    MainInit();
    vTaskDelay(2000/portTICK_PERIOD_MS);
    while (1)
    {
        IMU(&complimentary_angle);
        ESP_LOGI("main","pitch:%10.2f roll:%10.2f accz:%10.2f ", complimentary_angle.pitch, complimentary_angle.roll, complimentary_angle.accz);
        // mpu6050_raw_data_return_t mpu6050;
        // mpu6050 = mpu6050_get_raw_data();
        // printf("accx %5.2lf accy %5.2lf accz %5.2lf gyrox %5.2lf gyroy %5.2lf gyroz %5.2lf\n",mpu6050.x1,mpu6050.y1,mpu6050.z1,mpu6050.x2,mpu6050.y2,mpu6050.z2);
        // printf("%5.2f\n",mpu6050.x1);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    
}
