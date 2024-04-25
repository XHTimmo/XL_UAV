#include <stdio.h>
#include <IIC.h>
#include <MPU6050.h>
#include <freertos/FreeRTOS.h>

void MainInit(){
    i2c_bus_init();
    i2c_sensor_mpu6050_init();
}
void app_main(void)
{
    printf("XL_UAV\n");
    MainInit();
    while (1)
    {

        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    
}
