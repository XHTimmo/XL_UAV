#include <main.h>
#include <IIC.h>
#include <MPU6050.h>
#include <freertos/FreeRTOS.h>

void MainInit(){
    i2c_bus_init();
    i2c_sensor_mpu6050_init();
}
void showbanner(void){
    printf("+-+-+-+-+-+-+\n|X|L|_|U|A|V|\n+-+-+-+-+-+-+\n");
}
void app_main(void)
{
    showbanner();
    MainInit();
    while (1)
    {
        mpu6050_raw_data_return_t mpu6050;
        mpu6050 = mpu6050_get_raw_data();
        printf("accx:%5.2lf\taccy:%5.2lf\taccz:%5.2lf\tgyrox:%5.2lf\tgyroy:%5.2lf\tgyroz:%5.2lf\n",mpu6050.x1,mpu6050.y1,mpu6050.z1,mpu6050.x2,mpu6050.y2,mpu6050.z2);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    
}
