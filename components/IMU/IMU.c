#include <stdio.h>
#include "IMU.h"
#include "esp_log.h"

#define ALPHA 0.3f              /*!< Weight of gyroscope */
#define RAD_TO_DEG 57.27272727f /*!< Radians to degrees */
#define WINDOW_SIZE 50
mpu6050_all_raw_data_t raw_data;
mpu6050_all_raw_data_t physical_data;
mpu6050_all_raw_data_t windows[WINDOW_SIZE];
mpu6050_all_raw_data_t temp_data;
static uint16_t index = 0;
static mpu6050_all_raw_data_t sum = {
    .x1 = 0,
    .y1 = 0,
    .z1 = 0,
    .x2 = 0,
    .y2 = 0,
    .z2 = 0

};

static const char *TAG = "IMU";

static mpu6050_all_raw_data_t imu_windows_filt(mpu6050_all_raw_data_t new_data)
{
    mpu6050_all_raw_data_t ret;
    if (index < WINDOW_SIZE)
    {
        sum.x1 += new_data.x1;
        sum.y1 += new_data.y1;
        sum.z1 += new_data.z1;
        sum.x2 += new_data.x2;
        sum.y2 += new_data.y2;
        sum.z2 += new_data.z2;
    }
    else
    {
        sum.x1 = -windows[index % WINDOW_SIZE].x1 + new_data.x1;
        sum.y1 = -windows[index % WINDOW_SIZE].y1 + new_data.y1;
        sum.z1 = -windows[index % WINDOW_SIZE].z1 + new_data.z1;
        sum.x2 = -windows[index % WINDOW_SIZE].x2 + new_data.x2;
        sum.y2 = -windows[index % WINDOW_SIZE].y2 + new_data.y2;
        sum.z2 = -windows[index % WINDOW_SIZE].z2 + new_data.z2;
    }
    windows[index % WINDOW_SIZE].x1 = new_data.x1;
    windows[index % WINDOW_SIZE].y1 = new_data.y1;
    windows[index % WINDOW_SIZE].z1 = new_data.z1;
    windows[index % WINDOW_SIZE].x2 = new_data.x2;
    windows[index % WINDOW_SIZE].y2 = new_data.y2;
    windows[index % WINDOW_SIZE].z2 = new_data.z2;

    index++;
    
    if (index < WINDOW_SIZE)
    {
        ret.x1 = sum.x1 / (index * 1.0);
        ret.y1 = sum.y1 / (index * 1.0);
        ret.z1 = sum.z1 / (index * 1.0);
        ret.x2 = sum.x2 / (index * 1.0);
        ret.y2 = sum.y2 / (index * 1.0);
        ret.z2 = sum.z2 / (index * 1.0);
    }
    else
    {
        ret.x1 = sum.x1 / (WINDOW_SIZE * 1.0);
        ret.y1 = sum.y1 / (WINDOW_SIZE * 1.0);
        ret.z1 = sum.z1 / (WINDOW_SIZE * 1.0);
        ret.x2 = sum.x2 / (WINDOW_SIZE * 1.0);
        ret.y2 = sum.y2 / (WINDOW_SIZE * 1.0);
        ret.z2 = sum.z2 / (WINDOW_SIZE * 1.0);
    }
    // printf("windows:");
    // for(uint8_t i =0;i<WINDOW_SIZE;i++){
    //     printf("%5.2f ",windows[i].x1);
    // }
    // printf("\n");
    if (index == WINDOW_SIZE*2)
    {
        index = WINDOW_SIZE;
    }
    return ret;
}

static void raw_to_physical()
{
    raw_data = mpu6050_get_raw_data();
    temp_data = imu_windows_filt(raw_data);
    physical_data.x1 = temp_data.x1 / 16384.0;
    physical_data.y1 = temp_data.y1 / 16384.0;
    physical_data.z1 = temp_data.z1 / 16384.0;
}

void IMU()
{
    raw_to_physical();
    // ESP_LOGI(TAG,"successful get physical data");
    printf("%5.2f\n",temp_data.x1);
    // printf("raw:%05.6lf %05.6lf %05.6lf\traw:%05.6lf %05.6lf %05.6lf\n", raw_data.x1, raw_data.y1, raw_data.z1, physical_data.x1, physical_data.y1, physical_data.z1);
}