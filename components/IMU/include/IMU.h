// #ifndef _IMU_H_
// #define _IMU_H_
// #include <MPU6050.h>
// typedef struct
// {
//     double Last_P; // 上次估算协方差 不可以为0 ! ! ! ! !
//     double Now_P;  // 当前估算协方差
//     double out;    // 卡尔曼滤波器输出
//     double Kg;     // 卡尔曼增益
//     double Q;      // 过程噪声协方差
//     double R;      // 观测噪声协方差
// } Kalman;

// typedef struct 
// {
//     double pitch;
//     double roll;
//     double yaw;
// }att_angel_t;

// #define G			9.80665f		      	// m/s^2	
// #define RadtoDeg    57.324841f				//弧度到角度 (弧度 * 180/3.1415)
// #define DegtoRad    0.0174533f				//角度到弧度 (角度 * 3.1415/180)

// static void raw_to_physical();
// static mpu6050_all_raw_data_t imu_windows_filt(mpu6050_all_raw_data_t new_data);
// static double imu_kalman_filt(Kalman *kfp, double input);
// static esp_err_t IMU_updata(mpu6050_dev_t *sensor,att_angel_t *att_angel);
// void IMU();
// void Kalman_Init();
// #endif