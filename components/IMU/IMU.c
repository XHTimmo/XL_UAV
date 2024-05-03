// // #include <stdio.h>
// // #include "IMU.h"
// // #include "esp_log.h"
// // #include <math.h>
// // #include <time.h>
// // #include <sys/time.h>
// // #define ALPHA 0.3f              /*!< Weight of gyroscope */
// // #define RAD_TO_DEG 57.27272727f /*!< Radians to degrees */
// // #define WINDOW_SIZE 10
// // mpu6050_all_raw_data_t raw_data;
// // mpu6050_all_raw_data_t physical_data;
// // mpu6050_all_raw_data_t windows[WINDOW_SIZE];
// // mpu6050_all_raw_data_t windows_data;
// // mpu6050_all_raw_data_t kalman_data;
// // static uint16_t index = 0;
// // float accb[3], DCMgb[3][3]; // 方向余弦阵（将 惯性坐标系 转化为 机体坐标系）

// // static mpu6050_all_raw_data_t sum = {
// //     .x1 = 0,
// //     .y1 = 0,
// //     .z1 = 0,
// //     .x2 = 0,
// //     .y2 = 0,
// //     .z2 = 0

// // };

// // Kalman kfp;

// // static const char *TAG = "IMU";

// // static mpu6050_all_raw_data_t imu_windows_filt(mpu6050_all_raw_data_t new_data)
// // {
// //     mpu6050_all_raw_data_t ret;
// //     if (index < WINDOW_SIZE)
// //     {
// //         sum.x1 += new_data.x1;
// //         sum.y1 += new_data.y1;
// //         sum.z1 += new_data.z1;
// //         sum.x2 += new_data.x2;
// //         sum.y2 += new_data.y2;
// //         sum.z2 += new_data.z2;
// //     }
// //     else
// //     {
// //         sum.x1 = sum.x1 - windows[index % WINDOW_SIZE].x1 + new_data.x1;
// //         sum.y1 = sum.y1 - windows[index % WINDOW_SIZE].y1 + new_data.y1;
// //         sum.z1 = sum.z1 - windows[index % WINDOW_SIZE].z1 + new_data.z1;
// //         sum.x2 = sum.x2 - windows[index % WINDOW_SIZE].x2 + new_data.x2;
// //         sum.y2 = sum.y2 - windows[index % WINDOW_SIZE].y2 + new_data.y2;
// //         sum.z2 = sum.z2 - windows[index % WINDOW_SIZE].z2 + new_data.z2;
// //     }
// //     // printf("%5.2lf\t",sum.x1);
// //     windows[index % WINDOW_SIZE].x1 = new_data.x1;
// //     windows[index % WINDOW_SIZE].y1 = new_data.y1;
// //     windows[index % WINDOW_SIZE].z1 = new_data.z1;
// //     windows[index % WINDOW_SIZE].x2 = new_data.x2;
// //     windows[index % WINDOW_SIZE].y2 = new_data.y2;
// //     windows[index % WINDOW_SIZE].z2 = new_data.z2;

// //     index++;

// //     if (index < WINDOW_SIZE)
// //     {
// //         ret.x1 = sum.x1 / (index * 1.0);
// //         ret.y1 = sum.y1 / (index * 1.0);
// //         ret.z1 = sum.z1 / (index * 1.0);
// //         ret.x2 = sum.x2 / (index * 1.0);
// //         ret.y2 = sum.y2 / (index * 1.0);
// //         ret.z2 = sum.z2 / (index * 1.0);
// //     }
// //     else
// //     {
// //         ret.x1 = sum.x1 / (WINDOW_SIZE * 1.0);
// //         ret.y1 = sum.y1 / (WINDOW_SIZE * 1.0);
// //         ret.z1 = sum.z1 / (WINDOW_SIZE * 1.0);
// //         ret.x2 = sum.x2 / (WINDOW_SIZE * 1.0);
// //         ret.y2 = sum.y2 / (WINDOW_SIZE * 1.0);
// //         ret.z2 = sum.z2 / (WINDOW_SIZE * 1.0);
// //     }
// //     if (index == WINDOW_SIZE * 2)
// //     {
// //         index = WINDOW_SIZE;
// //     }
// //     return ret;
// // }

// // void Kalman_Init()
// // {
// //     kfp.Last_P = 1;
// //     kfp.Now_P = 0;
// //     kfp.out = 0;
// //     kfp.Kg = 0;
// //     kfp.Q = 0.0005;
// //     kfp.R = 0.01;
// // }

// // double imu_kalman_filt(Kalman *kfp, double input)
// // {
// //     // 预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
// //     kfp->Now_P = kfp->Last_P + kfp->Q;
// //     // 卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
// //     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
// //     // 更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
// //     kfp->out = kfp->out + kfp->Kg * (input - kfp->out); // 因为这一次的预测值就是上一次的输出值
// //     // 更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
// //     kfp->Last_P = (1 - kfp->Kg) * kfp->Now_P;
// //     return kfp->out;
// // }

// // // /****************************************************************************************************
// // //  * 函  数：static float invSqrt(float x)
// // //  * 功　能: 快速计算 1/Sqrt(x)
// // //  * 参  数：要计算的值
// // //  * 返回值：计算的结果
// // //  * 备  注：比普通Sqrt()函数要快四倍See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
// // //  *****************************************************************************************************/
// // // static float invSqrt(float x)
// // // {
// // //     float halfx = 0.5f * x;
// // //     float y = x;
// // //     long i = *(long *)&y;
// // //     i = 0x5f375a86 - (i >> 1);
// // //     y = *(float *)&i;
// // //     y = y * (1.5f - (halfx * y * y));
// // //     return y;
// // // }

// // static void raw_to_physical()
// // {
// //     raw_data = mpu6050_get_raw_data();
// //     // windows_data = imu_windows_filt(raw_data);
// //     kalman_data.x1 = imu_kalman_filt(&kfp, raw_data.x1);
// //     kalman_data.y1 = imu_kalman_filt(&kfp, raw_data.y1);
// //     kalman_data.z1 = imu_kalman_filt(&kfp, raw_data.z1);
// //     kalman_data.x2 = imu_kalman_filt(&kfp, raw_data.x2);
// //     kalman_data.y2 = imu_kalman_filt(&kfp, raw_data.y2);
// //     kalman_data.z2 = imu_kalman_filt(&kfp, raw_data.z2);
// //     physical_data.x1 = kalman_data.x1 / 16384.0; // 2*2/65535
// //     physical_data.y1 = kalman_data.y1 / 16384.0;
// //     physical_data.z1 = kalman_data.z1 / 16384.0;
// //     physical_data.x2 = kalman_data.x2 / 65.5; 
// //     physical_data.y2 = kalman_data.y2 / 65.5;
// //     physical_data.z2 = kalman_data.z2 / 65.5;
// //     accb[0] = physical_data.x1;
// //     accb[1] = physical_data.y1;
// //     accb[2] = physical_data.z1;
// // }

// // // #define Kp 0    // proportional gain governs rate of convergence to accelerometer/magnetometer
// // //                      // 比例增益控制加速度计，磁力计的收敛速率
// // // #define Ki 0    // integral gain governs rate of convergence of gyroscope biases
// // //                      // 积分增益控制陀螺偏差的收敛速度
// // // #define halfT 0.005f // half the sample period 采样周期的一半

// // // float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // quaternion elements representing the estimated orientation
// // // float exInt = 0, eyInt = 0, ezInt = 0; // scaled integral error

// // // void IMU_updata(att_angel_t *att_angel)
// // // {
// // //     uint8_t i;
// // //     float matrix[9] = {1.f, 0.0f, 0.0f, 0.0f, 1.f, 0.0f, 0.0f, 0.0f, 1.f}; // 初始化矩阵
// // //     float ax = physical_data.x1, ay = physical_data.y1, az = physical_data.z1;
// // //     float gx = physical_data.x2, gy = physical_data.y2, gz = physical_data.z2;
// // //     float vx, vy, vz;
// // //     float ex, ey, ez;
// // //     float norm;

// // //     float q0q0 = q0 * q0;
// // //     float q0q1 = q0 * q1;
// // //     float q0q2 = q0 * q2;
// // //     float q0q3 = q0 * q3;
// // //     float q1q1 = q1 * q1;
// // //     float q1q2 = q1 * q2;
// // //     float q1q3 = q1 * q3;
// // //     float q2q2 = q2 * q2;
// // //     float q2q3 = q2 * q3;
// // //     float q3q3 = q3 * q3;

// // //     if (ax * ay * az == 0)
// // //         return;

// // //     norm = invSqrt(ax * ax + ay * ay + az * az);
// // //     ax = ax * norm;
// // //     ay = ay * norm;
// // //     az = az * norm;
// // //     //	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);

// // //     // 陀螺仪积分估计重力向量(机体坐标系)
// // //     vx = 2 * (q1q3 - q0q2);
// // //     vy = 2 * (q0q1 + q2q3);
// // //     vz = q0q0 - q1q1 - q2q2 + q3q3;
// // //     // printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz);

// // //     // 测量的重力向量与估算的重力向量差积求出向量间的误差
// // //     ex = (ay * vz - az * vy); //+ (my*wz - mz*wy);
// // //     ey = (az * vx - ax * vz); //+ (mz*wx - mx*wz);
// // //     ez = (ax * vy - ay * vx); //+ (mx*wy - my*wx);

// // //     // 用上面求出误差进行积分
// // //     exInt = exInt + ex * Ki;
// // //     eyInt = eyInt + ey * Ki;
// // //     ezInt = ezInt + ez * Ki;

// // //     // 将误差PI后补偿到陀螺仪
// // //     gx = gx + Kp * ex + exInt;
// // //     gy = gy + Kp * ey + eyInt;
// // //     gz = gz + Kp * ez + ezInt; // 这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

// // //     // 四元素的微分方程
// // //     q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
// // //     q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
// // //     q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
// // //     q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

// // //     // 单位化四元数
// // //     norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
// // //     q0 = q0 * norm;
// // //     q1 = q1 * norm;
// // //     q2 = q2 * norm;
// // //     q3 = q3 * norm;

// // //     // 矩阵R 将惯性坐标系(n)转换到机体坐标系(b)
// // //     matrix[0] = q0q0 + q1q1 - q2q2 - q3q3; // 11(前列后行)
// // //     matrix[1] = 2.f * (q1q2 + q0q3);       // 12
// // //     matrix[2] = 2.f * (q1q3 - q0q2);       // 13
// // //     matrix[3] = 2.f * (q1q2 - q0q3);       // 21
// // //     matrix[4] = q0q0 - q1q1 + q2q2 - q3q3; // 22
// // //     matrix[5] = 2.f * (q2q3 + q0q1);       // 23
// // //     matrix[6] = 2.f * (q1q3 + q0q2);       // 31
// // //     matrix[7] = 2.f * (q2q3 - q0q1);       // 32
// // //     matrix[8] = q0q0 - q1q1 - q2q2 + q3q3; // 33

// // //     // 四元数转换成欧拉角(Z->Y->X)
// // //     att_angel->yaw += physical_data.z2 * RadtoDeg * 0.01f;
// // //     //	Att_Angle->yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f; // yaw
// // //     att_angel->roll = -asin(2.f * (q1q3 - q0q2)) * 57.3f;                                 // roll(负号要注意)
// // //     att_angel->pitch = -atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3) * 57.3f; // pitch
// // //     for (i = 0; i < 9; i++)
// // //     {
// // //         *(&(DCMgb[0][0]) + i) = matrix[i];
// // //     }

// // //     // // 失控保护 (调试时可注释掉)
// // //     // Safety_Check();
// // // }

// // esp_err_t IMU_updata(mpu6050_dev_t *sensor, att_angel_t *att_angel)
// // {
// //     float acce_angle[2];
// //     float gyro_angle[2];
// //     float gyro_rate[2];
// //     mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;

// //     sens->counter++;
// //     if (sens->counter == 1)
// //     {
// //         acce_angle[0] = (atan2(physical_data.y1, physical_data.z1) * RAD_TO_DEG);
// //         acce_angle[1] = (atan2(physical_data.x1, physical_data.z1) * RAD_TO_DEG);
// //         att_angel->roll = acce_angle[0];
// //         att_angel->pitch = acce_angle[1];
// //         gettimeofday(sens->timer, NULL);
// //         return ESP_OK;
// //     }

// //     struct timeval now, dt_t;
// //     gettimeofday(&now, NULL);
// //     timersub(&now, sens->timer, &dt_t);
// //     sens->dt = (float)(dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
// //     gettimeofday(sens->timer, NULL);

// //     acce_angle[0] = (atan2(physical_data.y1, physical_data.z1) * RAD_TO_DEG);
// //     acce_angle[1] = (atan2(physical_data.x1, physical_data.z1) * RAD_TO_DEG);

// //     gyro_rate[0] = physical_data.x2;
// //     gyro_rate[1] = physical_data.y2;
// //     gyro_angle[0] = gyro_rate[0] * sens->dt;
// //     gyro_angle[1] = gyro_rate[1] * sens->dt;

// //     att_angel->roll = (ALPHA * (att_angel->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
// //     att_angel->pitch = (ALPHA * (att_angel->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);

// //     return ESP_OK;
// // }
// // void IMU(att_angel_t *att_angel)
// // {
// //     raw_to_physical();
// //     // ESP_LOGI(TAG,"successful get physical data");
// //     // printf("%5.2f,%5.2f,%5.2f\n", physical_data.x1, physical_data.y1, physical_data.z1);
// //     // printf("raw:%05.6lf %05.6lf %05.6lf\traw:%05.6lf %05.6lf %05.6lf\n", raw_data.x1, raw_data.y1, raw_data.z1, physical_data.x1, physical_data.y1, physical_data.z1);
// //     esp_err_t ret = IMU_updata(mpu6050, att_angel);
// //     printf("pitch:%5.2lf\troll:%5.2lf\tyaw:%5.2lf\n", att_angel->pitch, att_angel->roll, att_angel->yaw);
// // }






// #include <stdio.h>
// #include "MPU6050.h"
// #include <math.h>
// #include <time.h>
// #include <sys/time.h>
// #include "../IIC/include/IIC.h"
// #include "unity.h"
// #include "esp_log.h"

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

// #define ALPHA 0.3f              /*!< Weight of gyroscope */
// #define RAD_TO_DEG 57.27272727f /*!< Radians to degrees */

// static void *mpu6050 = NULL;

// static const char *MPU6050TAG = "MPU6050";

// mpu6050_acce_value_t acce = {
//     .acce_x = 0,
//     .acce_y = 0,
//     .acce_z = 0};
// mpu6050_gyro_value_t gyro = {
//     .gyro_x = 0,
//     .gyro_y = 0,
//     .gyro_z = 0};
// mpu6050_raw_acce_value_t raw_acce_zero = {
//     .raw_acce_x = 0,
//     .raw_acce_y = 0,
//     .raw_acce_z = 0};
// mpu6050_raw_gyro_value_t raw_gyro_zero = {
//     .raw_gyro_x = 0,
//     .raw_gyro_y = 0,
//     .raw_gyro_z = 0};

// typedef struct
// {
//     i2c_port_t bus;
//     gpio_num_t int_pin;
//     uint16_t dev_addr;
//     uint32_t counter;
//     float dt; /*!< delay time between two measurements, dt should be small (ms level) */
//     struct timeval *timer;
// } mpu6050_dev_t;

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
//     assert(ESP_OK == ret);
//     ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
//     assert(ESP_OK == ret);
//     ret = i2c_master_write_byte(cmd, reg_start_addr, true);
//     assert(ESP_OK == ret);
//     ret = i2c_master_start(cmd);
//     assert(ESP_OK == ret);
//     ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_READ, true);
//     assert(ESP_OK == ret);
//     ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
//     assert(ESP_OK == ret);
//     ret = i2c_master_stop(cmd);
//     assert(ESP_OK == ret);
//     ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);

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
//  *函数：void *mpu6050_create(i2c_port_t port, const uint16_t dev_addr)
//  *功能：创建MPU6050句柄
//  *参数：
//  *i2c_port_t port   MPU6050IIC端口
//  *const uint16_t dev_addr   MPU6050IIC地址
//  *返回值：指向MPU6050模块信息句柄的指针
//  *备注：创建包含了MPU6050模块信息的句柄
//  ***************************************************************************************************/
// void *mpu6050_create(i2c_port_t port, const uint16_t dev_addr)
// {
//     mpu6050_dev_t *sensor = (mpu6050_dev_t *)calloc(1, sizeof(mpu6050_dev_t));
//     sensor->bus = port;
//     sensor->dev_addr = dev_addr << 1;
//     sensor->counter = 0;
//     sensor->dt = 0;
//     sensor->timer = (struct timeval *)calloc(1, sizeof(struct timeval));
//     return (void *)sensor;
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

//     mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
//     TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

//     ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
//     TEST_ASSERT_EQUAL(ESP_OK, ret);

//     ret = mpu6050_wake_up(mpu6050);
//     TEST_ASSERT_EQUAL(ESP_OK, ret);
//     uint8_t mpu6050_deviceid = 0;
//     ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
//     printf("%.2x\n", mpu6050_deviceid);
//     TEST_ASSERT_EQUAL(ESP_OK, ret);
//     TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");

//     uint16_t i = 500;
//     mpu6050_raw_acce_value_t raw1_acce;
//     mpu6050_raw_gyro_value_t raw1_gyro;
//     raw_acce_zero.raw_acce_x = 0;
//     raw_acce_zero.raw_acce_y = 0;
//     raw_acce_zero.raw_acce_z = 0;
//     raw_gyro_zero.raw_gyro_x = 0;
//     raw_gyro_zero.raw_gyro_y = 0;
//     raw_gyro_zero.raw_gyro_z = 0;
//     while (i--)
//     {
//         ret = mpu6050_get_raw_acce(mpu6050, &raw1_acce);
//         TEST_ASSERT_EQUAL(ESP_OK, ret);
//         ret = mpu6050_get_raw_gyro(mpu6050, &raw1_gyro);
//         TEST_ASSERT_EQUAL(ESP_OK, ret);

//         raw_acce_zero.raw_acce_x += raw1_acce.raw_acce_x;
//         raw_acce_zero.raw_acce_y += raw1_acce.raw_acce_y;
//         raw_acce_zero.raw_acce_z += raw1_acce.raw_acce_z - 8196;
//         raw_gyro_zero.raw_gyro_x += raw1_gyro.raw_gyro_x;
//         raw_gyro_zero.raw_gyro_y += raw1_gyro.raw_gyro_y;
//         raw_gyro_zero.raw_gyro_z += raw1_gyro.raw_gyro_z;
//     }
//     raw_acce_zero.raw_acce_x /= 500;
//     raw_acce_zero.raw_acce_y /= 500;
//     raw_acce_zero.raw_acce_z /= 500;
//     raw_gyro_zero.raw_gyro_x /= 500;
//     raw_gyro_zero.raw_gyro_y /= 500;
//     raw_gyro_zero.raw_gyro_z /= 500;

//     ESP_LOGI(MPU6050TAG, "acc zero = %d %d %d\n", raw_acce_zero.raw_acce_x, raw_acce_zero.raw_acce_y, raw_acce_zero.raw_acce_z);
//     ESP_LOGI(MPU6050TAG, "gyro zero = %d %d %d\n", raw_gyro_zero.raw_gyro_x, raw_gyro_zero.raw_gyro_y, raw_gyro_zero.raw_gyro_z);
// }

// /***************************************************************************************************
//  *函数：esp_err_t mpu6050_wake_up(void *sensor)
//  *功能：唤醒MPU6050
//  *参数：
//  *void *sensor指向MPU6050句柄的指针
//  *返回值：无
//  *备注：
//  ***************************************************************************************************/
// esp_err_t mpu6050_wake_up(void *sensor)
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
//  *函数：esp_err_t mpu6050_config(void *sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
//  *功能：配置MPU6050量程
//  *参数：
//  *void *sensor  指向MPU6050句柄的指针
//  *const mpu6050_acce_fs_t acce_fs   acce量程配置项
//  *const mpu6050_gyro_fs_t gyro_fs   gyro量程配置项
//  *返回值：错误值
//  *备注：
//  ***************************************************************************************************/
// esp_err_t mpu6050_config(void *sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
// {
//     uint8_t config_regs[2] = {gyro_fs << 3, acce_fs << 3};
//     return mpu6050_write(sensor, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
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
//  *函数：esp_err_t mpu6050_get_acce(void * sensor, mpu6050_acce_value_t *const acce_value)
//  *功能：获取ACCE数据
//  *参数：
//  *sensor mpu6050句柄
//  *const acce_value 指向加速度物理量值的指针
//  *返回值：错误代码
//  *备注：只获取除零漂的加速度并转换为实际的物理量
//  ***************************************************************************************************/
// esp_err_t mpu6050_get_acce(void *sensor, mpu6050_acce_value_t *const acce_value)
// {
//     esp_err_t ret;
//     float acce_sensitivity;
//     mpu6050_raw_acce_value_t raw_acce = {
//         .raw_acce_x = 0,
//         .raw_acce_y = 0,
//         .raw_acce_z = 0};

//     ret = mpu6050_get_acce_sensitivity(sensor, &acce_sensitivity);
//     if (ret != ESP_OK)
//     {
//         return ret;
//     }
//     ret = mpu6050_get_raw_acce(sensor, &raw_acce);
//     if (ret != ESP_OK)
//     {
//         return ret;
//     }

//     acce_value->acce_x = (raw_acce.raw_acce_x - raw_acce_zero.raw_acce_x) / acce_sensitivity;
//     acce_value->acce_y = (raw_acce.raw_acce_y - raw_acce_zero.raw_acce_y) / acce_sensitivity;
//     acce_value->acce_z = (raw_acce.raw_acce_z - raw_acce_zero.raw_acce_z) / acce_sensitivity;
//     return ESP_OK;
// }
// /***************************************************************************************************
//  *函数：esp_err_t mpu6050_get_gyro(void * sensor, mpu6050_gyro_value_t *const gyro_value)
//  *功能：获取GYRO数据
//  *参数：
//  *sensor mpu6050句柄
//  *const gyro_value 指向角加速度值物理量的指针
//  *返回值：错误代码
//  *备注：只获取角加速度值除零漂并转换为实际的物理量
//  ***************************************************************************************************/
// esp_err_t mpu6050_get_gyro(void *sensor, mpu6050_gyro_value_t *const gyro_value)
// {
//     esp_err_t ret;
//     float gyro_sensitivity;
//     mpu6050_raw_gyro_value_t raw_gyro = {
//         .raw_gyro_x = 0,
//         .raw_gyro_y = 0,
//         .raw_gyro_z = 0};

//     ret = mpu6050_get_gyro_sensitivity(sensor, &gyro_sensitivity);
//     if (ret != ESP_OK)
//     {
//         return ret;
//     }
//     ret = mpu6050_get_raw_gyro(sensor, &raw_gyro);
//     if (ret != ESP_OK)
//     {
//         return ret;
//     }

//     gyro_value->gyro_x = (raw_gyro.raw_gyro_x - raw_gyro_zero.raw_gyro_x) / gyro_sensitivity;
//     gyro_value->gyro_y = (raw_gyro.raw_gyro_y - raw_gyro_zero.raw_gyro_y) / gyro_sensitivity;
//     gyro_value->gyro_z = (raw_gyro.raw_gyro_z - raw_gyro_zero.raw_gyro_z) / gyro_sensitivity;
//     return ESP_OK;
// }

// /***************************************************************************************************
//  *函数：esp_err_t mpu6050_get_acce_sensitivity(void * sensor, float *const acce_sensitivity)
//  *功能：获取ACCE的分度值
//  *参数：
//  *sensor mpu6050句柄
//  *const acce_sensitivity 指向加速度分度值的指针
//  *返回值：错误代码
//  *备注：获取ACCE加速的分度值
//  ***************************************************************************************************/
// esp_err_t mpu6050_get_acce_sensitivity(void *sensor, float *const acce_sensitivity)
// {
//     esp_err_t ret;
//     uint8_t acce_fs;
//     ret = mpu6050_read(sensor, MPU6050_ACCEL_CONFIG, &acce_fs, 1);
//     acce_fs = (acce_fs >> 3) & 0x03;
//     switch (acce_fs)
//     {
//     case ACCE_FS_2G:
//         *acce_sensitivity = 16384;
//         break;

//     case ACCE_FS_4G:
//         *acce_sensitivity = 8192;
//         break;

//     case ACCE_FS_8G:
//         *acce_sensitivity = 4096;
//         break;

//     case ACCE_FS_16G:
//         *acce_sensitivity = 2048;
//         break;

//     default:
//         break;
//     }
//     return ret;
// }

// /***************************************************************************************************
//  *函数：esp_err_t mpu6050_get_gyro_sensitivity(void * sensor, float *const gyro_sensitivity)
//  *功能：获取GYRO的分度值
//  *参数：
//  *sensor mpu6050句柄
//  *const gyro_sensitivity 指向加速度分度值的指针
//  *返回值：错误代码
//  *备注：获取GYRO加速的分度值
//  ***************************************************************************************************/
// esp_err_t mpu6050_get_gyro_sensitivity(void *sensor, float *const gyro_sensitivity)
// {
//     esp_err_t ret;
//     uint8_t gyro_fs;
//     ret = mpu6050_read(sensor, MPU6050_GYRO_CONFIG, &gyro_fs, 1);
//     gyro_fs = (gyro_fs >> 3) & 0x03;
//     switch (gyro_fs)
//     {
//     case GYRO_FS_250DPS:
//         *gyro_sensitivity = 131;
//         break;

//     case GYRO_FS_500DPS:
//         *gyro_sensitivity = 65.5;
//         break;

//     case GYRO_FS_1000DPS:
//         *gyro_sensitivity = 32.8;
//         break;

//     case GYRO_FS_2000DPS:
//         *gyro_sensitivity = 16.4;
//         break;

//     default:
//         break;
//     }
//     return ret;
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

// esp_err_t mpu6050_complimentory_filter(void *sensor, const mpu6050_acce_value_t *const acce_value,
//                                        const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle)
// {
//     float acce_angle[2];
//     float gyro_angle[2];
//     float gyro_rate[2];
//     mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;

//     sens->counter++;
//     if (sens->counter == 1)
//     {
//         acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
//         acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
//         complimentary_angle->roll = acce_angle[0];
//         complimentary_angle->pitch = acce_angle[1];
//         gettimeofday(sens->timer, NULL);
//         return ESP_OK;
//     }

//     struct timeval now, dt_t;
//     gettimeofday(&now, NULL);
//     timersub(&now, sens->timer, &dt_t);
//     sens->dt = (float)(dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
//     gettimeofday(sens->timer, NULL);

//     acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
//     acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);

//     gyro_rate[0] = gyro_value->gyro_x;
//     gyro_rate[1] = gyro_value->gyro_y;
//     gyro_angle[0] = gyro_rate[0] * sens->dt;
//     gyro_angle[1] = gyro_rate[1] * sens->dt;
    
//     complimentary_angle->roll = (ALPHA * (complimentary_angle->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
//     complimentary_angle->pitch = (ALPHA * (complimentary_angle->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);

//     return ESP_OK;
// }
// /***************************************************************************************************
//  *函数：esp_err_t IMU(complimentary_angle_t *complimentary_angle)
//  *功能：获取俯仰角、横滚角、Z轴加速度
//  *参数：
//  *complimentary_angle_t *complimentary_angle 指向回传数据的指针（pitch、row、z_acce）
//  *返回值：错误代码
//  *备注：与main函数的接口函数，传递读取和解算出的数据
//  ***************************************************************************************************/
// esp_err_t IMU(complimentary_angle_t *complimentary_angle)
// {

//     esp_err_t ret;
//     ret = mpu6050_get_acce(mpu6050, &acce);
//     TEST_ASSERT_EQUAL(ESP_OK, ret);
//     // ESP_LOGI(MPU6050TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

//     ret = mpu6050_get_gyro(mpu6050, &gyro);
//     TEST_ASSERT_EQUAL(ESP_OK, ret);
//     // ESP_LOGI(MPU6050TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

//     ret = mpu6050_complimentory_filter(mpu6050, &acce, &gyro, complimentary_angle);
//     TEST_ASSERT_EQUAL(ESP_OK, ret);
//     complimentary_angle->accz = acce.acce_z;
//     // ESP_LOGI(MPU6050TAG, "pitch:%.2f roll:%.2f \n", complimentary_angle->pitch,complimentary_angle->roll);

//     return ret;
// }






