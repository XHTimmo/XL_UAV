#ifndef _MOTOR_H
#define _MOTOR_H
#include <stdint.h>
#define MOTO_GPIO1  6
#define MOTO_GPIO2  8
#define MOTO_GPIO3  45
#define MOTO_GPIO4  1

typedef struct{
    uint32_t motor1;
    uint32_t motor2;
    uint32_t motor3;
    uint32_t motor4;
} motor_pwm_t;

void MotorControl_Init(void);
void set_pwm1(uint32_t pwm);
void set_pwm2(uint32_t pwm);
void set_pwm3(uint32_t pwm);
void set_pwm4(uint32_t pwm);
#endif