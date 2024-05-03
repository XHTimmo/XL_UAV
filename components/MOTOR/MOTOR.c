#include "MOTOR.h"
#include "driver/ledc.h"
#include <stdio.h>
#define speed 4000
void MotorControl_Init(void) {
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                    .duty_resolution = LEDC_TIMER_13_BIT,
                                    .timer_num = LEDC_TIMER_0,
                                    .freq_hz = 4000,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);
  ledc_channel_config_t ledc_channer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                        .channel = LEDC_CHANNEL_0,
                                        .timer_sel = LEDC_TIMER_0,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .gpio_num = MOTO_GPIO1,
                                        .duty = 4096,
                                        .hpoint = 0};
  ledc_channel_config(&ledc_channer);
  ledc_channer.channel = LEDC_CHANNEL_1;
  ledc_channer.gpio_num = MOTO_GPIO2;
  ledc_channel_config(&ledc_channer);
  ledc_channer.channel = LEDC_CHANNEL_2;
  ledc_channer.gpio_num = MOTO_GPIO3;
  ledc_channel_config(&ledc_channer);
  ledc_channer.channel = LEDC_CHANNEL_3;
  ledc_channer.gpio_num = MOTO_GPIO4;
  ledc_channel_config(&ledc_channer);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, speed);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, speed);
}

void set_pwm1(uint32_t pwm) {
  // printf("set pwm1 %ld\n",pwm);
  if (pwm > 4000) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4000);
  } else {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm);
  }
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}
void set_pwm2(uint32_t pwm) {
  if (pwm > 4000) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 4000);
  } else {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, pwm);
  }
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}
void set_pwm3(uint32_t pwm) {
  if (pwm > 4000) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 4000);
  } else {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, pwm);
  }
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}
void set_pwm4(uint32_t pwm) {
  if (pwm > 4000) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 4000);
  } else {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, pwm);
  }
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}