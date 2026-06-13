#include "servo.h"
#include "pid.h"
#include "image.h"

/* 初始化舵机 PWM，并打开 PWME 输出。 */
void servo_init(void)
{
    pwm_init(STEER_PWM_PIN, SERVO_FREQ, SERVO_DUTY_MID);

    /* 打开 PWME 输出，否则舵机 PWM 不会真正输出到引脚。 */
    PWME_BKR |= 0x80;
}

/* 设置舵机占空比，写入 PWM 前先限制在安全范围内。 */
void Set_Servo_Duty(uint32 duty)
{
    if (duty > SERVO_DUTY_MAX) {
        duty = SERVO_DUTY_MAX;
    } else if (duty < SERVO_DUTY_MIN) {
        duty = SERVO_DUTY_MIN;
    }

    pwm_set_duty(STEER_PWM_PIN, duty);
}

/* 舵机控制循环：根据当前中线误差计算并输出新的舵机 PWM。 */
void Servo_Loop(void)
{
    Error_sum();
    PID_servof(&servo_pidf);
    Set_Servo_Duty((uint32)Out_servo);
}
