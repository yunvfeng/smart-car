#ifndef __SERVO_H_
#define __SERVO_H_

#include "zf_common_headfile.h"

/* 舵机 PWM 通道和安全限幅。 */
#define STEER_PWM_PIN    PWME_CH2P_PA2
#define SERVO_FREQ       100

#define SERVO_DUTY_MID   850
#define SERVO_DUTY_MIN   690
#define SERVO_DUTY_MAX   1040
#define SERVO_CENTER     SERVO_DUTY_MID
#define MAX_TURN         ((SERVO_DUTY_MAX - SERVO_DUTY_MIN) / 2)
#define SERVO_PATH_BIAS_STEP_PER_TICK  6

/* 188 宽图像的中心列是 94，主要用第 80 行中线控制舵机。 */
#define controlReferenceLine 80
#define Mid_Col              94

extern volatile uint16 Out_servo;

typedef enum
{
    SERVO_MODE_VISION = 0,
    SERVO_MODE_PATH_BIAS,
    SERVO_MODE_CENTER
} servo_mode_enum;

/* 初始化舵机 PWM。 */
void servo_init(void);
/* 舵机控制循环，计算并输出 PWM。 */
void Servo_Loop(void);
/* Change steering mode; transitions clear PID history. */
void Servo_Set_Mode(servo_mode_enum mode);
/* Set PATH_BIAS target offset; positive values move the target column right. */
void Servo_Set_Path_Bias(int16 bias);
/* Clear steering PID history without modifying mid_line[]. */
void Servo_Reset_Controller(void);
/* 设置舵机占空比，并自动限幅。 */
void Set_Servo_Duty(uint32 duty);

#endif
