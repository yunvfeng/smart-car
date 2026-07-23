#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

extern int16 encoder_data_r;
extern int16 encoder_data_l;
extern int16 tar_speed;
extern volatile int16 target_speed_l;
extern volatile int16 target_speed_r;
extern volatile int16 motor_pwm_l;
extern volatile int16 motor_pwm_r;

typedef enum
{
    MOTOR_SAFETY_NORMAL = 0,
    MOTOR_SAFETY_CAP,
    MOTOR_SAFETY_STOP
} motor_safety_mode_enum;

/* 左右电机接线。 */
#define DIR_L              IO_P74
#define PWM_L              PWMB_CH3_P76

#define DIR_R              IO_P75
#define PWM_R              PWMB_CH4_P77

#define MOTOR_FREQ         17000
#define MOTOR_MAX_LIMIT    7500
#define MOTOR_MIN_EFFECTIVE_PWM  2250
#define MOTOR_PWM_RISE_STEP      250
#define MOTOR_PWM_FALL_STEP      400

#define ENCODER_DIR_1          PWMA_ENCODER
#define ENCODER_DIR_PULSE_1    PWMA_ENCODER_CH1P_P60
#define ENCODER_DIR_DIR_1      PWMA_ENCODER_CH2P_P62

#define ENCODER_DIR_2          PWMC_ENCODER
#define ENCODER_DIR_PULSE_2    PWMC_ENCODER_CH1P_P40
#define ENCODER_DIR_DIR_2      PWMC_ENCODER_CH2P_P42

/* 这里的速度单位是 15 ms 内读到的编码器脉冲数。 */
#define MAX_SPEED          280
#define MAX_SPEED_TUNE_MAX 700
#define MIN_SPEED          200
#define BASE_TARGET_SPEED  200
#define WHEEL_TARGET_MIN   200
#define TURN_DIFF_MAX      8
#define MOTOR_SPEED_GATE_MARGIN  30

/* 运行时最低/最高目标速度；WiFi 调试路径关闭时保持默认值。 */
extern volatile int16 min_speed;
extern volatile int16 max_speed;

/* 原子更新或读取最低/最高目标速度。 */
void Motor_Set_Speed_Range(int16 speed_min, int16 speed_max);
void Motor_Get_Speed_Range(int16 *speed_min, int16 *speed_max);
/* Atomically select normal, speed-capped, or hard-stop operation. */
void Motor_Set_Safety_Command(motor_safety_mode_enum mode, int16 cap);
/* Read whether the independent manual hard-stop command is currently latched. */
uint8 Motor_Is_Force_Stopped(void);
/* Latch STOP, clear both PI controllers, and write zero PWM immediately. */
void Motor_Force_Stop_Reset(void);

/* 初始化电机 PWM 和方向引脚。 */
void Motor_Init(void);
/* 初始化左右编码器。 */
void Encoder_Init(void);
/* 电机正反转测试。 */
void motor_test(void);
/* 读取编码器脉冲并清零。 */
void Encoder_GetValue(void);
/* 电机速度闭环主函数。 */
void Motor_Loop(void);
/* 控制指定电机的方向和 PWM 输出。 */
void Motor_control(pwm_channel_enum wheel, int16 speed);
/* 根据转向幅度计算目标速度和差速。 */
void Dream_speed(void);

#endif
