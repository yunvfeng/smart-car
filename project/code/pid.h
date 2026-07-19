#ifndef __PID_H_
#define __PID_H_

#include "zf_common_headfile.h"
#include "motor.h"
#include "servo.h"

/* Q10 表示参数放大 1024 倍保存，计算时再右移 10 位还原。 */
#define PID_Q_SHIFT             10
#define PID_Q_ONE               (1L << PID_Q_SHIFT)

#define SERVO_KP_TURN_Q10       6575L
#define SERVO_KP_STRAIGHT_Q10   2000L
#define SERVO_KD_Q10            498L
#define SERVO_KF_Q10            200L
#define SERVO_KP2_Q10           0L
#define SERVO_KG_Q10            22L

#define MOTOR_KP_Q10            654L
#define MOTOR_KI_Q10            150L
#define MOTOR_KD_Q10            0L

#define OUT_MAX                 MOTOR_MAX_LIMIT
#define OUT_MIN                 (-MOTOR_MAX_LIMIT)

typedef struct
{
    int32 kp;
    int32 ki;
    int32 kd;
    int32 kg;
    int32 kf;
    int32 kp2;

    int16 err1;
    int16 err2;
    int32 err_sum;
    int32 out;
} PID_t;

extern volatile uint16 Out_servo;
extern volatile PID_t pid_rf;
extern volatile PID_t pid_lf;
extern volatile PID_t servo_pidf;

/* 把 32 位计算结果限制到 int16 范围。 */
int16 limit_int16(int32 x, int16 min_v, int16 max_v);
/* 根据中线误差计算舵机输出。 */
void PID_servof(volatile PID_t *pid);
/* Use a caller-supplied image target column without modifying mid_line[]. */
void PID_servof_Target(volatile PID_t *pid, int16 target_col);
/* Clear controller history and output while preserving configured gains. */
void PID_Reset(volatile PID_t *pid);
/* 原子设置/读取带符号的陀螺仪横摆阻尼增益（Q10）。 */
void PID_Set_Servo_Gyro_Gain(int16 gain_q10);
int16 PID_Get_Servo_Gyro_Gain(void);
/* 增量式 PID，用于左右电机速度环。 */
void Increment_PID(volatile PID_t *pid, int16 tar_val, int16 act_val);

#endif
