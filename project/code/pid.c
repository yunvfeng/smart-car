#include "pid.h"
#include "image.h"
#include "gyro.h"

volatile uint16 Out_servo = SERVO_DUTY_MID;

/* 左右电机各一套 PI，参数在 pid.h 里改。 */
volatile PID_t pid_rf = {MOTOR_KP_Q10, MOTOR_KI_Q10, MOTOR_KD_Q10, 0, 0, 0, 0, 0, 0, 0};
volatile PID_t pid_lf = {MOTOR_KP_Q10, MOTOR_KI_Q10, MOTOR_KD_Q10, 0, 0, 0, 0, 0, 0, 0};

/* 舵机根据中线偏差、偏差变化和弯道趋势一起算输出。 */
volatile PID_t servo_pidf = {SERVO_KP_TURN_Q10, 0, SERVO_KD_Q10,
                             SERVO_KG_Q10, SERVO_KF_Q10, SERVO_KP2_Q10,
                             0, 0, 0, 0};

/* 返回 int16 的绝对值，供舵机非线性项计算使用。 */
static int16 abs16_local(int16 x)
{
    return (x < 0) ? (int16)(-x) : x;
}

/* 将 32 位计算结果限制到给定 int16 范围内。 */
int16 limit_int16(int32 x, int16 min_v, int16 max_v)
{
    if (x > max_v) return max_v;
    if (x < min_v) return min_v;
    return (int16)x;
}

/* 增量式 PID，用于左右电机速度闭环。 */
void Increment_PID(volatile PID_t *pid, int16 tar_val, int16 act_val)
{
    int16 error;
    int16 d_err;
    int16 dd_err;
    int32 delta;

    error = tar_val - act_val;
    d_err = error - pid->err1;
    dd_err = error - (int16)(2 * pid->err1) + pid->err2;

    delta = (pid->kp * (int32)d_err +
             pid->ki * (int32)error +
             pid->kd * (int32)dd_err) >> PID_Q_SHIFT;

    pid->out += delta;
    if (pid->out > OUT_MAX) pid->out = OUT_MAX;
    if (pid->out < OUT_MIN) pid->out = OUT_MIN;

    pid->err2 = pid->err1;
    pid->err1 = error;
}

/* 舵机 PID：根据中线偏差、偏差变化和曲率前馈计算舵机占空比。 */
void PID_servof(volatile PID_t *pid)
{
    uint8 ref;
    uint8 near_ref;
    int16 err;
    int16 d_err;
    int16 curve;
    int16 abs_err;
    int16 gyro_correction;
    int32 out;
    int32 duty;

    ref = controlReferenceLine;
    if (ref >= SEARCH_IMAGE_H) {
        ref = SEARCH_IMAGE_H - 1;
    }

    near_ref = (uint8)(ref + 20);
    if (near_ref >= SEARCH_IMAGE_H) {
        near_ref = SEARCH_IMAGE_H - 1;
    }

    err = (int16)mid_line[ref] - (int16)Mid_Col;
    curve = (int16)mid_line[ref] - (int16)mid_line[near_ref];
    d_err = err - pid->err1;
    abs_err = abs16_local(err);

    out = pid->kp * (int32)err;
    out += pid->kd * (int32)d_err;
    out += pid->kf * (int32)curve;
    out += pid->kp2 * (int32)err * (int32)abs_err;
    out >>= PID_Q_SHIFT;

    gyro_correction = limit_int16(
        (pid->kg * (int32)g_gyro_z_for_servo) / PID_Q_ONE,
        -GYRO_SERVO_CORRECTION_LIMIT,
        GYRO_SERVO_CORRECTION_LIMIT);
    Gyro_Set_Servo_Correction(gyro_correction);
    out -= gyro_correction;

    out = limit_int16(out, -MAX_TURN, MAX_TURN);

    duty = (int32)SERVO_DUTY_MID - out;
    duty = limit_int16(duty, SERVO_DUTY_MIN, SERVO_DUTY_MAX);

    Out_servo = (uint16)duty;
    pid->err2 = pid->err1;
    pid->err1 = err;
    pid->out = out;
}

void PID_Set_Servo_Gyro_Gain(int16 gain_q10)
{
    bit interrupt_state;

    interrupt_state = EA;
    EA = 0;
    servo_pidf.kg = gain_q10;
    EA = interrupt_state;
}

int16 PID_Get_Servo_Gyro_Gain(void)
{
    bit interrupt_state;
    int32 gain_q10;

    interrupt_state = EA;
    EA = 0;
    gain_q10 = servo_pidf.kg;
    EA = interrupt_state;

    return limit_int16(gain_q10, -32768, 32767);
}
