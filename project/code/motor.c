#include "motor.h"
#include "pid.h"
#include "servo.h"
#include "image.h"

int16 encoder_data_r = 0;
int16 encoder_data_l = 0;

int16 tar_speed = BASE_TARGET_SPEED;
int16 diff_left = 0;
int16 diff_right = 0;
int16 diff_kp_q10 = 0;

/* 限制电机 PWM 输出，防止超过驱动允许范围。 */
static int16 clamp_motor(int16 x)
{
    if (x > MOTOR_MAX_LIMIT) return MOTOR_MAX_LIMIT;
    if (x < -MOTOR_MAX_LIMIT) return -MOTOR_MAX_LIMIT;
    return x;
}

/* 初始化左右轮编码器通道。 */
void Encoder_Init(void)
{
    encoder_quad_init(ENCODER_DIR_1, ENCODER_DIR_PULSE_1, ENCODER_DIR_DIR_1);
    encoder_quad_init(ENCODER_DIR_2, ENCODER_DIR_PULSE_2, ENCODER_DIR_DIR_2);
}

/* 初始化左右电机 PWM 和方向控制引脚。 */
void Motor_Init(void)
{
    pwm_init(PWM_L, MOTOR_FREQ, 0);
    pwm_init(PWM_R, MOTOR_FREQ, 0);

    gpio_init(DIR_L, GPO, 0, GPO_PUSH_PULL);
    gpio_init(DIR_R, GPO, 0, GPO_PUSH_PULL);
}

/* 电机正反转测试函数，用于单独检查左右电机接线和方向。 */
void motor_test(void)
{
    Motor_control(PWM_L, 1000);
    system_delay_ms(1000);
    Motor_control(PWM_L, -1000);
    system_delay_ms(1000);
    Motor_control(PWM_L, 0);

    Motor_control(PWM_R, 1000);
    system_delay_ms(1000);
    Motor_control(PWM_R, -1000);
    system_delay_ms(1000);
    Motor_control(PWM_R, 0);
}

/* 控制指定电机输出，speed 正负决定方向，绝对值决定 PWM 占空比。 */
void Motor_control(pwm_channel_enum wheel, int16 speed)
{
    speed = clamp_motor(speed);

    if (wheel == PWM_L) {
        if (speed >= 0) {
            gpio_set_level(DIR_L, 1);
            pwm_set_duty(PWM_L, (uint32)speed);
        } else {
            gpio_set_level(DIR_L, 0);
            pwm_set_duty(PWM_L, (uint32)(-speed));
        }
    } else {
        if (speed >= 0) {
            gpio_set_level(DIR_R, 1);
            pwm_set_duty(PWM_R, (uint32)speed);
        } else {
            gpio_set_level(DIR_R, 0);
            pwm_set_duty(PWM_R, (uint32)(-speed));
        }
    }
}

/* 读取左右编码器脉冲并清零，供本周期速度闭环使用。 */
void Encoder_GetValue(void)
{
    /* 右轮编码器安装方向相反，所以读取后取负。 */
    encoder_data_l = encoder_get_count(ENCODER_DIR_2);
    encoder_clear_count(ENCODER_DIR_2);

    encoder_data_r = -encoder_get_count(ENCODER_DIR_1);
    encoder_clear_count(ENCODER_DIR_1);
}

/* 根据当前舵机转角降低目标速度，并计算左右轮差速补偿。 */
void Dream_speed(void)
{
    uint16 tp_turn;
    int16 delta;
    int32 diff_tmp;

    delta = (int16)(err_sum - Mid_Col);

    if (Out_servo > SERVO_CENTER) {
        tp_turn = Out_servo - SERVO_CENTER;
        diff_tmp = ((int32)delta * (int32)diff_kp_q10) >> 10;
        diff_left  = (int16)(-diff_tmp);
        diff_right = (int16)((diff_tmp * 76L) / 100L);
    } else {
        tp_turn = SERVO_CENTER - Out_servo;
        delta = -delta;
        diff_tmp = ((int32)delta * (int32)diff_kp_q10) >> 10;
        diff_left  = (int16)((diff_tmp * 76L) / 100L);
        diff_right = (int16)(-diff_tmp);
    }

    if (tp_turn > MAX_TURN) {
        tp_turn = MAX_TURN;
    }

    tar_speed = MAX_SPEED -
        (int16)(((uint32)(MAX_SPEED - MIN_SPEED) * tp_turn) / MAX_TURN);
}

/* 电机速度闭环：计算目标速度，更新左右 PI，再输出 PWM。 */
void Motor_Loop(void)
{
    Dream_speed();

    Increment_PID(&pid_lf, (int16)(tar_speed + diff_left),  encoder_data_l);
    Increment_PID(&pid_rf, (int16)(tar_speed + diff_right), encoder_data_r);

    Motor_control(PWM_L, (int16)pid_lf.out);
    Motor_control(PWM_R, (int16)pid_rf.out);
}
