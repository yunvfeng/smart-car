#include "motor.h"
#include "pid.h"
#include "servo.h"
#include "image.h"

int16 encoder_data_r = 0;
int16 encoder_data_l = 0;

int16 tar_speed = BASE_TARGET_SPEED;
volatile int16 min_speed = MIN_SPEED;
volatile int16 max_speed = MAX_SPEED;
volatile int16 target_speed_l = BASE_TARGET_SPEED;
volatile int16 target_speed_r = BASE_TARGET_SPEED;
volatile int16 motor_pwm_l = 0;
volatile int16 motor_pwm_r = 0;

static volatile motor_safety_mode_enum motor_safety_mode = MOTOR_SAFETY_NORMAL;
static volatile int16 motor_safety_cap = WHEEL_TARGET_MIN;
static volatile uint8 motor_force_stopped = 0u;

static int16 clamp_speed_setting(int16 speed)
{
    if (speed < WHEEL_TARGET_MIN) return WHEEL_TARGET_MIN;
    if (speed > MAX_SPEED_TUNE_MAX) return MAX_SPEED_TUNE_MAX;
    return speed;
}

static void motor_get_safety_command(motor_safety_mode_enum *mode, int16 *cap)
{
    bit interrupt_state;

    interrupt_state = EA;
    EA = 0;
    if (mode != NULL) {
        *mode = motor_safety_mode;
    }
    if (cap != NULL) {
        *cap = motor_safety_cap;
    }
    EA = interrupt_state;
}

void Motor_Set_Safety_Command(motor_safety_mode_enum mode, int16 cap)
{
    bit interrupt_state;

    if (mode == MOTOR_SAFETY_STOP) {
        Motor_Force_Stop_Reset();
        return;
    }

    if (mode != MOTOR_SAFETY_CAP) {
        mode = MOTOR_SAFETY_NORMAL;
    }
    cap = clamp_speed_setting(cap);

    interrupt_state = EA;
    EA = 0;
    motor_safety_cap = cap;
    motor_safety_mode = mode;
    motor_force_stopped = 0u;
    EA = interrupt_state;
}

uint8 Motor_Is_Force_Stopped(void)
{
    return motor_force_stopped;
}

void Motor_Force_Stop_Reset(void)
{
    bit interrupt_state;

    interrupt_state = EA;
    EA = 0;

    motor_safety_mode = MOTOR_SAFETY_STOP;
    motor_force_stopped = 1u;
    tar_speed = 0;
    target_speed_l = 0;
    target_speed_r = 0;
    motor_pwm_l = 0;
    motor_pwm_r = 0;
    PID_Reset(&pid_lf);
    PID_Reset(&pid_rf);

    /* Bypass minimum PWM and both slew-rate limits for a true hard stop. */
    Motor_control(PWM_L, 0);
    Motor_control(PWM_R, 0);

    EA = interrupt_state;
}

void Motor_Set_Speed_Range(int16 speed_min, int16 speed_max)
{
    bit interrupt_state;

    speed_min = clamp_speed_setting(speed_min);
    speed_max = clamp_speed_setting(speed_max);
    if (speed_min > speed_max) {
        speed_min = speed_max;
    }

    interrupt_state = EA;
    EA = 0;
    min_speed = speed_min;
    max_speed = speed_max;
    EA = interrupt_state;
}

void Motor_Get_Speed_Range(int16 *speed_min, int16 *speed_max)
{
    bit interrupt_state;

    if (speed_min == NULL || speed_max == NULL) {
        return;
    }

    interrupt_state = EA;
    EA = 0;
    *speed_min = min_speed;
    *speed_max = max_speed;
    EA = interrupt_state;
}

/* 限制电机 PWM 输出，防止超过驱动允许范围。 */
static int16 clamp_motor(int16 x)
{
    if (x > MOTOR_MAX_LIMIT) return MOTOR_MAX_LIMIT;
    if (x < -MOTOR_MAX_LIMIT) return -MOTOR_MAX_LIMIT;
    return x;
}

/* 将单轮目标速度限制在可用范围内。 */
static int16 clamp_wheel_target(int16 x, int16 speed_ceiling)
{
    if (x > speed_ceiling) return speed_ceiling;
    if (x < WHEEL_TARGET_MIN) return WHEEL_TARGET_MIN;
    return x;
}

/*
 * 对 PI 请求的 PWM 加死区补偿和变化率限制。
 * 返回值是本周期真正写入电机的 PWM，后续同步回 pid.out。
 */
static int16 shape_motor_pwm(int16 target, int32 requested, int16 previous)
{
    int32 next;
    int32 upper;
    int32 lower;

    next = requested;
    if (target > 0 && next < MOTOR_MIN_EFFECTIVE_PWM) {
        next = MOTOR_MIN_EFFECTIVE_PWM;
    }

    if (next > MOTOR_MAX_LIMIT) {
        next = MOTOR_MAX_LIMIT;
    } else if (next < -MOTOR_MAX_LIMIT) {
        next = -MOTOR_MAX_LIMIT;
    }

    upper = (int32)previous + MOTOR_PWM_RISE_STEP;
    lower = (int32)previous - MOTOR_PWM_FALL_STEP;
    if (next > upper) {
        next = upper;
    } else if (next < lower) {
        next = lower;
    }

    return clamp_motor((int16)next);
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
    Motor_control(PWM_L, MOTOR_MIN_EFFECTIVE_PWM);
    system_delay_ms(1000);
    Motor_control(PWM_L, -MOTOR_MIN_EFFECTIVE_PWM);
    system_delay_ms(1000);
    Motor_control(PWM_L, 0);

    Motor_control(PWM_R, MOTOR_MIN_EFFECTIVE_PWM);
    system_delay_ms(1000);
    Motor_control(PWM_R, -MOTOR_MIN_EFFECTIVE_PWM);
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
    uint16 turn_limit;
    uint32 turn_square;
    uint32 max_turn_square;
    int16 speed_floor;
    int16 schedule_ceiling;
    int16 output_ceiling;
    int16 scheduled_speed;
    int16 turn_diff;
    int16 left_target;
    int16 right_target;
    int16 safety_cap;
    motor_safety_mode_enum safety_mode;

    if (Out_servo > SERVO_CENTER) {
        tp_turn = Out_servo - SERVO_CENTER;
        turn_limit = SERVO_DUTY_MAX - SERVO_CENTER;
    } else {
        tp_turn = SERVO_CENTER - Out_servo;
        turn_limit = SERVO_CENTER - SERVO_DUTY_MIN;
    }

    if (turn_limit > MAX_TURN) {
        turn_limit = MAX_TURN;
    }
    if (tp_turn > turn_limit) {
        tp_turn = turn_limit;
    }

    /* 每个控制周期使用同一份原子快照，避免运行时更新时混用新旧范围。 */
    Motor_Get_Speed_Range(&speed_floor, &schedule_ceiling);
    motor_get_safety_command(&safety_mode, &safety_cap);

    if (schedule_ceiling < WHEEL_TARGET_MIN) {
        schedule_ceiling = WHEEL_TARGET_MIN;
    } else if (schedule_ceiling > MAX_SPEED_TUNE_MAX) {
        schedule_ceiling = MAX_SPEED_TUNE_MAX;
    }

    if (speed_floor < WHEEL_TARGET_MIN) {
        speed_floor = WHEEL_TARGET_MIN;
    } else if (speed_floor > schedule_ceiling) {
        speed_floor = schedule_ceiling;
    }

    /* The curve uses entered maximum n; normal output is gated by the configured margin. */
    output_ceiling = (int16)(schedule_ceiling - MOTOR_SPEED_GATE_MARGIN);
    if (safety_mode == MOTOR_SAFETY_CAP && safety_cap < output_ceiling) {
        output_ceiling = safety_cap;
    }
    if (output_ceiling < WHEEL_TARGET_MIN) {
        output_ceiling = WHEEL_TARGET_MIN;
    } else if (output_ceiling > MAX_SPEED_TUNE_MAX) {
        output_ceiling = MAX_SPEED_TUNE_MAX;
    }

    /* 转角平方映射：中小弯少降速，大转角仍平滑降到可调最低速度。 */
    turn_square = (uint32)tp_turn * (uint32)tp_turn;
    max_turn_square = (uint32)turn_limit * (uint32)turn_limit;
    scheduled_speed = schedule_ceiling -
        (int16)(((uint32)(schedule_ceiling - speed_floor) * turn_square) / max_turn_square);
    tar_speed = (scheduled_speed > output_ceiling) ? output_ceiling : scheduled_speed;

    turn_diff = (int16)(((uint32)TURN_DIFF_MAX * tp_turn) / turn_limit);
    left_target = tar_speed;
    right_target = tar_speed;

    /* 沿用当前舵机方向约定：大于中值时右轮为内轮。 */
    if (Out_servo > SERVO_CENTER) {
        left_target += turn_diff;
        right_target -= turn_diff;
    } else if (Out_servo < SERVO_CENTER) {
        left_target -= turn_diff;
        right_target += turn_diff;
    }

    target_speed_l = clamp_wheel_target(left_target, output_ceiling);
    target_speed_r = clamp_wheel_target(right_target, output_ceiling);
}

/* 电机速度闭环：计算目标速度，更新左右 PI，再输出 PWM。 */
void Motor_Loop(void)
{
    motor_safety_mode_enum safety_mode;

    motor_get_safety_command(&safety_mode, NULL);
    if (safety_mode == MOTOR_SAFETY_STOP) {
        Motor_Force_Stop_Reset();
        return;
    }

    Dream_speed();

    Increment_PID(&pid_lf, target_speed_l, encoder_data_l);
    Increment_PID(&pid_rf, target_speed_r, encoder_data_r);

    motor_pwm_l = shape_motor_pwm(target_speed_l, pid_lf.out, motor_pwm_l);
    motor_pwm_r = shape_motor_pwm(target_speed_r, pid_rf.out, motor_pwm_r);

    /* PI 内部输出与实际 PWM 保持一致，不留隐藏积累。 */
    pid_lf.out = motor_pwm_l;
    pid_rf.out = motor_pwm_r;

    Motor_control(PWM_L, motor_pwm_l);
    Motor_control(PWM_R, motor_pwm_r);
}
