#include "servo.h"
#include "pid.h"
#include "image.h"
#include "ring.h"

#define SERVO_VISION_STRAIGHT_ERROR_PX    10
#define SERVO_VISION_STRAIGHT_HEADING_PX  6
#define SERVO_VISION_STRAIGHT_DUTY_STEP   8u
#define SERVO_VISION_TURN_DUTY_STEP       24u

static volatile servo_mode_enum servo_mode = SERVO_MODE_VISION;
static volatile int16 servo_path_bias_target = 0;
static volatile int16 servo_path_bias_applied = 0;
static uint16 servo_duty_applied = SERVO_DUTY_MID;

static int16 Servo_Abs16(int16 value)
{
    return (value < 0) ? (int16)(-value) : value;
}

/* Limit only normal vision steering; ring and avoidance steering stay unrestricted. */
static uint16 Servo_Limit_Vision_Duty(uint16 target_duty)
{
    uint8 ref;
    uint8 near_ref;
    uint16 step;
    int16 error;
    int16 heading;

    ref = controlReferenceLine;
    if (ref >= SEARCH_IMAGE_H) {
        ref = SEARCH_IMAGE_H - 1;
    }
    near_ref = (uint8)(ref + 20u);
    if (near_ref >= SEARCH_IMAGE_H) {
        near_ref = SEARCH_IMAGE_H - 1;
    }

    error = (int16)mid_line[ref] - Mid_Col;
    heading = (int16)mid_line[ref] - (int16)mid_line[near_ref];
    if (Servo_Abs16(error) <= SERVO_VISION_STRAIGHT_ERROR_PX &&
        Servo_Abs16(heading) <= SERVO_VISION_STRAIGHT_HEADING_PX) {
        step = SERVO_VISION_STRAIGHT_DUTY_STEP;
    } else {
        step = SERVO_VISION_TURN_DUTY_STEP;
    }

    if (target_duty > servo_duty_applied) {
        if ((uint16)(target_duty - servo_duty_applied) > step) {
            servo_duty_applied += step;
        } else {
            servo_duty_applied = target_duty;
        }
    } else if ((uint16)(servo_duty_applied - target_duty) > step) {
        servo_duty_applied -= step;
    } else {
        servo_duty_applied = target_duty;
    }

    return servo_duty_applied;
}

void Servo_Reset_Controller(void)
{
    PID_Reset(&servo_pidf);
}

void Servo_Set_Mode(servo_mode_enum mode)
{
    bit interrupt_state;

    if (mode != SERVO_MODE_PATH_BIAS && mode != SERVO_MODE_CENTER) {
        mode = SERVO_MODE_VISION;
    }

    interrupt_state = EA;
    EA = 0;
    if (servo_mode != mode) {
        servo_mode = mode;
        servo_path_bias_applied = 0;
        PID_Reset(&servo_pidf);
    }
    EA = interrupt_state;
}

void Servo_Set_Path_Bias(int16 bias)
{
    bit interrupt_state;

    if (bias > (SEARCH_IMAGE_W - 1)) {
        bias = SEARCH_IMAGE_W - 1;
    } else if (bias < -(SEARCH_IMAGE_W - 1)) {
        bias = -(SEARCH_IMAGE_W - 1);
    }

    interrupt_state = EA;
    EA = 0;
    servo_path_bias_target = bias;
    EA = interrupt_state;
}

/* 初始化舵机 PWM，并打开 PWME 输出。 */
void servo_init(void)
{
    servo_duty_applied = SERVO_DUTY_MID;
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
    bit interrupt_state;
    servo_mode_enum mode;
    int16 path_bias_target;
    int16 path_bias_applied;
    int16 target_col;

    interrupt_state = EA;
    EA = 0;
    mode = servo_mode;
    path_bias_target = servo_path_bias_target;
    path_bias_applied = servo_path_bias_applied;

    if (mode == SERVO_MODE_PATH_BIAS) {
        if (path_bias_applied < path_bias_target) {
            path_bias_applied += SERVO_PATH_BIAS_STEP_PER_TICK;
            if (path_bias_applied > path_bias_target) {
                path_bias_applied = path_bias_target;
            }
        } else if (path_bias_applied > path_bias_target) {
            path_bias_applied -= SERVO_PATH_BIAS_STEP_PER_TICK;
            if (path_bias_applied < path_bias_target) {
                path_bias_applied = path_bias_target;
            }
        }
        servo_path_bias_applied = path_bias_applied;
    }
    EA = interrupt_state;

    if (mode == SERVO_MODE_CENTER) {
        Out_servo = SERVO_DUTY_MID;
    } else if (mode == SERVO_MODE_PATH_BIAS) {
        target_col = (int16)Mid_Col + path_bias_applied;
        PID_servof_Target(&servo_pidf, target_col);
    } else {
        PID_servof(&servo_pidf);
    }

    if (mode == SERVO_MODE_VISION && current_step == 0u) {
        Out_servo = Servo_Limit_Vision_Duty(Out_servo);
    } else {
        servo_duty_applied = Out_servo;
    }
    Set_Servo_Duty((uint32)Out_servo);
}
