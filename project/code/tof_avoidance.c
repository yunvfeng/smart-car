#include "tof_avoidance.h"

#include "zf_device_dl1b.h"
#include "motor.h"
#include "servo.h"

#define TOF_AVOID_INVALID_DISTANCE_MM         8192u

/* 只有轮询请求和状态会跨主循环/中断访问，均为 8 位。 */
static volatile uint8 tof_poll_due = 0;
static volatile uint8 tof_state = TOF_AVOID_STATE_FOLLOW;

static uint8 dl1b_ready = 0;
static uint8 poll_tick_count = 0;
static uint8 near_sample_count = 0;
static uint8 clear_sample_count = 0;
static uint8 invalid_sample_count = 0;
static uint8 rearm_requested = 0;
static uint8 tof_last_valid = 0;
static uint16 tof_last_distance_mm = TOF_AVOID_INVALID_DISTANCE_MM;

static void TofAvoid_SetState(tof_avoid_state_enum next_state)
{
    bit interrupt_state;

    interrupt_state = EA;
    EA = 0;

    near_sample_count = 0;
    clear_sample_count = 0;
    invalid_sample_count = 0;
    rearm_requested = 0;

    if (next_state == TOF_AVOID_STATE_AVOID) {
        Servo_Set_Path_Bias(TOF_AVOID_RIGHT_BIAS_PX);
        Servo_Set_Mode(SERVO_MODE_PATH_BIAS);
        Motor_Set_Safety_Command(MOTOR_SAFETY_CAP, TOF_AVOID_SPEED_CAP);
    } else if (next_state == TOF_AVOID_STATE_STOPPED) {
        /* Hard-stop first; publish STOPPED only after the outputs are safe. */
        Motor_Set_Safety_Command(MOTOR_SAFETY_STOP, 0);
        Servo_Set_Path_Bias(0);
        Servo_Set_Mode(SERVO_MODE_CENTER);
    } else {
        /* PATH_BIAS=0 与普通循迹目标相同，并可平滑撤销右偏。 */
        Servo_Set_Path_Bias(0);
        Servo_Set_Mode(SERVO_MODE_PATH_BIAS);
        Motor_Set_Safety_Command(MOTOR_SAFETY_NORMAL, 0);
    }

    /* ISR feature gating observes the new state only after commands are ready. */
    tof_state = (uint8)next_state;
    EA = interrupt_state;
}

static void TofAvoid_ProcessSample(uint8 valid, uint16 distance_mm)
{
    tof_last_valid = valid;
    tof_last_distance_mm = valid ? distance_mm : TOF_AVOID_INVALID_DISTANCE_MM;

    if (!valid) {
        /* 数据可能只是尚未 ready；连续约 150 ms 无效才按失效处理。 */
        if (invalid_sample_count < TOF_AVOID_INVALID_LIMIT) {
            invalid_sample_count++;
        }
        if (invalid_sample_count >= TOF_AVOID_INVALID_LIMIT) {
            near_sample_count = 0;
            clear_sample_count = 0;
            if (tof_state == TOF_AVOID_STATE_AVOID) {
                TofAvoid_SetState(TOF_AVOID_STATE_FOLLOW);
            }
        }
        return;
    }

    invalid_sample_count = 0;

    if (distance_mm <= TOF_AVOID_EMERGENCY_MM) {
        if (tof_state != TOF_AVOID_STATE_STOPPED) {
            TofAvoid_SetState(TOF_AVOID_STATE_STOPPED);
        }
        return;
    }

    if (tof_state == TOF_AVOID_STATE_STOPPED) {
        if (rearm_requested && distance_mm > TOF_AVOID_CLEAR_MM) {
            TofAvoid_SetState(TOF_AVOID_STATE_FOLLOW);
        }
        return;
    }

    if (tof_state == TOF_AVOID_STATE_FOLLOW) {
        if (distance_mm < TOF_AVOID_TRIGGER_MM) {
            if (near_sample_count < TOF_AVOID_TRIGGER_SAMPLES) {
                near_sample_count++;
            }
        } else {
            near_sample_count = 0;
        }

        if (near_sample_count >= TOF_AVOID_TRIGGER_SAMPLES) {
            TofAvoid_SetState(TOF_AVOID_STATE_AVOID);
        }
        return;
    }

    /* AVOID：前方扫空后继续保持右偏约 20 x 30 ms，再交回摄像头。 */
    if (distance_mm > TOF_AVOID_CLEAR_MM) {
        if (clear_sample_count < TOF_AVOID_PASS_CLEAR_SAMPLES) {
            clear_sample_count++;
        }
    } else {
        clear_sample_count = 0;
    }

    if (clear_sample_count >= TOF_AVOID_PASS_CLEAR_SAMPLES) {
        TofAvoid_SetState(TOF_AVOID_STATE_FOLLOW);
    }
}

uint8 TofAvoid_Init(void)
{
    uint8 init_error;

    tof_poll_due = 0;
    poll_tick_count = 0;
    near_sample_count = 0;
    clear_sample_count = 0;
    invalid_sample_count = 0;
    rearm_requested = 0;
    tof_last_valid = 0;
    tof_last_distance_mm = TOF_AVOID_INVALID_DISTANCE_MM;
    dl1b_ready = 0;

    TofAvoid_SetState(TOF_AVOID_STATE_FOLLOW);

    init_error = dl1b_init();
    if (!init_error) {
        dl1b_ready = 1;
        tof_poll_due = 1;
    }
    return init_error;
}

void TofAvoid_ServiceMain(void)
{
    uint8 should_poll;
    uint8 valid;
    uint16 distance_mm;
    bit interrupt_state;

    if (!dl1b_ready) {
        return;
    }

    should_poll = 0;
    interrupt_state = EA;
    EA = 0;
    if (tof_poll_due) {
        tof_poll_due = 0;
        should_poll = 1;
    }
    EA = interrupt_state;

    if (!should_poll) {
        return;
    }

    valid = 0;
    distance_mm = TOF_AVOID_INVALID_DISTANCE_MM;
    dl1b_finsh_flag = 0;
    dl1b_get_distance();
    if (dl1b_finsh_flag && dl1b_distance_mm <= 4000u) {
        valid = 1;
        distance_mm = dl1b_distance_mm;
    }

    TofAvoid_ProcessSample(valid, distance_mm);
}

void TofAvoid_ControlTick(void)
{
    poll_tick_count++;
    if (poll_tick_count >= TOF_AVOID_POLL_TICKS) {
        poll_tick_count = 0;
        tof_poll_due = 1;
    }
}

void TofAvoid_RequestRearm(void)
{
    if (tof_state == TOF_AVOID_STATE_STOPPED) {
        rearm_requested = 1;
    }
}

uint8 TofAvoid_InhibitFeatures(void)
{
    return (tof_state != TOF_AVOID_STATE_FOLLOW) ? 1u : 0u;
}

void TofAvoid_GetDebug(uint8 *valid,
                       uint16 *distance_mm,
                       uint8 *state,
                       uint8 *clear_samples)
{
    if (valid != NULL) {
        *valid = tof_last_valid;
    }
    if (distance_mm != NULL) {
        *distance_mm = tof_last_distance_mm;
    }
    if (state != NULL) {
        *state = tof_state;
    }
    if (clear_samples != NULL) {
        *clear_samples = clear_sample_count;
    }
}
