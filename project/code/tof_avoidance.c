#include "tof_avoidance.h"

#include "zf_device_dl1b.h"
#include "image.h"
#include "motor.h"
#include "servo.h"

#define TOF_AVOID_INVALID_DISTANCE_MM     8192u

static uint8 dl1b_ready = 0u;
static uint8 poll_frame_count = 0u;
static uint8 communication_error_count = 0u;
static uint8 no_data_poll_count = 0u;
static uint8 enter_sample_count = 0u;
static uint8 exit_sample_count = 0u;
static int16 selected_bias = -TOF_AVOID_PATH_BIAS_PX;

static volatile uint8 tof_state = TOF_AVOID_STATE_FOLLOW;
static uint8 tof_last_valid = 0u;
static uint16 tof_last_distance_mm = TOF_AVOID_INVALID_DISTANCE_MM;

static void TofAvoid_RecoverSensor(void)
{
    uint8 avoiding;

    communication_error_count = 0u;
    no_data_poll_count = 0u;
    enter_sample_count = 0u;
    exit_sample_count = 0u;
    tof_last_valid = 0u;
    tof_last_distance_mm = TOF_AVOID_INVALID_DISTANCE_MM;

    /* XSHUT reset and configuration run only in the main loop. */
    avoiding = (tof_state == TOF_AVOID_STATE_AVOID) ? 1u : 0u;
    if (!Motor_Is_Force_Stopped()) {
        Motor_Set_Safety_Command(MOTOR_SAFETY_CAP, TOF_AVOID_SPEED_CAP);
    }
    (void)dl1b_init();
    if (!avoiding && !Motor_Is_Force_Stopped()) {
        Motor_Set_Safety_Command(MOTOR_SAFETY_NORMAL, 0);
    }
    poll_frame_count = TOF_AVOID_POLL_FRAMES - 1u;
}

static void TofAvoid_Start(void)
{
    enter_sample_count = 0u;
    exit_sample_count = 0u;

    Servo_Set_Path_Bias(selected_bias);
    Servo_Set_Mode(SERVO_MODE_PATH_BIAS);
    if (!Motor_Is_Force_Stopped()) {
        Motor_Set_Safety_Command(MOTOR_SAFETY_CAP, TOF_AVOID_SPEED_CAP);
    }

    tof_state = TOF_AVOID_STATE_AVOID;
}

static void TofAvoid_Finish(void)
{
    enter_sample_count = 0u;
    exit_sample_count = 0u;

    Servo_Set_Path_Bias(0);
    Servo_Set_Mode(SERVO_MODE_PATH_BIAS);
    if (!Motor_Is_Force_Stopped()) {
        Motor_Set_Safety_Command(MOTOR_SAFETY_NORMAL, 0);
    }

    tof_state = TOF_AVOID_STATE_FOLLOW;
}

uint8 TofAvoid_Init(void)
{
    dl1b_ready = 0u;
    poll_frame_count = TOF_AVOID_POLL_FRAMES - 1u;
    communication_error_count = 0u;
    no_data_poll_count = 0u;
    enter_sample_count = 0u;
    exit_sample_count = 0u;
    selected_bias = -TOF_AVOID_PATH_BIAS_PX;
    tof_state = TOF_AVOID_STATE_FOLLOW;
    tof_last_valid = 0u;
    tof_last_distance_mm = TOF_AVOID_INVALID_DISTANCE_MM;

    if (dl1b_init()) {
        return 1u;
    }

    dl1b_ready = 1u;
    return 0u;
}

void TofAvoid_OnFrame(void)
{
    uint8 left;
    uint8 right;
    uint8 lane_center;

    if (!dl1b_ready || tof_state != TOF_AVOID_STATE_FOLLOW) {
        return;
    }

    /* Unknown or centered cases keep the fixed right-bypass fallback. */
    selected_bias = -TOF_AVOID_PATH_BIAS_PX;

    if (TOF_AVOID_VISION_ROW <= lost_left ||
        TOF_AVOID_VISION_ROW <= lost_right) {
        return;
    }

    left = left_edge_line[TOF_AVOID_VISION_ROW];
    right = right_edge_line[TOF_AVOID_VISION_ROW];
    if (!left || right == SEARCH_IMAGE_W - 1u || left >= right) {
        return;
    }

    lane_center = (uint8)(((uint16)left + (uint16)right) >> 1);
    if (lane_center <= Mid_Col - TOF_AVOID_VISION_CENTER_DELTA_PX) {
        /* The open passage is on the left, so bypass on the left. */
        selected_bias = TOF_AVOID_PATH_BIAS_PX;
    }
}

void TofAvoid_ServiceMain(void)
{
    uint16 distance_mm;

    if (!dl1b_ready) {
        return;
    }

    poll_frame_count++;
    if (poll_frame_count < TOF_AVOID_POLL_FRAMES) {
        return;
    }
    poll_frame_count = 0u;

    dl1b_get_distance();

    if (!dl1b_communication_ok) {
        no_data_poll_count = 0u;
        if (communication_error_count < TOF_AVOID_COMM_ERROR_LIMIT) {
            communication_error_count++;
        }
        tof_last_valid = 0u;
        tof_last_distance_mm = TOF_AVOID_INVALID_DISTANCE_MM;
        if (communication_error_count >= TOF_AVOID_COMM_ERROR_LIMIT) {
            TofAvoid_RecoverSensor();
        }
        return;
    }
    communication_error_count = 0u;

    if (!dl1b_data_ready_flag) {
        if (no_data_poll_count < TOF_AVOID_NO_DATA_LIMIT) {
            no_data_poll_count++;
        }
        if (no_data_poll_count >= TOF_AVOID_NO_DATA_LIMIT) {
            TofAvoid_RecoverSensor();
        }
        return;
    }
    no_data_poll_count = 0u;

    if (!dl1b_finsh_flag || dl1b_distance_mm > 4000u) {
        /* Completed but invalid range: it was acknowledged, so keep polling. */
        tof_last_valid = 0u;
        tof_last_distance_mm = TOF_AVOID_INVALID_DISTANCE_MM;
        return;
    }

    distance_mm = dl1b_distance_mm;
    tof_last_valid = 1u;
    tof_last_distance_mm = distance_mm;

    if (distance_mm >= TOF_AVOID_TRIGGER_MIN_MM &&
        distance_mm <= TOF_AVOID_TRIGGER_MAX_MM) {
        if (tof_state == TOF_AVOID_STATE_FOLLOW) {
            exit_sample_count = 0u;
            if (enter_sample_count < TOF_AVOID_ENTER_SAMPLES) {
                enter_sample_count++;
            }
            if (enter_sample_count >= TOF_AVOID_ENTER_SAMPLES) {
                TofAvoid_Start();
            }
        } else {
            enter_sample_count = 0u;
            exit_sample_count = 0u;
        }
        return;
    }

    if (tof_state == TOF_AVOID_STATE_AVOID) {
        enter_sample_count = 0u;
        if (exit_sample_count < TOF_AVOID_EXIT_SAMPLES) {
            exit_sample_count++;
        }
        if (exit_sample_count >= TOF_AVOID_EXIT_SAMPLES) {
            TofAvoid_Finish();
        }
    } else {
        enter_sample_count = 0u;
        exit_sample_count = 0u;
    }
}

uint8 TofAvoid_InhibitRing(void)
{
    return (tof_state == TOF_AVOID_STATE_AVOID) ? 1u : 0u;
}

void TofAvoid_GetDebug(uint8 *valid, uint16 *distance_mm, uint8 *state)
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
}
