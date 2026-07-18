#include "tof_avoidance.h"

#include "zf_device_dl1b.h"
#include "image.h"
#include "motor.h"
#include "ring.h"
#include "servo.h"

#define TOF_AVOID_INVALID_DISTANCE_MM         8192u
#define TOF_AVOID_VISION_ROW_1                83u
#define TOF_AVOID_VISION_ROW_2                87u
#define TOF_AVOID_VISION_ROW_3                89u
#define TOF_AVOID_VISION_CENTER_DELTA_PX      6u

static volatile uint8 tof_poll_due = 0u;
static volatile uint8 tof_state = TOF_AVOID_STATE_FOLLOW;

static uint8 dl1b_ready = 0u;
static uint8 poll_tick_count = 0u;
static uint8 near_sample_count = 0u;
static uint8 clear_sample_count = 0u;
static uint8 invalid_sample_count = 0u;
static uint8 tof_last_valid = 0u;
static uint16 tof_last_distance_mm = TOF_AVOID_INVALID_DISTANCE_MM;
static uint8 tof_obstacle_confirmed = 0u;
static uint8 tof_vision_obstacle_side = TOF_AVOID_OBSTACLE_UNKNOWN;
static uint8 tof_selected_obstacle_side = TOF_AVOID_OBSTACLE_UNKNOWN;

static void TofAvoid_ResetVisionSide(void)
{
    tof_vision_obstacle_side = TOF_AVOID_OBSTACLE_UNKNOWN;
}

static uint8 TofAvoid_VisionRowVote(uint8 row)
{
    uint8 left;
    uint8 right;
    uint8 open_mid;

    if (row >= SEARCH_IMAGE_H || row <= lost_left || row <= lost_right) {
        return TOF_AVOID_OBSTACLE_UNKNOWN;
    }

    left = left_edge_line[row];
    right = right_edge_line[row];
    if (!left || right == SEARCH_IMAGE_W - 1u || left >= right) {
        return TOF_AVOID_OBSTACLE_UNKNOWN;
    }

    open_mid = (uint8)(((uint16)left + (uint16)right) >> 1);
    if (open_mid >= Mid_Col + TOF_AVOID_VISION_CENTER_DELTA_PX) {
        /* The open passage moved right, so the obstacle is on the left. */
        return TOF_AVOID_OBSTACLE_LEFT;
    }
    if (open_mid <= Mid_Col - TOF_AVOID_VISION_CENTER_DELTA_PX) {
        /* The open passage moved left, so the obstacle is on the right. */
        return TOF_AVOID_OBSTACLE_RIGHT;
    }
    return TOF_AVOID_OBSTACLE_UNKNOWN;
}

static uint8 TofAvoid_DetectVisionSide(void)
{
    uint8 vote;
    uint8 left_votes;
    uint8 right_votes;

    left_votes = 0u;
    right_votes = 0u;

    vote = TofAvoid_VisionRowVote(TOF_AVOID_VISION_ROW_1);
    if (vote == TOF_AVOID_OBSTACLE_LEFT) left_votes++;
    else if (vote == TOF_AVOID_OBSTACLE_RIGHT) right_votes++;

    vote = TofAvoid_VisionRowVote(TOF_AVOID_VISION_ROW_2);
    if (vote == TOF_AVOID_OBSTACLE_LEFT) left_votes++;
    else if (vote == TOF_AVOID_OBSTACLE_RIGHT) right_votes++;

    vote = TofAvoid_VisionRowVote(TOF_AVOID_VISION_ROW_3);
    if (vote == TOF_AVOID_OBSTACLE_LEFT) left_votes++;
    else if (vote == TOF_AVOID_OBSTACLE_RIGHT) right_votes++;

    if (left_votes >= 2u) return TOF_AVOID_OBSTACLE_LEFT;
    if (right_votes >= 2u) return TOF_AVOID_OBSTACLE_RIGHT;
    return TOF_AVOID_OBSTACLE_UNKNOWN;
}

static void TofAvoid_SetState(tof_avoid_state_enum next_state)
{
    bit interrupt_state;

    interrupt_state = EA;
    EA = 0;

    near_sample_count = 0u;
    clear_sample_count = 0u;
    invalid_sample_count = 0u;
    tof_obstacle_confirmed = 0u;

    if (next_state == TOF_AVOID_STATE_AVOID) {
        tof_selected_obstacle_side = tof_vision_obstacle_side;
        if (tof_selected_obstacle_side == TOF_AVOID_OBSTACLE_RIGHT) {
            /* Obstacle on the right: use the left passage. */
            Servo_Set_Path_Bias(TOF_AVOID_PATH_BIAS_PX);
        } else {
            /* Obstacle on the left or unknown: keep the existing right fallback. */
            Servo_Set_Path_Bias(-TOF_AVOID_PATH_BIAS_PX);
        }
        Servo_Set_Mode(SERVO_MODE_PATH_BIAS);
        if (!Motor_Is_Force_Stopped()) {
            Motor_Set_Safety_Command(MOTOR_SAFETY_CAP, TOF_AVOID_SPEED_CAP);
        }
    } else {
        /* Keeping PATH_BIAS active lets the existing slew limiter recenter smoothly. */
        Servo_Set_Path_Bias(0);
        Servo_Set_Mode(SERVO_MODE_PATH_BIAS);
        if (!Motor_Is_Force_Stopped()) {
            Motor_Set_Safety_Command(MOTOR_SAFETY_NORMAL, 0);
        }
        tof_selected_obstacle_side = TOF_AVOID_OBSTACLE_UNKNOWN;
        TofAvoid_ResetVisionSide();
    }

    tof_state = (uint8)next_state;
    EA = interrupt_state;
}

static void TofAvoid_ProcessSample(uint8 valid, uint16 distance_mm)
{
    tof_last_valid = valid;
    tof_last_distance_mm = valid ? distance_mm : TOF_AVOID_INVALID_DISTANCE_MM;

    if (!valid) {
        if (invalid_sample_count < TOF_AVOID_INVALID_LIMIT) {
            invalid_sample_count++;
        }
        if (invalid_sample_count >= TOF_AVOID_INVALID_LIMIT) {
            near_sample_count = 0u;
            clear_sample_count = 0u;
            tof_obstacle_confirmed = 0u;
            TofAvoid_ResetVisionSide();
            if (tof_state == TOF_AVOID_STATE_AVOID) {
                /* Sensor loss degrades to ordinary camera following; it never stops. */
                TofAvoid_SetState(TOF_AVOID_STATE_FOLLOW);
            }
        }
        return;
    }

    invalid_sample_count = 0u;

    if (tof_state == TOF_AVOID_STATE_FOLLOW) {
        /* New obstacle triggers are accepted only in the normal ring state. */
        if (current_step != 0u) {
            near_sample_count = 0u;
            tof_obstacle_confirmed = 0u;
            TofAvoid_ResetVisionSide();
            return;
        }

        if (distance_mm > TOF_AVOID_TRIGGER_MIN_MM &&
            distance_mm < TOF_AVOID_TRIGGER_MM) {
            if (!near_sample_count) {
                /* The direction must come from frames belonging to this near event. */
                tof_obstacle_confirmed = 0u;
                TofAvoid_ResetVisionSide();
            }
            if (near_sample_count < TOF_AVOID_TRIGGER_SAMPLES) {
                near_sample_count++;
            }
        } else {
            near_sample_count = 0u;
            tof_obstacle_confirmed = 0u;
            TofAvoid_ResetVisionSide();
        }

        if (near_sample_count >= TOF_AVOID_TRIGGER_SAMPLES) {
            if (tof_vision_obstacle_side != TOF_AVOID_OBSTACLE_UNKNOWN) {
                TofAvoid_SetState(TOF_AVOID_STATE_AVOID);
            } else {
                /* Wait for one completed camera frame, never for a blocking loop. */
                tof_obstacle_confirmed = 1u;
            }
        }
        return;
    }

    if (distance_mm < TOF_AVOID_CLEAR_MM) {
        if (clear_sample_count < TOF_AVOID_PASS_CLEAR_SAMPLES) {
            clear_sample_count++;
        }
    } else {
        clear_sample_count = 0u;
    }

    if (clear_sample_count >= TOF_AVOID_PASS_CLEAR_SAMPLES) {
        TofAvoid_SetState(TOF_AVOID_STATE_FOLLOW);
    }
}

uint8 TofAvoid_Init(void)
{
    uint8 init_error;

    tof_poll_due = 0u;
    poll_tick_count = 0u;
    near_sample_count = 0u;
    clear_sample_count = 0u;
    invalid_sample_count = 0u;
    tof_last_valid = 0u;
    tof_last_distance_mm = TOF_AVOID_INVALID_DISTANCE_MM;
    dl1b_ready = 0u;

    TofAvoid_SetState(TOF_AVOID_STATE_FOLLOW);

    init_error = dl1b_init();
    if (!init_error) {
        dl1b_ready = 1u;
        tof_poll_due = 1u;
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

    should_poll = 0u;
    interrupt_state = EA;
    EA = 0;
    if (tof_poll_due) {
        tof_poll_due = 0u;
        should_poll = 1u;
    }
    EA = interrupt_state;

    if (!should_poll) {
        return;
    }

    valid = 0u;
    distance_mm = TOF_AVOID_INVALID_DISTANCE_MM;
    dl1b_finsh_flag = 0u;
    dl1b_get_distance();
    if (dl1b_finsh_flag && dl1b_distance_mm <= 4000u) {
        valid = 1u;
        distance_mm = dl1b_distance_mm;
    }

    TofAvoid_ProcessSample(valid, distance_mm);
}

void TofAvoid_OnFrame(void)
{
    uint8 detected_side;

    if (!dl1b_ready || tof_state != TOF_AVOID_STATE_FOLLOW) {
        return;
    }

    /* Do not retain a direction guessed before ToF sees a near obstacle. */
    if (!near_sample_count) {
        tof_obstacle_confirmed = 0u;
        TofAvoid_ResetVisionSide();
        return;
    }

    if (current_step != 0u) {
        near_sample_count = 0u;
        tof_obstacle_confirmed = 0u;
        TofAvoid_ResetVisionSide();
        return;
    }

    detected_side = TofAvoid_DetectVisionSide();
    if (detected_side != TOF_AVOID_OBSTACLE_UNKNOWN) {
        tof_vision_obstacle_side = detected_side;
    }

    if (tof_obstacle_confirmed) {
        /* UNKNOWN keeps the original right-bypass fallback after this fresh frame. */
        TofAvoid_SetState(TOF_AVOID_STATE_AVOID);
    }
}

void TofAvoid_ControlTick(void)
{
    /* A manual WiFi stop remains authoritative; releasing it restores this cap. */
    if (tof_state == TOF_AVOID_STATE_AVOID && !Motor_Is_Force_Stopped()) {
        Motor_Set_Safety_Command(MOTOR_SAFETY_CAP, TOF_AVOID_SPEED_CAP);
    }

    poll_tick_count++;
    if (poll_tick_count >= TOF_AVOID_POLL_TICKS) {
        poll_tick_count = 0u;
        tof_poll_due = 1u;
    }
}

uint8 TofAvoid_InhibitRing(void)
{
    return (tof_state == TOF_AVOID_STATE_AVOID) ? 1u : 0u;
}

void TofAvoid_GetDebug(uint8 *valid,
                       uint16 *distance_mm,
                       uint8 *state,
                       uint8 *clear_samples,
                       uint8 *obstacle_side)
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
    if (obstacle_side != NULL) {
        *obstacle_side = (tof_state == TOF_AVOID_STATE_AVOID) ?
                         tof_selected_obstacle_side :
                         tof_vision_obstacle_side;
    }
}
