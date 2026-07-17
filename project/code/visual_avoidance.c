#include "visual_avoidance.h"

#include "image.h"
#include "ring.h"
#include "servo.h"
#include "motor.h"

#define VISUAL_AVOID_RING_NORM          0u
#define VISUAL_AVOID_LANE_MARGIN        8u
#define VISUAL_AVOID_CHECK_ROW_OFFSET   6u
#define VISUAL_AVOID_CHECK_ROW_MAX      110u
#define VISUAL_AVOID_MATCH_TOLERANCE    12u
#define VISUAL_AVOID_HISTORY_MASK       0x07u
#define VISUAL_AVOID_CLEAR_FRAMES       2u

static volatile uint8 visual_avoid_state = VISUAL_AVOID_STATE_FOLLOW;
static volatile uint16 visual_avoid_travel = 0u;
static volatile uint8 visual_avoid_recenter_ticks = 0u;
static volatile uint8 visual_avoid_recenter_ready = 0u;
static volatile int8 visual_avoid_bias = 0;
static volatile uint8 visual_avoid_resume_cap = 0u;

static uint8 visual_avoid_hit_history = 0u;
static uint8 visual_avoid_last_hit_age = 3u;
static uint8 visual_avoid_last_center = 0u;
static uint8 visual_avoid_last_bottom = 0u;
static volatile uint8 visual_avoid_clear_frames = 0u;

static uint8 visual_avoid_debug_valid = 0u;
static uint8 visual_avoid_debug_left = 0u;
static uint8 visual_avoid_debug_right = 0u;
static uint8 visual_avoid_debug_bottom = 0u;

static uint8 VisualAvoid_AbsDiffU8(uint8 a, uint8 b)
{
    return (a >= b) ? (uint8)(a - b) : (uint8)(b - a);
}

static uint8 VisualAvoid_CountHits(uint8 history)
{
    uint8 count;

    count = 0u;
    if (history & 0x01u) count++;
    if (history & 0x02u) count++;
    if (history & 0x04u) count++;
    return count;
}

static uint16 VisualAvoid_AbsEncoder(int16 value)
{
    if (value < 0) {
        /* Unsigned subtraction also handles the -32768 endpoint. */
        return (uint16)(0u - (uint16)value);
    }
    return (uint16)value;
}

static uint8 VisualAvoid_RawCandidateValid(const visual_avoid_candidate_t *candidate)
{
    if (candidate == NULL || !candidate->valid) {
        return 0u;
    }
    if (candidate->left >= candidate->right ||
        candidate->right >= SEARCH_IMAGE_W ||
        candidate->bottom_row >= SEARCH_IMAGE_H) {
        return 0u;
    }
    return 1u;
}

/* Lane edges improve side selection, but never veto an obstacle trigger. */
static uint8 VisualAvoid_GetLaneGaps(const visual_avoid_candidate_t *candidate,
                                     uint16 *left_gap,
                                     uint16 *right_gap)
{
    uint8 check_row;
    int16 lane_left;
    int16 lane_right;

    if (!VisualAvoid_RawCandidateValid(candidate)) {
        return 0u;
    }

    check_row = (uint8)(candidate->bottom_row + VISUAL_AVOID_CHECK_ROW_OFFSET);
    if (check_row > VISUAL_AVOID_CHECK_ROW_MAX) {
        check_row = VISUAL_AVOID_CHECK_ROW_MAX;
    }

    /* Search_line() owns edges only above its lost-row markers. */
    if (check_row <= lost_left || check_row <= lost_right) {
        return 0u;
    }
    /* The Search_line() defaults mean that no real lane edge was found. */
    if (left_edge_line[check_row] == 0u ||
        right_edge_line[check_row] == SEARCH_IMAGE_W - 1u) {
        return 0u;
    }

    lane_left = (int16)left_edge_line[check_row] + VISUAL_AVOID_LANE_MARGIN;
    lane_right = (int16)right_edge_line[check_row] - VISUAL_AVOID_LANE_MARGIN;
    if (lane_left >= lane_right ||
        (int16)candidate->left < lane_left ||
        (int16)candidate->right > lane_right) {
        return 0u;
    }

    if (left_gap != NULL) {
        *left_gap = (uint16)((int16)candidate->left - lane_left);
    }
    if (right_gap != NULL) {
        *right_gap = (uint16)(lane_right - (int16)candidate->right);
    }
    return 1u;
}

static void VisualAvoid_StartBypass(int8 bias)
{
    bit interrupt_state;

    /* Publish BYPASS only after steering and speed commands are installed. */
    interrupt_state = EA;
    EA = 0;
    Servo_Set_Path_Bias((int16)bias);
    Servo_Set_Mode(SERVO_MODE_PATH_BIAS);
    Motor_Set_Safety_Command(MOTOR_SAFETY_CAP, VISUAL_AVOID_SPEED_CAP);

    visual_avoid_travel = 0u;
    visual_avoid_recenter_ticks = 0u;
    visual_avoid_recenter_ready = 0u;
    visual_avoid_bias = bias;
    visual_avoid_resume_cap = 0u;
    visual_avoid_clear_frames = 0u;
    visual_avoid_hit_history = 0u;
    visual_avoid_last_hit_age = 3u;
    visual_avoid_state = VISUAL_AVOID_STATE_BYPASS;
    EA = interrupt_state;
}

void VisualAvoid_Init(void)
{
    bit interrupt_state;

    interrupt_state = EA;
    EA = 0;
    Servo_Set_Path_Bias(0);
    Servo_Set_Mode(SERVO_MODE_VISION);
    Motor_Set_Safety_Command(MOTOR_SAFETY_NORMAL, 0);

    visual_avoid_state = VISUAL_AVOID_STATE_FOLLOW;
    visual_avoid_travel = 0u;
    visual_avoid_recenter_ticks = 0u;
    visual_avoid_recenter_ready = 0u;
    visual_avoid_bias = 0;
    visual_avoid_resume_cap = 0u;
    visual_avoid_hit_history = 0u;
    visual_avoid_last_hit_age = 3u;
    visual_avoid_last_center = 0u;
    visual_avoid_last_bottom = 0u;
    visual_avoid_clear_frames = 0u;
    visual_avoid_debug_valid = 0u;
    visual_avoid_debug_left = 0u;
    visual_avoid_debug_right = 0u;
    visual_avoid_debug_bottom = 0u;
    EA = interrupt_state;
}

void VisualAvoid_OnFrame(const visual_avoid_candidate_t *candidate)
{
    uint8 state;
    uint8 valid;
    uint8 lane_gaps_valid;
    uint8 center;
    uint16 left_gap;
    uint16 right_gap;
    int8 bypass_bias;

    center = 0u;
    left_gap = 0u;
    right_gap = 0u;
    valid = VisualAvoid_RawCandidateValid(candidate);
    visual_avoid_debug_valid = valid;
    if (valid) {
        visual_avoid_debug_left = candidate->left;
        visual_avoid_debug_right = candidate->right;
        visual_avoid_debug_bottom = candidate->bottom_row;
        center = (uint8)(((uint16)candidate->left + candidate->right) >> 1);
    }

    state = visual_avoid_state;
    if (state == VISUAL_AVOID_STATE_RECENTER) {
        if (valid) {
            visual_avoid_clear_frames = 0u;
        } else if (visual_avoid_clear_frames < VISUAL_AVOID_CLEAR_FRAMES) {
            visual_avoid_clear_frames++;
        }

        if (visual_avoid_recenter_ready &&
            visual_avoid_clear_frames >= VISUAL_AVOID_CLEAR_FRAMES) {
            visual_avoid_hit_history = 0u;
            visual_avoid_last_hit_age = 3u;
            visual_avoid_state = VISUAL_AVOID_STATE_FOLLOW;
        }
        return;
    }

    if (state == VISUAL_AVOID_STATE_BYPASS) {
        return;
    }

    /* A far candidate remains visible in debug, but cannot enter confirmation. */
    if (valid && candidate->bottom_row < VISUAL_AVOID_TRIGGER_ROW) {
        valid = 0u;
    }

    /* New candidates are accepted only while the ring state is NORM. */
    if (current_step != VISUAL_AVOID_RING_NORM) {
        visual_avoid_hit_history = 0u;
        visual_avoid_last_hit_age = 3u;
        return;
    }

    if (!valid) {
        visual_avoid_hit_history =
            (uint8)((visual_avoid_hit_history << 1) & VISUAL_AVOID_HISTORY_MASK);
        if (visual_avoid_last_hit_age < 3u) {
            visual_avoid_last_hit_age++;
        }
        return;
    }

    if (visual_avoid_hit_history != 0u &&
        visual_avoid_last_hit_age < 3u &&
        (VisualAvoid_AbsDiffU8(center, visual_avoid_last_center) >
             VISUAL_AVOID_MATCH_TOLERANCE ||
         VisualAvoid_AbsDiffU8(candidate->bottom_row, visual_avoid_last_bottom) >
             VISUAL_AVOID_MATCH_TOLERANCE)) {
        visual_avoid_hit_history = 0u;
    }

    visual_avoid_hit_history =
        (uint8)(((visual_avoid_hit_history << 1) | 0x01u) &
                VISUAL_AVOID_HISTORY_MASK);
    visual_avoid_last_center = center;
    visual_avoid_last_bottom = candidate->bottom_row;
    visual_avoid_last_hit_age = 0u;

    if (VisualAvoid_CountHits(visual_avoid_hit_history) < 2u) {
        return;
    }

    lane_gaps_valid = VisualAvoid_GetLaneGaps(candidate, &left_gap, &right_gap);
    if (lane_gaps_valid) {
        /* A negative target-column bias moves the vehicle to the right. */
        bypass_bias = (right_gap >= left_gap) ?
                      (int8)-VISUAL_AVOID_PATH_BIAS_PX :
                      (int8)VISUAL_AVOID_PATH_BIAS_PX;
    } else {
        /* With missing lane edges, simply pass on the side opposite the brick. */
        bypass_bias = (center <= Mid_Col) ?
                      (int8)-VISUAL_AVOID_PATH_BIAS_PX :
                      (int8)VISUAL_AVOID_PATH_BIAS_PX;
    }
    /* The WiFi channel-5 hard stop always has priority over avoidance. */
    if (Motor_Is_Force_Stopped()) {
        return;
    }
    VisualAvoid_StartBypass(bypass_bias);
}

void VisualAvoid_ControlTick(int16 encoder_l, int16 encoder_r)
{
    uint16 encoder_l_abs;
    uint16 encoder_r_abs;
    uint16 delta;

    if (visual_avoid_state == VISUAL_AVOID_STATE_FOLLOW) {
        return;
    }

    if (Motor_Is_Force_Stopped()) {
        if (visual_avoid_state == VISUAL_AVOID_STATE_BYPASS ||
            (visual_avoid_state == VISUAL_AVOID_STATE_RECENTER &&
             !visual_avoid_recenter_ready)) {
            visual_avoid_resume_cap = 1u;
        }
        return;
    }

    if (visual_avoid_resume_cap) {
        if (visual_avoid_state == VISUAL_AVOID_STATE_BYPASS ||
            (visual_avoid_state == VISUAL_AVOID_STATE_RECENTER &&
             !visual_avoid_recenter_ready)) {
            Motor_Set_Safety_Command(MOTOR_SAFETY_CAP, VISUAL_AVOID_SPEED_CAP);
        }
        visual_avoid_resume_cap = 0u;
    }

    if (visual_avoid_state == VISUAL_AVOID_STATE_BYPASS) {
        encoder_l_abs = VisualAvoid_AbsEncoder(encoder_l);
        encoder_r_abs = VisualAvoid_AbsEncoder(encoder_r);
        /* Split before addition so the uint16 sum cannot overflow. */
        delta = (uint16)((encoder_l_abs >> 1) +
                         (encoder_r_abs >> 1) +
                         (((encoder_l_abs & 1u) + (encoder_r_abs & 1u)) >> 1));
        if (delta >= (uint16)(VISUAL_AVOID_PASS_PULSES - visual_avoid_travel)) {
            visual_avoid_travel = VISUAL_AVOID_PASS_PULSES;
        } else {
            visual_avoid_travel = (uint16)(visual_avoid_travel + delta);
        }

        if (visual_avoid_travel >= VISUAL_AVOID_PASS_PULSES) {
            Servo_Set_Path_Bias(0);
            visual_avoid_bias = 0;
            visual_avoid_recenter_ticks = 0u;
            visual_avoid_recenter_ready = 0u;
            visual_avoid_clear_frames = 0u;
            visual_avoid_state = VISUAL_AVOID_STATE_RECENTER;
        }
        return;
    }

    if (visual_avoid_state != VISUAL_AVOID_STATE_RECENTER ||
        visual_avoid_recenter_ready) {
        return;
    }

    if (visual_avoid_recenter_ticks < VISUAL_AVOID_RECENTER_TICKS) {
        visual_avoid_recenter_ticks++;
    }
    if (visual_avoid_recenter_ticks >= VISUAL_AVOID_RECENTER_TICKS) {
        /* The 3 px/tick bias slew has now returned +/-30 to zero. */
        Servo_Set_Mode(SERVO_MODE_VISION);
        Motor_Set_Safety_Command(MOTOR_SAFETY_NORMAL, 0);
        visual_avoid_recenter_ready = 1u;
    }
}

uint8 VisualAvoid_InhibitRing(void)
{
    if (visual_avoid_state != VISUAL_AVOID_STATE_FOLLOW) {
        return 1u;
    }
    /* A raw shape must never abort an actual ring already in progress. */
    if (current_step != VISUAL_AVOID_RING_NORM) {
        return 0u;
    }
    return (visual_avoid_hit_history != 0u) ? 1u : 0u;
}

void VisualAvoid_GetDebug(uint8 *candidate_valid,
                          uint8 *left,
                          uint8 *right,
                          uint8 *bottom,
                          uint8 *state,
                          uint8 *pending_hit,
                          int8 *bias,
                          uint16 *travel)
{
    bit interrupt_state;
    uint8 valid_value;
    uint8 left_value;
    uint8 right_value;
    uint8 bottom_value;
    uint8 state_value;
    uint8 pending_hit_value;
    int8 bias_value;
    uint16 travel_value;

    interrupt_state = EA;
    EA = 0;
    valid_value = visual_avoid_debug_valid;
    left_value = visual_avoid_debug_left;
    right_value = visual_avoid_debug_right;
    bottom_value = visual_avoid_debug_bottom;
    state_value = visual_avoid_state;
    pending_hit_value = (visual_avoid_hit_history != 0u) ? 1u : 0u;
    bias_value = visual_avoid_bias;
    travel_value = visual_avoid_travel;
    EA = interrupt_state;

    if (candidate_valid != NULL) *candidate_valid = valid_value;
    if (left != NULL) *left = left_value;
    if (right != NULL) *right = right_value;
    if (bottom != NULL) *bottom = bottom_value;
    if (state != NULL) *state = state_value;
    if (pending_hit != NULL) *pending_hit = pending_hit_value;
    if (bias != NULL) *bias = bias_value;
    if (travel != NULL) *travel = travel_value;
}
