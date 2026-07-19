#include "visual_avoidance.h"

#include "image.h"
#include "ring.h"
#include "servo.h"

#define VISUAL_AVOID_STATE_SCAN             0u
#define VISUAL_AVOID_STATE_ACTIVE           1u
#define VISUAL_AVOID_STATE_RECENTER         2u

#define VISUAL_AVOID_EXACT_BOTTOM_ROW       109u
#define VISUAL_AVOID_COARSE_FIRST_ROW       107u
#define VISUAL_AVOID_COARSE_STEP            4u
#define VISUAL_AVOID_COARSE_TOP_ROW         19u
#define VISUAL_AVOID_LOWER_COARSE_JUMP_PX   8
#define VISUAL_AVOID_UPPER_COARSE_JUMP_PX   4
#define VISUAL_AVOID_RIGHT_STEP_MAX         4
#define VISUAL_AVOID_VERTICAL_ROWS_MIN      6u
#define VISUAL_AVOID_VERTICAL_ROWS_MAX      40u
#define VISUAL_AVOID_LANE_WIDTH_MIN         20u

#define VISUAL_AVOID_POINT_VALID(row, left, right)                         \
    ((row) > lost_left && (row) > lost_right &&                            \
     (left) != 0u && (right) != SEARCH_IMAGE_W - 1u &&                     \
     (left) < (right) && (uint8)((right) - (left)) >= VISUAL_AVOID_LANE_WIDTH_MIN)

#define VISUAL_AVOID_MIRROR_COL(col)                                       \
    ((uint8)(SEARCH_IMAGE_W - 1u - (col)))

/* Mirror right-side points into the existing left-obstacle coordinate system. */
#define VISUAL_AVOID_LOAD_POINT(row, mirror, left, right)                  \
    do {                                                                   \
        if (mirror) {                                                      \
            (left) = VISUAL_AVOID_MIRROR_COL(right_edge_line[(row)]);      \
            (right) = VISUAL_AVOID_MIRROR_COL(left_edge_line[(row)]);      \
        } else {                                                           \
            (left) = left_edge_line[(row)];                                \
            (right) = right_edge_line[(row)];                              \
        }                                                                  \
    } while (0)

static volatile uint8 visual_avoid_state = VISUAL_AVOID_STATE_SCAN;
static volatile uint8 visual_avoid_done = 0u;
static volatile uint16 visual_avoid_travel = 0u;
static volatile uint8 visual_avoid_recenter_ticks = 0u;

static uint8 visual_avoid_detected = 0u;
static uint8 visual_avoid_upper_row = 0u;
static uint8 visual_avoid_lower_row = 0u;
static uint8 visual_avoid_lower_col = 0u;

/* Return 1 for wide-to-narrow, -1 for narrow-to-wide, otherwise 0. */
static int8 VisualAvoid_ClassifyPair(uint8 left_below,
                                    uint8 right_below,
                                    uint8 left_above,
                                    uint8 right_above)
{
    int16 left_step;
    int16 right_step;
    int16 width_below;
    int16 width_above;

    left_step = (int16)left_above - (int16)left_below;
    right_step = (int16)right_above - (int16)right_below;
    if (right_step < -VISUAL_AVOID_RIGHT_STEP_MAX ||
        right_step > VISUAL_AVOID_RIGHT_STEP_MAX) {
        return 0;
    }

    width_below = (int16)right_below - (int16)left_below;
    width_above = (int16)right_above - (int16)left_above;
    if (left_step >= VISUAL_AVOID_WIDTH_JUMP_PX &&
        width_below - width_above >= VISUAL_AVOID_WIDTH_JUMP_PX) {
        return 1;
    }
    if (left_step <= -VISUAL_AVOID_UPPER_JUMP_PX &&
        width_above - width_below >= VISUAL_AVOID_UPPER_JUMP_PX) {
        return -1;
    }
    return 0;
}

/* Refine one four-row coarse interval with its skipped raw Search_line point. */
static uint8 VisualAvoid_RefineJump(uint8 below_row,
                                    uint8 left_below,
                                    uint8 right_below,
                                    uint8 left_above,
                                    uint8 right_above,
                                    uint8 mirror,
                                    int8 wanted,
                                    uint8 *corner_row,
                                    uint8 *corner_col)
{
    uint8 middle_row;
    uint8 left_middle;
    uint8 right_middle;
    int8 jump;

    middle_row = (uint8)(below_row - PIXEL_OFFSET);
    VISUAL_AVOID_LOAD_POINT(middle_row, mirror,
                            left_middle, right_middle);
    if (!VISUAL_AVOID_POINT_VALID(middle_row, left_middle, right_middle)) {
        return 0u;
    }

    jump = VisualAvoid_ClassifyPair(left_below, right_below,
                                    left_middle, right_middle);
    if (jump == wanted) {
        if (wanted > 0) {
            *corner_row = middle_row;
            *corner_col = left_middle;
        } else {
            *corner_row = below_row;
            *corner_col = left_below;
        }
        return 1u;
    }

    jump = VisualAvoid_ClassifyPair(left_middle, right_middle,
                                    left_above, right_above);
    if (jump == wanted) {
        if (wanted > 0) {
            *corner_row = (uint8)(below_row - VISUAL_AVOID_COARSE_STEP);
            *corner_col = left_above;
        } else {
            *corner_row = middle_row;
            *corner_col = left_middle;
        }
        return 1u;
    }
    return 0u;
}

/* Check only raw points skipped by the four-row coarse scan. */
static uint8 VisualAvoid_VerticalValid(uint8 lower_row,
                                       uint8 upper_row,
                                       uint8 lower_col,
                                       uint8 upper_col,
                                       uint8 coarse_min,
                                       uint8 coarse_max,
                                       uint8 mirror)
{
    int16 row;
    int16 corner_diff;
    uint8 left;
    uint8 right;
    uint8 vertical_min;
    uint8 vertical_max;

    if (lower_row <= upper_row) {
        return 0u;
    }
    if ((uint8)(lower_row - upper_row) < VISUAL_AVOID_VERTICAL_ROWS_MIN ||
        (uint8)(lower_row - upper_row) > VISUAL_AVOID_VERTICAL_ROWS_MAX) {
        return 0u;
    }

    corner_diff = (int16)upper_col - (int16)lower_col;
    if (corner_diff <= -VISUAL_AVOID_LEFT_POINT_DIFF_PX ||
        corner_diff >= VISUAL_AVOID_LEFT_POINT_DIFF_PX) {
        return 0u;
    }

    vertical_min = coarse_min;
    vertical_max = coarse_max;
    if (upper_col < vertical_min) vertical_min = upper_col;
    if (upper_col > vertical_max) vertical_max = upper_col;
    if ((uint8)(vertical_max - vertical_min) >= VISUAL_AVOID_LEFT_POINT_DIFF_PX) {
        return 0u;
    }

    /* Coarse rows are 3 mod 4.  Scan only the interleaved 1 mod 4 rows. */
    if ((lower_row & 0x03u) == (VISUAL_AVOID_COARSE_FIRST_ROW & 0x03u)) {
        row = (int16)lower_row - PIXEL_OFFSET;
    } else {
        row = (int16)lower_row - VISUAL_AVOID_COARSE_STEP;
    }

    while (row > (int16)upper_row) {
        VISUAL_AVOID_LOAD_POINT((uint8)row, mirror, left, right);
        if (!VISUAL_AVOID_POINT_VALID((uint8)row, left, right)) {
            return 0u;
        }
        if (left < vertical_min) vertical_min = left;
        if (left > vertical_max) vertical_max = left;
        if ((uint8)(vertical_max - vertical_min) >=
            VISUAL_AVOID_LEFT_POINT_DIFF_PX) {
            return 0u;
        }
        row -= VISUAL_AVOID_COARSE_STEP;
    }
    return 1u;
}

/*
 * Coarse scan only raw odd rows.  Four-row candidates are refined back to
 * the original two-row jump rules, so normal frames load each edge row once.
 */
static uint8 VisualAvoid_Detect(uint8 mirror,
                               uint8 *upper_row_out,
                               uint8 *lower_row_out,
                               uint8 *lower_col_out)
{
    uint8 below_row;
    uint8 above_row;
    uint8 left_below;
    uint8 right_below;
    uint8 left_above;
    uint8 right_above;
    uint8 corner_row;
    uint8 corner_col;
    uint8 lower_row;
    uint8 lower_col;
    uint8 vertical_min;
    uint8 vertical_max;
    uint8 stage;
    int16 coarse_left_step;

    stage = 0u;
    lower_row = 0u;
    lower_col = 0u;
    vertical_min = 0u;
    vertical_max = 0u;

    below_row = VISUAL_AVOID_EXACT_BOTTOM_ROW;
    above_row = VISUAL_AVOID_COARSE_FIRST_ROW;
    VISUAL_AVOID_LOAD_POINT(below_row, mirror, left_below, right_below);
    VISUAL_AVOID_LOAD_POINT(above_row, mirror, left_above, right_above);

    if (VISUAL_AVOID_POINT_VALID(below_row, left_below, right_below) &&
        VISUAL_AVOID_POINT_VALID(above_row, left_above, right_above) &&
        VisualAvoid_ClassifyPair(left_below, right_below,
                                 left_above, right_above) > 0) {
        lower_row = above_row;
        lower_col = left_above;
        vertical_min = lower_col;
        vertical_max = lower_col;
        stage = 1u;
    }

    below_row = above_row;
    left_below = left_above;
    right_below = right_above;
    above_row = (uint8)(below_row - VISUAL_AVOID_COARSE_STEP);

    while (1) {
        VISUAL_AVOID_LOAD_POINT(above_row, mirror,
                                left_above, right_above);

        if (!VISUAL_AVOID_POINT_VALID(below_row, left_below, right_below) ||
            !VISUAL_AVOID_POINT_VALID(above_row, left_above, right_above)) {
            stage = 0u;
        } else {
            coarse_left_step = (int16)left_above - (int16)left_below;

            if (!stage) {
                if (coarse_left_step >= VISUAL_AVOID_LOWER_COARSE_JUMP_PX &&
                    VisualAvoid_RefineJump(below_row,
                                           left_below, right_below,
                                           left_above, right_above,
                                           mirror,
                                           1, &corner_row, &corner_col)) {
                    lower_row = corner_row;
                    lower_col = corner_col;
                    vertical_min = corner_col;
                    vertical_max = corner_col;
                    if (left_above < vertical_min) vertical_min = left_above;
                    if (left_above > vertical_max) vertical_max = left_above;
                    stage = 1u;
                }
            } else {
                if (coarse_left_step <= -VISUAL_AVOID_UPPER_COARSE_JUMP_PX &&
                    VisualAvoid_RefineJump(below_row,
                                           left_below, right_below,
                                           left_above, right_above,
                                           mirror,
                                           -1, &corner_row, &corner_col)) {
                    if (VisualAvoid_VerticalValid(lower_row, corner_row,
                                                  lower_col, corner_col,
                                                  vertical_min, vertical_max,
                                                  mirror)) {
                        *upper_row_out = corner_row;
                        *lower_row_out = lower_row;
                        *lower_col_out = lower_col;
                        return 1u;
                    }
                    stage = 0u;
                } else {
                    if (left_above < vertical_min) vertical_min = left_above;
                    if (left_above > vertical_max) vertical_max = left_above;
                    if ((uint8)(vertical_max - vertical_min) >=
                            VISUAL_AVOID_LEFT_POINT_DIFF_PX ||
                        (uint8)(lower_row - above_row) >
                            VISUAL_AVOID_VERTICAL_ROWS_MAX) {
                        stage = 0u;
                    }
                }
            }
        }

        if (above_row <= VISUAL_AVOID_COARSE_TOP_ROW) {
            break;
        }
        below_row = above_row;
        left_below = left_above;
        right_below = right_above;
        above_row = (uint8)(above_row - VISUAL_AVOID_COARSE_STEP);
    }
    return 0u;
}

static uint16 VisualAvoid_AbsEncoder(int16 value)
{
    if (value < 0) {
        return (uint16)(0u - (uint16)value);
    }
    return (uint16)value;
}

static void VisualAvoid_ResetRuntime(void)
{
    bit interrupt_state;

    interrupt_state = EA;
    EA = 0;
    visual_avoid_state = VISUAL_AVOID_STATE_SCAN;
    visual_avoid_done = 0u;
    visual_avoid_travel = 0u;
    visual_avoid_recenter_ticks = 0u;
    EA = interrupt_state;
}

void VisualAvoid_Init(void)
{
    Servo_Set_Path_Bias(0);
    Servo_Set_Mode(SERVO_MODE_VISION);
    VisualAvoid_ResetRuntime();

    visual_avoid_detected = 0u;
    visual_avoid_upper_row = 0u;
    visual_avoid_lower_row = 0u;
    visual_avoid_lower_col = 0u;
}

uint8 VisualAvoid_ProcessFrame(uint8 enable)
{
    bit interrupt_state;
    uint8 state;
    uint8 done;
    uint8 upper_row;
    uint8 lower_row;
    uint8 lower_col;
    uint8 mirror;

    state = visual_avoid_state;
    done = visual_avoid_done;

    if (done) {
        Servo_Set_Mode(SERVO_MODE_VISION);
        VisualAvoid_ResetRuntime();
        visual_avoid_detected = 0u;
        visual_avoid_upper_row = 0u;
        visual_avoid_lower_row = 0u;
        visual_avoid_lower_col = 0u;
        return VISUAL_AVOID_RESULT_DONE;
    }

    if (state != VISUAL_AVOID_STATE_SCAN) {
        return VISUAL_AVOID_RESULT_HOLD;
    }

    if (!enable) {
        return VISUAL_AVOID_RESULT_NONE;
    }

    visual_avoid_detected = 0u;
    visual_avoid_upper_row = 0u;
    visual_avoid_lower_row = 0u;
    visual_avoid_lower_col = 0u;

    if (ring_l && !ring_r) {
        mirror = 0u;
    } else if (ring_r && !ring_l) {
        mirror = 1u;
    } else {
        return VISUAL_AVOID_RESULT_NONE;
    }

    if (!VisualAvoid_Detect(mirror, &upper_row, &lower_row, &lower_col)) {
        return VISUAL_AVOID_RESULT_NONE;
    }

    if (mirror) {
        lower_col = VISUAL_AVOID_MIRROR_COL(lower_col);
    }

    visual_avoid_detected = 1u;
    visual_avoid_upper_row = upper_row;
    visual_avoid_lower_row = lower_row;
    visual_avoid_lower_col = lower_col;

    Servo_Set_Path_Bias(mirror ? VISUAL_AVOID_LEFT_BIAS_PX :
                                 VISUAL_AVOID_RIGHT_BIAS_PX);
    Servo_Set_Mode(SERVO_MODE_PATH_BIAS);

    interrupt_state = EA;
    EA = 0;
    visual_avoid_travel = 0u;
    visual_avoid_recenter_ticks = 0u;
    visual_avoid_done = 0u;
    visual_avoid_state = VISUAL_AVOID_STATE_ACTIVE;
    EA = interrupt_state;
    return VISUAL_AVOID_RESULT_HOLD;
}

void VisualAvoid_ControlTick(int16 encoder_l, int16 encoder_r)
{
    uint8 state;
    uint16 encoder_l_abs;
    uint16 encoder_r_abs;
    uint16 delta;
    uint16 travel;

    if (visual_avoid_done) {
        return;
    }

    state = visual_avoid_state;
    if (state == VISUAL_AVOID_STATE_ACTIVE) {
        encoder_l_abs = VisualAvoid_AbsEncoder(encoder_l);
        encoder_r_abs = VisualAvoid_AbsEncoder(encoder_r);
        delta = (uint16)((encoder_l_abs >> 1) +
                         (encoder_r_abs >> 1) +
                         (((encoder_l_abs & 1u) +
                           (encoder_r_abs & 1u)) >> 1));
        travel = visual_avoid_travel;

        if (delta >= (uint16)(VISUAL_AVOID_PASS_PULSES - travel)) {
            visual_avoid_travel = VISUAL_AVOID_PASS_PULSES;
            /* This control cycle's Servo_Loop is the first recenter step. */
            visual_avoid_recenter_ticks = 1u;
            Servo_Set_Path_Bias(0);
            visual_avoid_state = VISUAL_AVOID_STATE_RECENTER;
        } else {
            visual_avoid_travel = (uint16)(travel + delta);
        }
        return;
    }

    if (state == VISUAL_AVOID_STATE_RECENTER) {
        if (visual_avoid_recenter_ticks < VISUAL_AVOID_RECENTER_TICKS) {
            visual_avoid_recenter_ticks++;
        }
        if (visual_avoid_recenter_ticks >= VISUAL_AVOID_RECENTER_TICKS) {
            visual_avoid_done = 1u;
        }
    }
}

void VisualAvoid_GetDebug(uint8 *detected,
                          uint8 *active,
                          uint8 *upper_row,
                          uint8 *lower_row,
                          uint8 *lower_col)
{
    uint8 state;

    state = visual_avoid_state;
    if (detected != NULL) *detected = visual_avoid_detected;
    if (active != NULL) {
        *active = (state != VISUAL_AVOID_STATE_SCAN) ? 1u : 0u;
    }
    if (upper_row != NULL) *upper_row = visual_avoid_upper_row;
    if (lower_row != NULL) *lower_row = visual_avoid_lower_row;
    if (lower_col != NULL) *lower_col = visual_avoid_lower_col;
}
