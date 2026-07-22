#include "laser.h"
#include "image.h"

uint16 tar_i = 0;
uint8 tar_flag = 0;
uint8 tar_x = 0;
uint8 tar_y = 0;
uint8 tar_th = 120;
int16 center_offset = 0;
uint8 aim_ready_flag = 0;

uint8 top_d = 0;
uint8 under_d = 0;
uint8 l_d = 0;
uint8 r_d = 0;
uint8 debug_stage = TARGET_LASER_NONE;
uint8 pre_find_flag = 0;
int8 pre_find_offset = 0;

#define LASER_PIN_LEFT              IO_P61
#define LASER_PIN_CENTER            IO_P63
#define LASER_PIN_RIGHT             IO_P66
#define BUZZER_PIN                  IO_P67
#define BUZZER_ACTIVE_LEVEL         0u
#define BUZZER_IDLE_LEVEL           1u

#define LASER_MASK_NONE             0x00u
#define LASER_MASK_LEFT             0x01u
#define LASER_MASK_CENTER           0x02u
#define LASER_MASK_RIGHT            0x04u

#define TARGET_SCAN_TOP_ROW         61u
#define TARGET_SCAN_BOTTOM_ROW      79u
#define TARGET_SCAN_STEP            2u
#define TARGET_NOTCH_MIN_PX         8
#define TARGET_CENTER_ROW_TOL       4
#define TARGET_LANE_WIDTH_MIN       20u
#define TARGET_REARM_CLEAR_FRAMES   2u

#define LASER_SECOND_TICK_FLAG      0x80u
#define LASER_SHOOT_MASK            0x07u
#define CooldownTicks               34u

static volatile uint8 laser_pending_mask = LASER_MASK_NONE;
static uint8 laser_shoot_state = LASER_MASK_NONE;
static uint8 laser_cooldown_cnt = 0;
static uint8 laser_output_mask = LASER_MASK_NONE;

static int16 abs_i16(int16 x)
{
    return (x < 0) ? (int16)(-x) : x;
}

static uint8 target_edge_valid(uint8 row, uint8 left, uint8 right)
{
    if (row <= lost_left || row <= lost_right) return 0u;
    if (left <= 1u || right >= SEARCH_IMAGE_W - 2u) return 0u;
    if (left >= right) return 0u;
    if ((uint8)(right - left) < TARGET_LANE_WIDTH_MIN) return 0u;
    return 1u;
}

static void target_clear_debug(void)
{
    tar_i = 0;
    tar_flag = 0;
    tar_x = 0;
    tar_y = 0;
    center_offset = 0;
    aim_ready_flag = 0;
    top_d = 0;
    under_d = 0;
    l_d = 0;
    r_d = 0;
    debug_stage = TARGET_LASER_NONE;
    pre_find_offset = 0;
}

/* Main-loop producer and 15 ms ISR consumer share one atomic byte. */
static void laser_publish_request(uint8 mask)
{
    laser_pending_mask = mask;
}

static void laser_apply_mask(uint8 mask)
{
    if (mask == laser_output_mask) return;

    gpio_set_level(LASER_PIN_LEFT, (mask == LASER_MASK_LEFT) ? 1u : 0u);
    gpio_set_level(LASER_PIN_CENTER, (mask == LASER_MASK_CENTER) ? 1u : 0u);
    gpio_set_level(LASER_PIN_RIGHT, (mask == LASER_MASK_RIGHT) ? 1u : 0u);
    gpio_set_level(BUZZER_PIN,
                   (mask == LASER_MASK_NONE) ? BUZZER_IDLE_LEVEL :
                                               BUZZER_ACTIVE_LEVEL);
    laser_output_mask = mask;
}

static void laser_off(void)
{
    laser_apply_mask(LASER_MASK_NONE);
}

void laser_init(void)
{
    laser_pending_mask = LASER_MASK_NONE;
    laser_shoot_state = LASER_MASK_NONE;
    laser_cooldown_cnt = 0;
    laser_output_mask = LASER_MASK_NONE;
    pre_find_flag = 0;
    target_clear_debug();

    gpio_init(LASER_PIN_LEFT, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LASER_PIN_CENTER, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LASER_PIN_RIGHT, GPO, 0, GPO_PUSH_PULL);
    gpio_init(BUZZER_PIN, GPO, BUZZER_IDLE_LEVEL, GPO_PUSH_PULL);
}

void Laser_Set_Inhibit(uint8 inhibit)
{
    if (inhibit) {
        laser_pending_mask = LASER_MASK_NONE;
        laser_shoot_state = LASER_MASK_NONE;
        laser_cooldown_cnt = 0;
        laser_off();
    }
}

void Laser_Task(void)
{
    uint8 request;

    /* A request that arrives during cooldown is deliberately discarded. */
    if (laser_cooldown_cnt) {
        laser_cooldown_cnt--;
        laser_pending_mask = LASER_MASK_NONE;
        laser_shoot_state = LASER_MASK_NONE;
        laser_off();
        return;
    }

    if (laser_shoot_state & LASER_SECOND_TICK_FLAG) {
        laser_pending_mask = LASER_MASK_NONE;
        laser_apply_mask((uint8)(laser_shoot_state & LASER_SHOOT_MASK));
        laser_shoot_state = LASER_MASK_NONE;
        laser_cooldown_cnt = CooldownTicks;
        return;
    }

    request = laser_pending_mask;
    laser_pending_mask = LASER_MASK_NONE;
    if (request != LASER_MASK_LEFT &&
        request != LASER_MASK_CENTER &&
        request != LASER_MASK_RIGHT) {
        laser_shoot_state = LASER_MASK_NONE;
        laser_off();
        return;
    }

    laser_shoot_state = (uint8)(request | LASER_SECOND_TICK_FLAG);
    laser_apply_mask(request);
}

void Target_Notch_Reset(void)
{
    pre_find_flag = 0;
    target_clear_debug();
    laser_publish_request(LASER_MASK_NONE);
}

/*
 * Reuse the ten raw Search_line points at rows 61, 63, ... 79.
 * The endpoints form a conservative perspective baseline; the eight interior
 * rows are each loaded exactly once, for at most twenty edge-array reads.
 */
void Target_Notch_ProcessFrame(void)
{
    uint8 row;
    uint8 left;
    uint8 right;
    uint8 left_top;
    uint8 right_top;
    uint8 left_bottom;
    uint8 right_bottom;
    uint8 left_baseline;
    uint8 right_baseline;
    uint8 left_peak;
    uint8 right_valley;
    uint8 left_peak_row;
    uint8 right_valley_row;
    uint8 request;
    uint8 laser_id;
    int16 left_depth;
    int16 right_depth;

    left_top = left_edge_line[TARGET_SCAN_TOP_ROW];
    right_top = right_edge_line[TARGET_SCAN_TOP_ROW];
    left_bottom = left_edge_line[TARGET_SCAN_BOTTOM_ROW];
    right_bottom = right_edge_line[TARGET_SCAN_BOTTOM_ROW];

    if (!target_edge_valid(TARGET_SCAN_TOP_ROW, left_top, right_top) ||
        !target_edge_valid(TARGET_SCAN_BOTTOM_ROW, left_bottom, right_bottom)) {
        target_clear_debug();
        if (pre_find_flag) pre_find_flag = 1u;
        laser_publish_request(LASER_MASK_NONE);
        return;
    }

    left_baseline = (left_top > left_bottom) ? left_top : left_bottom;
    right_baseline = (right_top < right_bottom) ? right_top : right_bottom;

    left_peak = 0;
    right_valley = SEARCH_IMAGE_W - 1u;
    left_peak_row = 0;
    right_valley_row = 0;

    for (row = TARGET_SCAN_TOP_ROW + TARGET_SCAN_STEP;
         row < TARGET_SCAN_BOTTOM_ROW;
         row += TARGET_SCAN_STEP) {
        left = left_edge_line[row];
        right = right_edge_line[row];

        if (!target_edge_valid(row, left, right)) {
            target_clear_debug();
            if (pre_find_flag) pre_find_flag = 1u;
            laser_publish_request(LASER_MASK_NONE);
            return;
        }

        if (left > left_peak) {
            left_peak = left;
            left_peak_row = row;
        }
        if (right < right_valley) {
            right_valley = right;
            right_valley_row = row;
        }
    }

    left_depth = (int16)left_peak - (int16)left_baseline;
    right_depth = (int16)right_baseline - (int16)right_valley;
    request = LASER_MASK_NONE;
    laser_id = TARGET_LASER_NONE;

    if (left_depth >= TARGET_NOTCH_MIN_PX &&
        right_depth >= TARGET_NOTCH_MIN_PX) {
        if (abs_i16((int16)left_peak_row - (int16)right_valley_row) <=
            TARGET_CENTER_ROW_TOL) {
            request = LASER_MASK_CENTER;
            laser_id = TARGET_LASER_CENTER;
        }
    } else if (left_depth >= TARGET_NOTCH_MIN_PX) {
        request = LASER_MASK_LEFT;
        laser_id = TARGET_LASER_LEFT;
    } else if (right_depth >= TARGET_NOTCH_MIN_PX) {
        request = LASER_MASK_RIGHT;
        laser_id = TARGET_LASER_RIGHT;
    }

    if (request == LASER_MASK_NONE) {
        target_clear_debug();
        laser_publish_request(LASER_MASK_NONE);

        if (pre_find_flag == 1u) pre_find_flag = TARGET_REARM_CLEAR_FRAMES;
        else if (pre_find_flag >= TARGET_REARM_CLEAR_FRAMES) pre_find_flag = 0;
        return;
    }

    tar_i = laser_id;
    tar_flag = 1u;
    tar_y = (laser_id == TARGET_LASER_LEFT) ? left_peak_row :
            ((laser_id == TARGET_LASER_RIGHT) ? right_valley_row :
             (uint8)(((uint16)left_peak_row + (uint16)right_valley_row) >> 1));
    tar_x = (laser_id == TARGET_LASER_LEFT) ? left_peak :
            ((laser_id == TARGET_LASER_RIGHT) ? right_valley :
             (uint8)(((uint16)left_peak + (uint16)right_valley) >> 1));
    center_offset = (int16)tar_x - (int16)Mid_Col;
    aim_ready_flag = 1u;
    top_d = TARGET_SCAN_TOP_ROW;
    under_d = TARGET_SCAN_BOTTOM_ROW;
    l_d = left_peak;
    r_d = right_valley;
    debug_stage = laser_id;
    pre_find_offset = 0;

    if (!pre_find_flag) {
        laser_publish_request(request);
    }
    pre_find_flag = 1u;
}
