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
uint8 debug_stage = 0;
uint8 pre_find_flag = 0;
int8 pre_find_offset = 0;

#define LASER_PIN      IO_P67
#define Pre_Line       60
#define TopFind        25
#define BottomFind     110
#define TargetScanColMin 45
#define TargetScanColMax 150
#define MinTargetH     10
#define MinTargetW     10
#define MaxTargetH     44
#define MaxTargetW     44
#define FindOffset     4
#define TargetPointStep 3
#define TargetWhiteCheckRows 3
#define TargetWhiteNeedRows  2
#define TargetBottomRunMax   32
#define AimTolerance   30
#define ShootTicks     2
#define CooldownTicks  34

static uint8 xdata target_bottom_y[TargetScanColMax + 1];

static int16 abs_i16(int16 x)
{
    return (x < 0) ? (int16)(-x) : x;
}

/* 将数值限制在指定范围内，避免目标搜索越界。 */
static int16 clamp_i16(int16 x, int16 min_v, int16 max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static void Target_Build_Bottom_Map(uint8 scan_min, uint8 scan_max)
{
    uint8 row;
    uint8 row_start;
    uint8 col;
    uint8 try_col;
    uint8 count = 0;
    uint8 white_count = 0;
    uint8 found_count = 0;
    int16 l_edge;
    int16 r_edge;
    uint8 far *row_ptr;
    uint8 far *row_up;
    uint8 far *row_down;

    for (col = scan_min; col <= scan_max; col++) {
        target_bottom_y[col] = 0;
    }

#define TARGET_TRY_COL(col_value)                                                   \
    do {                                                                            \
        try_col = (uint8)(col_value);                                               \
        if (!target_bottom_y[try_col] &&                                            \
            (int16)try_col > l_edge && (int16)try_col < r_edge) {                   \
            count = 0;                                                              \
            white_count = 0;                                                        \
            if (row_ptr[try_col] < tar_th) count++;                                 \
            if (try_col > 0 && row_ptr[try_col - 1] < tar_th) count++;              \
            if (try_col < SEARCH_IMAGE_W - 1 && row_ptr[try_col + 1] < tar_th) count++; \
            if (row_up[try_col] < tar_th) count++;                                  \
            if (row_down[try_col] > tar_th) white_count++;                          \
            if (try_col > 0 && row_down[try_col - 1] > tar_th) white_count++;        \
            if (try_col < SEARCH_IMAGE_W - 1 && row_down[try_col + 1] > tar_th) white_count++; \
            if (count >= 2 && white_count >= 1) {                                   \
                target_bottom_y[try_col] = row;                                     \
                found_count++;                                                      \
            }                                                                       \
        }                                                                           \
    } while (0)

    row_start = BottomFind;
    if (row_start > controlReferenceLine) {
        row_start = controlReferenceLine;
    }

    for (row = row_start; row > TopFind; row -= PIXEL_OFFSET) {
        l_edge = (int16)left_edge_line[row] + FindOffset;
        r_edge = (int16)right_edge_line[row] - FindOffset;

        if (r_edge <= l_edge) {
            continue;
        }

        row_ptr = mt9v03x_image[row];
        row_up = mt9v03x_image[row - 1];
        row_down = mt9v03x_image[row + PIXEL_OFFSET];

        for (col = scan_min; col + 3 <= scan_max; col += 4) {
            TARGET_TRY_COL(col);
            TARGET_TRY_COL(col + 1);
            TARGET_TRY_COL(col + 2);
            TARGET_TRY_COL(col + 3);
        }

        for (; col <= scan_max; col++) {
            TARGET_TRY_COL(col);
        }

        if (found_count >= (uint8)(scan_max - scan_min + 1u)) {
            break;
        }
    }

#undef TARGET_TRY_COL
}

static uint8 Target_Has_White_Above(uint8 row, uint8 col)
{
    uint8 i;
    uint8 y;
    uint8 white_rows = 0;
    int16 l_edge;
    int16 r_edge;
    uint8 far *row_ptr;

    if (row <= TargetWhiteCheckRows) {
        return 0;
    }

    for (i = 1; i <= TargetWhiteCheckRows; i++) {
        y = row - i;
        l_edge = (int16)left_edge_line[y] + FindOffset;
        r_edge = (int16)right_edge_line[y] - FindOffset;
        if (r_edge <= l_edge ||
            (int16)col <= l_edge ||
            (int16)col >= r_edge) {
            continue;
        }

        row_ptr = mt9v03x_image[y];
        if (row_ptr[col] > tar_th) {
            white_rows++;
        } else if (col > 0 && (int16)(col - 1) > l_edge &&
                   row_ptr[col - 1] > tar_th) {
            white_rows++;
        } else if (col < SEARCH_IMAGE_W - 1 && (int16)(col + 1) < r_edge &&
                   row_ptr[col + 1] > tar_th) {
            white_rows++;
        }

        if (white_rows >= TargetWhiteNeedRows) {
            return 1;
        }
    }

    return 0;
}

static uint8 Target_Bottom_Run_Valid(uint8 row, uint8 col)
{
    uint8 left;
    uint8 right;
    uint8 run_width;
    int16 l_edge;
    int16 r_edge;
    uint8 far *row_ptr;

    left = col;
    right = col;
    run_width = 1;
    row_ptr = mt9v03x_image[row];
    l_edge = (int16)left_edge_line[row] + FindOffset;
    r_edge = (int16)right_edge_line[row] - FindOffset;

    if (r_edge <= l_edge ||
        (int16)col <= l_edge ||
        (int16)col >= r_edge ||
        row_ptr[col] >= tar_th) {
        return 0;
    }

    while (left > TargetScanColMin &&
           (int16)(left - 1) > l_edge &&
           row_ptr[left - 1] < tar_th) {
        left--;
        run_width++;
        if (run_width > TargetBottomRunMax) {
            return 0;
        }
    }

    while (right < TargetScanColMax &&
           (int16)(right + 1) < r_edge &&
           row_ptr[right + 1] < tar_th) {
        right++;
        run_width++;
        if (run_width > TargetBottomRunMax) {
            return 0;
        }
    }

    return 1;
}

/* 初始化激光 GPIO，默认关闭。 */
void laser_init(void)
{
    gpio_init(LASER_PIN, GPO, 0, GPO_PUSH_PULL);
}

/* 打开激光输出。 */
void laser_on(void)
{
    gpio_set_level(LASER_PIN, 1);
}

/* 关闭激光输出。 */
void laser_off(void)
{
    gpio_set_level(LASER_PIN, 0);
}

void Laser_Task(void)
{
    static uint8 shoot_cnt = 0;
    static uint8 cooldown_cnt = 0;

    if (cooldown_cnt) {
        cooldown_cnt--;
        shoot_cnt = 0;
        laser_off();
        return;
    }

    if (!shoot_cnt && aim_ready_flag) {
        shoot_cnt = ShootTicks;
    }

    if (shoot_cnt && aim_ready_flag) {
        laser_on();
        shoot_cnt--;
        if (!shoot_cnt) {
            cooldown_cnt = CooldownTicks;
        }
    } else {
        laser_off();
        if (shoot_cnt) {
            shoot_cnt = 0;
            cooldown_cnt = CooldownTicks;
        }
    }
}

/* 在预扫描行上粗略寻找目标横向位置，给后续精找提供偏移量。 */
void Pre_Scan(void)
{
    int16 i;
    uint8 x;
    uint8 pre_l = 0;
    uint8 pre_r = 0;
    int16 l_edge;
    int16 r_edge;

    pre_find_flag = 0;

    l_edge = left_edge_line[Pre_Line] + 4;
    r_edge = right_edge_line[Pre_Line] - 4;

    l_edge = clamp_i16(l_edge, 0, SEARCH_IMAGE_W - 1);
    r_edge = clamp_i16(r_edge, 0, SEARCH_IMAGE_W - 1);

    if (r_edge <= l_edge + 8) {
        pre_find_offset = 0;
        return;
    }

    for (i = l_edge; i < r_edge; i += 2) {
        x = mt9v03x_image[Pre_Line][i];
        if (x < tar_th) {
            pre_l = (uint8)i;
            break;
        }
    }

    for (i = r_edge; i > l_edge; i -= 2) {
        x = mt9v03x_image[Pre_Line][i];
        if (x < tar_th) {
            pre_r = (uint8)i;
            break;
        }
    }

    if (pre_r <= pre_l || pre_l == 0 || pre_r == 0 || pre_r - pre_l < 8) {
        pre_find_offset = 0;
        return;
    }

    pre_find_offset = (int8)(((int16)pre_l + (int16)pre_r) / 2 - (SEARCH_IMAGE_W / 2));
    pre_find_flag = 1;
}

/* 沿预扫描得到的偏移位置查找目标区域，并更新目标坐标和标志位。 */
void Target_find(void)
{
    uint8 x;
    uint8 scan_min;
    uint8 scan_max;
    uint8 y_left;
    uint8 y_mid;
    uint8 y_right;
    uint8 mid_x;
    uint8 best_x = 0;
    uint8 best_y = 0;

    tar_flag = 0;
    center_offset = 0;
    aim_ready_flag = 0;
    debug_stage = 0;

    top_d = 0;
    under_d = 0;
    l_d = 0;
    r_d = 0;

    scan_min = TargetScanColMin;
    scan_max = TargetScanColMax;
    if (scan_max > SEARCH_IMAGE_W - 1) {
        scan_max = SEARCH_IMAGE_W - 1;
    }
    if (scan_min >= scan_max ||
        scan_max <= scan_min + TargetPointStep * 2) {
        return;
    }

    Target_Build_Bottom_Map(scan_min, scan_max);

    for (x = scan_min; x <= scan_max - TargetPointStep * 2; x++) {
        mid_x = (uint8)(x + TargetPointStep);
        y_left = target_bottom_y[x];
        y_mid = target_bottom_y[mid_x];
        y_right = target_bottom_y[x + TargetPointStep * 2];

        if (!y_left || !y_mid || !y_right) {
            continue;
        }

        if (y_mid <= controlReferenceLine &&
            y_mid >= y_left && y_mid >= y_right &&
            abs_i16((int16)y_mid - (int16)y_left) <= 12 &&
            abs_i16((int16)y_mid - (int16)y_right) <= 12 &&
            abs_i16((int16)y_left - (int16)y_right) <= 12 &&
            Target_Bottom_Run_Valid(y_mid, mid_x) &&
            Target_Has_White_Above(y_mid, mid_x)) {
            if (y_mid > best_y) {
                best_y = y_mid;
                best_x = mid_x;
            }
        }
    }

    if (!best_x) {
        return;
    }

    l_d = best_x - TargetPointStep;
    r_d = best_x + TargetPointStep;
    top_d = best_y;
    under_d = best_y;
    tar_x = best_x;
    tar_y = best_y;
    center_offset = (int16)tar_x - (int16)Mid_Col;
    aim_ready_flag = (tar_y > 70 && tar_y < 90) ? 1 : 0;
    tar_flag = 1;
    debug_stage = 6;
}
