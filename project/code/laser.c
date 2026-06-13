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
#define Pre_Line       100
#define TopFind        25
#define BottomFind     110
#define MinTargetH     10
#define MinTargetW     10
#define MaxTargetH     44
#define MaxTargetW     44
#define FindOffset     4
#define AimTolerance   5
#define ShootTicks     2
#define CooldownTicks  34

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
void Target_find(int8 midOffset)
{
    int16 i;
    int16 j;
    int16 center_x;
    uint8 target_w;
    uint8 target_h;
    uint8 tar_y_local;
    uint8 far *row_ptr;

    tar_flag = 0;
    center_offset = 0;
    aim_ready_flag = 0;
    debug_stage = 0;

    top_d = 0;
    under_d = 0;
    l_d = 0;
    r_d = 0;

    center_x = (SEARCH_IMAGE_W >> 1) + midOffset;
    center_x = clamp_i16(center_x, 2, SEARCH_IMAGE_W - 3);

    for (i = TopFind; i < BottomFind; i += FindOffset) {
        if (mt9v03x_image[i][center_x] < tar_th) {
            top_d = (uint8)i;
            debug_stage = 1;
            break;
        }
    }
    if (top_d == 0) return;

    for (i = BottomFind; i > TopFind; i -= FindOffset) {
        if (mt9v03x_image[i][center_x] < tar_th) {
            under_d = (uint8)i;
            debug_stage = 2;
            break;
        }
    }
    if (under_d == 0) return;

    if (under_d <= top_d) return;
    if ((under_d - top_d) < MinTargetH) return;

    tar_y_local = (top_d + under_d) >> 1;
    debug_stage = 3;

    row_ptr = mt9v03x_image[tar_y_local];

    for (j = center_x; j >= FindOffset; j -= FindOffset) {
        if (row_ptr[j] < tar_th) {
            l_d = (uint8)j;
            debug_stage = 4;
            break;
        }
    }
    if (l_d == 0) return;

    for (j = center_x; j < SEARCH_IMAGE_W - FindOffset; j += FindOffset) {
        if (row_ptr[j] < tar_th) {
            r_d = (uint8)j;
            debug_stage = 5;
            break;
        }
    }
    if (r_d == 0) return;

    if (r_d <= l_d) return;
    target_w = r_d - l_d;
    target_h = under_d - top_d;
    if (target_w < MinTargetW || target_w > MaxTargetW) return;
    if (target_h < MinTargetH || target_h > MaxTargetH) return;
    if (target_w > (uint8)(target_h * 3u)) return;
    if (target_h > (uint8)(target_w * 3u)) return;

    if (l_d <= 1) return;
    if (r_d >= SEARCH_IMAGE_W - 2) return;

    tar_x = (l_d + r_d) >> 1;
    tar_y = tar_y_local;
    center_offset = (int16)tar_x - (int16)Mid_Col;
    aim_ready_flag = (abs_i16(center_offset) < AimTolerance) ? 1 : 0;
    tar_flag = 1;
    debug_stage = 6;
}
