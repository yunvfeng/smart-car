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

#define LASER_PIN_LEFT    IO_P61
#define LASER_PIN_CENTER  IO_P63
#define LASER_PIN_RIGHT   IO_P66
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

static uint8 laser_inhibit = 0;
static uint8 laser_shoot_cnt = 0;
static uint8 laser_cooldown_cnt = 0;

static int16 abs_i16(int16 x)
{
    return (x < 0) ? (int16)(-x) : x;
}

static int16 clamp_i16(int16 x, int16 min_v, int16 max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

void laser_init(void)
{
    laser_inhibit = 0;
    laser_shoot_cnt = 0;
    laser_cooldown_cnt = 0;
    gpio_init(LASER_PIN_LEFT, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LASER_PIN_CENTER, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LASER_PIN_RIGHT, GPO, 0, GPO_PUSH_PULL);
}

void laser_on(void)
{
    gpio_set_level(LASER_PIN_LEFT, 1);
    gpio_set_level(LASER_PIN_CENTER, 1);
    gpio_set_level(LASER_PIN_RIGHT, 1);
}

void laser_off(void)
{
    gpio_set_level(LASER_PIN_LEFT, 0);
    gpio_set_level(LASER_PIN_CENTER, 0);
    gpio_set_level(LASER_PIN_RIGHT, 0);
}

void Laser_Set_Inhibit(uint8 inhibit)
{
    laser_inhibit = inhibit ? 1u : 0u;
    if (laser_inhibit) {
        aim_ready_flag = 0;
        tar_flag = 0;
        laser_shoot_cnt = 0;
        laser_cooldown_cnt = 0;
        laser_off();
    }
}

void Laser_Task(void)
{
    if (laser_inhibit) {
        laser_shoot_cnt = 0;
        laser_cooldown_cnt = 0;
        laser_off();
        return;
    }

    if (laser_cooldown_cnt) {
        laser_cooldown_cnt--;
        laser_shoot_cnt = 0;
        laser_off();
        return;
    }

    if (!laser_shoot_cnt && aim_ready_flag) {
        laser_shoot_cnt = ShootTicks;
    }

    if (laser_shoot_cnt && aim_ready_flag) {
        laser_on();
        laser_shoot_cnt--;
        if (!laser_shoot_cnt) {
            laser_cooldown_cnt = CooldownTicks;
        }
    } else {
        laser_off();
        if (laser_shoot_cnt) {
            laser_shoot_cnt = 0;
            laser_cooldown_cnt = CooldownTicks;
        }
    }
}

/* Git 初版的轻量预扫描：只在一行内估算目标横向偏移。 */
void Pre_Scan(void)
{
    int16 i;
    uint8 x;
    uint8 pre_l = 0;
    uint8 pre_r = 0;
    int16 l_edge;
    int16 r_edge;

    pre_find_flag = 0;
    l_edge = (int16)left_edge_line[Pre_Line] + 4;
    r_edge = (int16)right_edge_line[Pre_Line] - 4;
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

    if (pre_r <= pre_l || !pre_l || !pre_r || pre_r - pre_l < 8) {
        pre_find_offset = 0;
        return;
    }

    pre_find_offset = (int8)(((int16)pre_l + (int16)pre_r) / 2 -
                             (SEARCH_IMAGE_W / 2));
    pre_find_flag = 1;
}

/* Git 初版的轻量靶点搜索：纵横各扫描一条线，不建立整幅 ROI 映射。 */
void Target_find(int8 mid_offset)
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

    center_x = (SEARCH_IMAGE_W >> 1) + mid_offset;
    center_x = clamp_i16(center_x, 2, SEARCH_IMAGE_W - 3);

    for (i = TopFind; i < BottomFind; i += FindOffset) {
        if (mt9v03x_image[i][center_x] < tar_th) {
            top_d = (uint8)i;
            debug_stage = 1;
            break;
        }
    }
    if (!top_d) return;

    for (i = BottomFind; i > TopFind; i -= FindOffset) {
        if (mt9v03x_image[i][center_x] < tar_th) {
            under_d = (uint8)i;
            debug_stage = 2;
            break;
        }
    }
    if (!under_d || under_d <= top_d || under_d - top_d < MinTargetH) return;

    tar_y_local = (top_d + under_d) >> 1;
    debug_stage = 3;
    row_ptr = mt9v03x_image[tar_y_local];

    /* 从目标内部向外寻找由黑到白的边界，修正初版在中心黑点立即停下的问题。 */
    for (j = center_x; j > 1; j--) {
        if (row_ptr[j] >= tar_th) {
            l_d = (uint8)(j + 1);
            debug_stage = 4;
            break;
        }
    }
    if (!l_d) return;

    for (j = center_x; j < SEARCH_IMAGE_W - 2; j++) {
        if (row_ptr[j] >= tar_th) {
            r_d = (uint8)(j - 1);
            debug_stage = 5;
            break;
        }
    }
    if (!r_d || r_d <= l_d) return;

    target_w = r_d - l_d;
    target_h = under_d - top_d;
    if (target_w < MinTargetW || target_w > MaxTargetW) return;
    if (target_h < MinTargetH || target_h > MaxTargetH) return;
    if (target_w > (uint8)(target_h * 3u)) return;
    if (target_h > (uint8)(target_w * 3u)) return;
    if (l_d <= 1 || r_d >= SEARCH_IMAGE_W - 2) return;

    tar_x = (l_d + r_d) >> 1;
    tar_y = tar_y_local;
    center_offset = (int16)tar_x - (int16)Mid_Col;
    aim_ready_flag = (abs_i16(center_offset) < AimTolerance) ? 1u : 0u;
    tar_flag = 1;
    debug_stage = 6;
}
