#include "image.h"
#include "image2.h"
#include "ring.h"
#include "pid.h"
#include "servo.h"
#include "laser.h"
#include <string.h>

#define CAMERA_AE_TARGET_BRIGHTNESS    126u
#define CAMERA_AE_BRIGHTNESS_LOW       112u
#define CAMERA_AE_BRIGHTNESS_HIGH      142u
#define CAMERA_AE_MIN_EXPOSURE         60u
#define CAMERA_AE_MAX_EXPOSURE         1200u
#define CAMERA_AE_MAX_ADJUST_TIMES     5u
#define CAMERA_AE_FRAME_TIMEOUT_MS     150u
#define CAMERA_AE_SAMPLE_START_ROW     20u
#define CAMERA_AE_SAMPLE_ROW_STEP      4u
#define CAMERA_AE_SAMPLE_COL_STEP      4u
#define REFLECT_BASE_POINT             225u
#define REFLECT_MIN_POINT              198u
#define REFLECT_WHITE_OFFSET           48u
#define WHITE_MAX_OFFSET               24u
#define REFERENCE_COL_MIN              0u
#define REFERENCE_COL_MAX              (SEARCH_IMAGE_W - 1u)
#define ZEBRA_ROW_TOP                  50u
#define ZEBRA_ROW_BOTTOM               100u
#define ZEBRA_ROW_STEP                 4u
#define ZEBRA_COL_STEP                 4u
#define ZEBRA_MARGIN                   8u
#define ZEBRA_ROW_TRANSITIONS          6u
#define ZEBRA_ROW_HITS                 3u
#define ZEBRA_COL_TRANSITIONS          4u
#define ZEBRA_COL_HITS                 3u
#define ZEBRA_DETECT_ENABLE            0u
#define IMAGE_COPY_ENABLE              0u
#define TARGET_DETECT_ENABLE           1u

/* 当前帧备份，调试显示或后续处理可以直接看这份图。 */
uint8 far image_copy[MT9V03X_H][MT9V03X_W];

uint8 reference_point;
uint8 white_max_point;
uint8 white_min_point;

uint8 refenence_col_line[SEARCH_IMAGE_H];
uint8 reference_contrast_ratio = 120;
uint8 reference_col;

uint8 left_edge_line[SEARCH_IMAGE_H];
uint8 right_edge_line[SEARCH_IMAGE_H];
uint8 mid_line[SEARCH_IMAGE_H];

uint8 lost_left;
uint8 lost_right;

uint8 cross_flag = 0;
uint8 zebra_flag = 0;
uint16 encoder_enter = 0;
uint8 th = 0;
int32 err_sum = 0;

uint16 camera_exposure_time = 35;
uint8 camera_init_brightness = 0;
static uint8 reflect_point = REFLECT_BASE_POINT;

/* 将 int16 数值限制到指定范围内。 */
static int16 limit_i16(int16 x, int16 min_v, int16 max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

/* 将 uint16 数值限制到指定范围内。 */
static uint16 limit_u16(uint16 x, uint16 min_v, uint16 max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

/* 判断像素是否过亮，过亮点通常按反光处理。 */
static uint8 is_reflect_pixel(uint8 pix)
{
    return pix >= reflect_point;
}

/* 根据当前白色阈值更新反光过滤阈值。 */
static void update_reflect_point(void)
{
    uint16 point;

    point = (uint16)white_min_point + REFLECT_WHITE_OFFSET;
    if (point < REFLECT_MIN_POINT) {
        point = REFLECT_MIN_POINT;
    } else if (point > REFLECT_BASE_POINT) {
        point = REFLECT_BASE_POINT;
    }

    reflect_point = (uint8)point;
}

/* 根据当前阈值判断像素黑白，灰区不参与斑马线跳变统计。 */
#if ZEBRA_DETECT_ENABLE
static int8 zebra_pixel_state(uint8 pix)
{
    if (pix < white_min_point) return 0;
    if (pix > white_max_point) return 1;
    return -1;
}

static uint8 Zebra_Row_Transitions(const uint8 *image, uint8 row)
{
    uint8 col;
    uint8 transitions = 0;
    int16 left;
    int16 right;
    int8 prev_state = -1;
    int8 state;
    const uint8 *row_ptr;

    left = (int16)left_edge_line[row] + ZEBRA_MARGIN;
    right = (int16)right_edge_line[row] - ZEBRA_MARGIN;

    if (left < 0) left = 0;
    if (right >= SEARCH_IMAGE_W) right = SEARCH_IMAGE_W - 1;
    if (right <= left + ZEBRA_MARGIN) return 0;

    row_ptr = image + (uint16)row * SEARCH_IMAGE_W;

    for (col = (uint8)left; col < (uint8)right; col += ZEBRA_COL_STEP) {
        state = zebra_pixel_state(row_ptr[col]);
        if (state < 0) {
            continue;
        }

        if (prev_state >= 0 && state != prev_state) {
            transitions++;
        }
        prev_state = state;
    }

    return transitions;
}

static uint8 Zebra_Col_Transitions(const uint8 *image, uint8 col)
{
    uint8 row;
    uint8 transitions = 0;
    int8 prev_state = -1;
    int8 state;
    const uint8 *row_ptr;
    uint16 row_step;

    row_ptr = image + (uint16)ZEBRA_ROW_BOTTOM * SEARCH_IMAGE_W;
    row_step = (uint16)PIXEL_OFFSET * SEARCH_IMAGE_W;

    for (row = ZEBRA_ROW_BOTTOM; row > ZEBRA_ROW_TOP; row -= PIXEL_OFFSET) {
        if (!((int16)col > (int16)left_edge_line[row] + ZEBRA_MARGIN &&
              (int16)col < (int16)right_edge_line[row] - ZEBRA_MARGIN)) {
            row_ptr -= row_step;
            continue;
        }

        state = zebra_pixel_state(row_ptr[col]);
        if (state < 0) {
            row_ptr -= row_step;
            continue;
        }

        if (prev_state >= 0 && state != prev_state) {
            transitions++;
        }
        prev_state = state;
        row_ptr -= row_step;
    }

    return transitions;
}

static uint8 Zebra_Detect(const uint8 *image)
{
    uint8 row;
    uint8 col;
    uint8 row_hits = 0;
    uint8 col_hits = 0;
    uint8 left;
    uint8 right;
    uint8 step;

    for (row = ZEBRA_ROW_TOP; row <= ZEBRA_ROW_BOTTOM; row += ZEBRA_ROW_STEP) {
        if (Zebra_Row_Transitions(image, row) >= ZEBRA_ROW_TRANSITIONS) {
            row_hits++;
            if (row_hits >= ZEBRA_ROW_HITS) {
                break;
            }
        }
    }

    left = (uint8)limit_i16((int16)left_edge_line[controlReferenceLine] + ZEBRA_MARGIN,
                            0, SEARCH_IMAGE_W - 1);
    right = (uint8)limit_i16((int16)right_edge_line[controlReferenceLine] - ZEBRA_MARGIN,
                             0, SEARCH_IMAGE_W - 1);
    if (right <= left + ZEBRA_MARGIN) {
        return 0;
    }

    step = (uint8)((right - left) / 4u);
    if (step < 8u) {
        step = 8u;
    }

    for (col = (uint8)(left + step); col < right; col += step) {
        if (Zebra_Col_Transitions(image, col) >= ZEBRA_COL_TRANSITIONS) {
            col_hits++;
            if (col_hits >= ZEBRA_COL_HITS) {
                return 1;
            }
        }
    }

    if (row_hits >= ZEBRA_ROW_HITS && col_hits > 0) {
        return 1;
    }

    return 0;
}

/* 等待摄像头完成一帧图像，超时后返回失败。 */
#endif
static uint8 camera_wait_finish_frame(uint16 timeout_ms)
{
    while (timeout_ms) {
        if (mt9v03x_finish_flag) {
            mt9v03x_finish_flag = 0;
            return 1;
        }

        system_delay_ms(1);
        timeout_ms--;
    }

    return 0;
}

/* 对图像做稀疏采样，估算当前画面的平均亮度。 */
static uint8 camera_get_average_brightness(void)
{
    uint8 row;
    uint8 col;
    uint16 sample_count;
    uint32 sum;
    const uint8 *p;

    sample_count = 0;
    sum = 0;

    for (row = CAMERA_AE_SAMPLE_START_ROW; row < MT9V03X_H; row += CAMERA_AE_SAMPLE_ROW_STEP) {
        p = &mt9v03x_image[row][0];

        for (col = 0; col < MT9V03X_W; col += CAMERA_AE_SAMPLE_COL_STEP) {
            sum += *(p + col);
            sample_count++;
        }
    }

    if (!sample_count) {
        return 0;
    }

    return (uint8)(sum / sample_count);
}

/* 上电后自动调整摄像头曝光，让初始画面亮度落到合适范围。 */
void Camera_Auto_Exposure_Init(void)
{
    uint8 i;
    uint8 brightness;
    uint16 target_exposure;
    uint16 next_exposure;
    uint32 scaled_exposure;

    camera_exposure_time = MT9V03X_EXP_TIME_DEF;
    mt9v03x_finish_flag = 0;

    for (i = 0; i < CAMERA_AE_MAX_ADJUST_TIMES; i++) {
        if (!camera_wait_finish_frame(CAMERA_AE_FRAME_TIMEOUT_MS)) {
            break;
        }

        brightness = camera_get_average_brightness();
        camera_init_brightness = brightness;

        if (brightness >= CAMERA_AE_BRIGHTNESS_LOW &&
            brightness <= CAMERA_AE_BRIGHTNESS_HIGH) {
            break;
        }

        if (!brightness) {
            brightness = 1;
        }

        scaled_exposure = (uint32)camera_exposure_time * CAMERA_AE_TARGET_BRIGHTNESS;
        target_exposure = (uint16)((scaled_exposure + brightness / 2u) / brightness);
        next_exposure = (uint16)(((uint32)camera_exposure_time + target_exposure + 1u) >> 1);

        if (next_exposure < camera_exposure_time * 3u / 4u) {
            next_exposure = camera_exposure_time * 3u / 4u;
        }

        if (next_exposure > camera_exposure_time * 2u) {
            next_exposure = camera_exposure_time * 2u;
        }

        next_exposure = limit_u16(next_exposure, CAMERA_AE_MIN_EXPOSURE, CAMERA_AE_MAX_EXPOSURE);

        if (next_exposure == camera_exposure_time) {
            break;
        }

        camera_exposure_time = next_exposure;
        mt9v03x_set_exposure_time(camera_exposure_time);
    }

    if (camera_wait_finish_frame(CAMERA_AE_FRAME_TIMEOUT_MS)) {
        camera_init_brightness = camera_get_average_brightness();
    }
    mt9v03x_finish_flag = 0;
}

/* 计算两个灰度值之间的对比度，数值越大表示边缘越明显。 */
uint16 get_contrast(uint8 temp1, uint8 temp2)
{
    int16 diff;
    diff = (int16)temp1 - (int16)temp2;
    if (diff < 0) diff = -diff;
    return (uint16)((uint32)diff * 200u / ((uint16)temp1 + (uint16)temp2 + 1u));
}

static uint8 contrast_over_threshold(uint8 temp1, uint8 temp2, uint8 threshold)
{
    uint16 diff;
    uint16 sum;
    uint16 scaled_diff;
    uint16 scaled_threshold;

    diff = (temp1 >= temp2) ? ((uint16)temp1 - (uint16)temp2) :
                              ((uint16)temp2 - (uint16)temp1);
    sum = (uint16)temp1 + (uint16)temp2 + 1u;

    /* threshold 最大为 120，两侧最大值 51000/61831，uint16 足够。 */
    scaled_diff = (uint16)(diff * 200u);
    scaled_threshold = (uint16)(((uint16)threshold + 1u) * sum);
    return (scaled_diff >= scaled_threshold) ? 1u : 0u;
}

/* 根据图像底部参考区域估算黑白阈值和反光过滤阈值。 */
void get_reference_point(const uint8 *image)
{
    uint16 i;
    const uint8 *p;
    uint16 count;
    uint16 valid_count;
    uint32 sum;
    uint8 pix;
    uint8 reference_min;

    p = image + (SEARCH_IMAGE_H - REFRENCE_ROW) * SEARCH_IMAGE_W;
    count = REFRENCE_ROW * SEARCH_IMAGE_W;
    valid_count = 0;
    sum = 0;

    for (i = 0; i < count; i++) {
        pix = *(p + i);
        if (!is_reflect_pixel(pix)) {
            sum += pix;
            valid_count++;
        }
    }

    if (!valid_count) {
        for (i = 0; i < count; i++) {
            sum += *(p + i);
        }
        valid_count = count;
    }

    reference_point = (uint8)(sum / valid_count);

    reference_min = (uint8)limit_i16((int16)((uint16)reference_point * WHITE_MIN_MUL / 10u),
                                     BALACK_POINT, 220);

    if (th >= OTSU_LIMIT_MIN && th <= OTSU_LIMIT_MAX) {
        white_min_point = (uint8)limit_i16((int16)(((uint16)reference_min + (uint16)th + 1u) >> 1),
                                           BALACK_POINT, 220);
    } else {
        white_min_point = reference_min;
    }

    update_reflect_point();

    white_max_point = (uint8)limit_i16((int16)((uint16)white_min_point + WHITE_MAX_OFFSET),
                                       (int16)white_min_point + 8, (int16)reflect_point - 1);
}

/* 在整幅图中寻找一条较可靠的参考列，作为左右寻线的起点。 */
void Search_reference_col(const uint8 *image)
{
    uint8 col;
    uint8 row;
    uint8 temp1;
    uint8 temp2;
    uint8 i;
    uint8 globe_remote;
    uint8 globe_remote_min;
    uint16 row_step;
    const uint8 *row_ptr;

    globe_remote_min = SEARCH_IMAGE_H;
    reference_col = SEARCH_IMAGE_W / 2;
    row_step = (uint16)PIXEL_OFFSET * SEARCH_IMAGE_W;

    for (col = REFERENCE_COL_MIN; col <= REFERENCE_COL_MAX; col += PIXEL_OFFSET) {
        globe_remote = SEARCH_IMAGE_H;
        row_ptr = image + (uint16)(SEARCH_IMAGE_H - 1u) * SEARCH_IMAGE_W + col;

        for (row = SEARCH_IMAGE_H - 1; row > PIXEL_OFFSET; row -= PIXEL_OFFSET) {
            temp1 = *row_ptr;
            temp2 = *(row_ptr - row_step);

            if (is_reflect_pixel(temp1) || is_reflect_pixel(temp2)) {
                row_ptr -= row_step;
                continue;
            }

            if (temp2 > white_max_point) {
                row_ptr -= row_step;
                continue;
            } else if (temp1 < white_min_point) {
                if (globe_remote > row) {
                    globe_remote = row;
                }
                break;
            }

            if (row == STOP_ROW ||
                contrast_over_threshold(temp1, temp2, reference_contrast_ratio)) {
                globe_remote = row;
                break;
            }

            row_ptr -= row_step;
        }

        if (globe_remote < globe_remote_min) {
            globe_remote_min = globe_remote;
            reference_col = col;
        }
    }

    for (i = 0; i < SEARCH_IMAGE_H; i++) {
        refenence_col_line[i] = reference_col;
    }
}

/* 从下往上搜索左右边线，并把缺失行插值补齐。 */
void Search_line(const uint8 *image)
{
    const uint8 *p;
    uint8 row_max;
    uint8 row_min;
    uint8 col_max;
    uint8 col_min;

    uint8 left_start_col;
    uint8 right_start_col;
    uint8 left_end_col;
    uint8 right_end_col;
    uint8 right_min_col;

    uint8 search_time;
    uint8 temp1;
    uint8 temp2;
    uint8 left_stop;
    uint8 right_stop;
    uint8 col;
    uint8 row;

    p = image;
    row_max = SEARCH_IMAGE_H - 1;
    row_min = STOP_ROW;
    col_max = SEARCH_IMAGE_W - 1;
    col_min = 0;

    left_start_col  = reference_col;
    right_start_col = (uint8)limit_i16((int16)reference_col, 0, col_max);
    left_end_col    = col_min;
    right_end_col   = col_max;

    left_stop = 0;
    right_stop = 0;
    lost_left = STOP_ROW;
    lost_right = STOP_ROW;

    memset(&left_edge_line[row_min], 0, row_max - row_min + 1u);
    memset(&right_edge_line[row_min], SEARCH_IMAGE_W - 1, row_max - row_min + 1u);

    for (row = row_max; row >= row_min; row -= PIXEL_OFFSET) {
        p = image + (uint16)row * SEARCH_IMAGE_W;

        if (!left_stop) {
            search_time = 2;
            do {
                if (search_time == 1) {
                    left_start_col = reference_col;
                    left_end_col = col_min;
                }
                search_time--;

                for (col = left_start_col; col > left_end_col + PIXEL_OFFSET; col -= PIXEL_OFFSET) {
                    temp1 = *(p + col);
                    temp2 = *(p + col - PIXEL_OFFSET);

                    if (is_reflect_pixel(temp1) || is_reflect_pixel(temp2)) {
                        continue;
                    }

                    if (temp1 < white_min_point &&
                        col == left_start_col &&
                        left_start_col == reference_col) {
                        left_stop = 1;
                        search_time = 0;

                        memset(&left_edge_line[2], col_min, row - 1u);
                        lost_left = row;
                        break;
                    }

                    if (temp1 < white_min_point) {
                        left_edge_line[row] = col;
                        break;
                    }

                    if (temp2 > white_max_point) {
                        continue;
                    }

                    if (col == col_min ||
                        contrast_over_threshold(temp1, temp2, reference_contrast_ratio)) {
                        left_edge_line[row] = col;
                        left_start_col = (uint8)limit_i16((int16)col + SEARCH_RANGE, col, col_max);
                        left_end_col   = (uint8)limit_i16((int16)col - SEARCH_RANGE, col_min, col);
                        search_time = 0;
                        break;
                    }
                }
            } while (search_time);
        }

        if (!right_stop) {
            /*
             * 右线始终从参考列右侧搜索，并且不得落到当前左线左侧。
             * 上一行的局部搜索窗可以跟随弯道，但不允许窗口越过这条下界。
             */
            right_min_col = (uint8)limit_i16((int16)left_edge_line[row] + PIXEL_OFFSET,
                                              reference_col,
                                              col_max);
            if (right_start_col < right_min_col) {
                right_start_col = right_min_col;
            }

            search_time = 2;
            do {
                if (search_time == 1) {
                    right_start_col = right_min_col;
                    right_end_col = col_max;
                }
                search_time--;

                for (col = right_start_col; col <= right_end_col - PIXEL_OFFSET; col += PIXEL_OFFSET) {
                    temp1 = *(p + col);
                    temp2 = *(p + col + PIXEL_OFFSET);

                    if (is_reflect_pixel(temp1) || is_reflect_pixel(temp2)) {
                        continue;
                    }

                    if (temp1 < white_min_point &&
                        col == right_start_col &&
                        right_start_col == reference_col) {
                        right_stop = 1;
                        search_time = 0;

                        memset(&right_edge_line[row_min + 1u],
                               SEARCH_IMAGE_W - 1,
                               row - row_min);
                        lost_right = row;
                        break;
                    }

                    if (temp1 < white_min_point) {
                        /*
                         * 局部窗口内的黑点先触发从 right_min_col 的复搜。
                         * 只有复搜仍找到时才落点，避免保留局部误点。
                         */
                        if (search_time == 0) {
                            right_edge_line[row] = col;
                        }
                        break;
                    }

                    if (temp2 > white_max_point) {
                        continue;
                    }

                    if (col >= col_max - PIXEL_OFFSET ||
                        contrast_over_threshold(temp1, temp2, reference_contrast_ratio)) {
                        right_edge_line[row] = col;
                        right_start_col = (uint8)limit_i16((int16)col - SEARCH_RANGE,
                                                           right_min_col,
                                                           col);
                        right_end_col   = (uint8)limit_i16((int16)col + SEARCH_RANGE, col, col_max);
                        search_time = 0;
                        break;
                    }
                }
            } while (search_time);
        }

        if (row < row_min + PIXEL_OFFSET) {
            break;
        }
    }

    insert_val();

    /* 插值后再做一次几何保护，右线不允许与左线交叉。 */
    for (row = row_max; row >= row_min; row--) {
        if (right_edge_line[row] <= left_edge_line[row]) {
            right_edge_line[row] = col_max;
        }
        if (row == row_min) {
            break;
        }
    }

    memcpy(left_control_line, left_edge_line, sizeof(left_edge_line));
    memcpy(right_control_line, right_edge_line, sizeof(right_edge_line));
}

/* 对按 PIXEL_OFFSET 搜到的稀疏边线做行间插值。 */
void insert_val(void)
{
    uint8 i;
    uint8 j;
    uint8 front_p;
    uint8 cur_p;
    uint8 step;

    front_p = left_edge_line[SEARCH_IMAGE_H - 1];
    for (i = SEARCH_IMAGE_H - PIXEL_OFFSET - 1; i >= STOP_ROW; i -= PIXEL_OFFSET) {
        cur_p = left_edge_line[i];
        if (cur_p >= front_p) {
            step = (uint8)((cur_p - front_p + PIXEL_OFFSET - 1) / PIXEL_OFFSET);
            for (j = 1; j < PIXEL_OFFSET; j++) {
                left_edge_line[i - j + PIXEL_OFFSET] = front_p + step * j;
            }
        } else {
            step = (uint8)((front_p - cur_p + PIXEL_OFFSET - 1) / PIXEL_OFFSET);
            for (j = 1; j < PIXEL_OFFSET; j++) {
                left_edge_line[i - j + PIXEL_OFFSET] = front_p - step * j;
            }
        }
        front_p = cur_p;

        if (i < STOP_ROW + PIXEL_OFFSET) {
            break;
        }
    }

    front_p = right_edge_line[SEARCH_IMAGE_H - 1];
    for (i = SEARCH_IMAGE_H - PIXEL_OFFSET - 1; i >= STOP_ROW; i -= PIXEL_OFFSET) {
        cur_p = right_edge_line[i];
        if (cur_p >= front_p) {
            step = (uint8)((cur_p - front_p + PIXEL_OFFSET - 1) / PIXEL_OFFSET);
            for (j = 1; j < PIXEL_OFFSET; j++) {
                right_edge_line[i - j + PIXEL_OFFSET] = front_p + step * j;
            }
        } else {
            step = (uint8)((front_p - cur_p + PIXEL_OFFSET - 1) / PIXEL_OFFSET);
            for (j = 1; j < PIXEL_OFFSET; j++) {
                right_edge_line[i - j + PIXEL_OFFSET] = front_p - step * j;
            }
        }
        front_p = cur_p;

        if (i < STOP_ROW + PIXEL_OFFSET) {
            break;
        }
    }
}

/* 根据左右控制线计算中线，舵机控制主要使用这条线。 */
void Fitted_Midline(void)
{
    uint8 i;
    for (i = 0; i < SEARCH_IMAGE_H; i++) {
        mid_line[i] = (uint8)(((uint16)left_control_line[i] + (uint16)right_control_line[i]) >> 1);
    }
}

/* 分段计算中线平均位置，给速度规划和舵机控制提供整体偏移。 */
void Error_sum(void)
{
    uint8 i;
    int32 err1 = 0;
    int32 err2 = 0;
    int32 err3 = 0;

    for (i = 8; i < 28; i++) {
        err1 += mid_line[i];
    }
    err1 /= 20;

    for (i = 28; i < 88; i++) {
        err2 += mid_line[i];
    }
    err2 /= 60;

    for (i = 89; i < 111; i++) {
        err3 += mid_line[i];
    }
    err3 /= 22;

    err_sum = (err1 * 20 + err2 * 30 + err3 * 60) / 100;
}

/* 根据左右边线是否连续，切换直道和转弯时的舵机 P 参数。 */
void straightAccelerate(void)
{
    uint8 isContinue_left;
    uint8 isContinue_right;

    isContinue_left = isContinueLine(left_edge_line);
    isContinue_right = isContinueLine(right_edge_line);

    if (isContinue_left && isContinue_right) {
        servo_pidf.kp = SERVO_KP_STRAIGHT_Q10;
    } else {
        servo_pidf.kp = SERVO_KP_TURN_Q10;
    }
}

/* 单帧图像处理总入口：阈值、寻线、圆环、中线和目标识别都在这里更新。 */
void Image_OldStyle_Process(void)
{
    const uint8 *img;
    static uint8 otsu_frame_count = 0;

#if IMAGE_COPY_ENABLE
    memcpy(image_copy, mt9v03x_image, sizeof(image_copy));
#endif

    img = &mt9v03x_image[0][0];

    /* 光照阈值变化远慢于边线，每隔几帧更新可减少直方图和除法开销。 */
    if (!otsu_frame_count) {
        th = otsuThreshold((uint8 *)img);
    }
    otsu_frame_count++;
    if (otsu_frame_count >= OTSU_FRAME_DIV) {
        otsu_frame_count = 0;
    }
    get_reference_point(img);
    Search_reference_col(img);
    Search_line(img);

    /* 圆环路段边线会断开或跳变，这里统一交给 Ring() 补线。 */
    zebra_flag = 0;
    Ring();

    Fitted_Midline();

    /* 识别到目标后，tar_flag 会在 15 ms 中断里控制激光开关。 */
    tar_th = white_min_point;
    pre_find_flag = 0;
    pre_find_offset = 0;
    tar_flag = 0;
    aim_ready_flag = 0;
    center_offset = 0;
    top_d = 0;
    under_d = 0;
    l_d = 0;
    r_d = 0;
    debug_stage = 0;

#if TARGET_DETECT_ENABLE
    if (current_step >= 2) {
        return;
    }
    Target_find();
#endif
}
