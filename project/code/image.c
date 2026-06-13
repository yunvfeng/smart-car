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
#define REFERENCE_COL_MAX              160u

/* 当前帧备份，调试显示或后续处理可以直接看这份图。 */
uint8 far image_copy[MT9V03X_H][MT9V03X_W];

uint8 reference_point;
uint8 white_max_point;
uint8 white_min_point;

uint8 refenence_col_line[SEARCH_IMAGE_H];
uint8 reference_contrast_ratio = 32;
uint8 reference_col;

uint8 left_edge_line[SEARCH_IMAGE_H];
uint8 right_edge_line[SEARCH_IMAGE_H];
uint8 mid_line[SEARCH_IMAGE_H];

uint8 lost_left;
uint8 lost_right;

uint8 cross_flag = 0;
uint16 encoder_enter = 0;
uint8 th = 0;
int32 err_sum = 0;

uint16 camera_exposure_time = MT9V03X_EXP_TIME_DEF;
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

/* 等待摄像头完成一帧图像，超时后返回失败。 */
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
    uint16 contrast;

    globe_remote_min = SEARCH_IMAGE_H;
    reference_col = SEARCH_IMAGE_W / 2;

    for (col = REFERENCE_COL_MIN; col <= REFERENCE_COL_MAX; col += PIXEL_OFFSET) {
        globe_remote = SEARCH_IMAGE_H;

        for (row = SEARCH_IMAGE_H - 1; row > PIXEL_OFFSET; row -= PIXEL_OFFSET) {
            temp1 = *(image + (uint16)row * SEARCH_IMAGE_W + col);
            temp2 = *(image + (uint16)(row - PIXEL_OFFSET) * SEARCH_IMAGE_W + col);

            if (is_reflect_pixel(temp1) || is_reflect_pixel(temp2)) {
                continue;
            }

            if (temp2 > white_max_point) {
                continue;
            } else if (temp1 < white_min_point) {
                if (globe_remote > row) {
                    globe_remote = row;
                }
                break;
            }

            contrast = get_contrast(temp1, temp2);
            if (contrast > reference_contrast_ratio || row == STOP_ROW) {
                globe_remote = row;
                break;
            }
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

    uint8 search_time;
    uint8 temp1;
    uint8 temp2;
    int16 contrast;

    uint8 left_stop;
    uint8 right_stop;
    uint8 stop_point;
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
    stop_point = 0;

    lost_left = STOP_ROW;
    lost_right = STOP_ROW;

    for (row = row_max; row >= row_min; row--) {
        left_edge_line[row] = 0;
        right_edge_line[row] = SEARCH_IMAGE_W - 1;
        if (row == row_min) break;
    }

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

                        for (stop_point = row; stop_point > 1; stop_point--) {
                            left_edge_line[stop_point] = col_min;
                        }
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

                    contrast = (int16)get_contrast(temp1, temp2);
                    if (contrast > reference_contrast_ratio || col == col_min) {
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
            search_time = 2;
            do {
                if (search_time == 1) {
                    right_start_col = reference_col;
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

                        for (stop_point = row; stop_point > row_min; stop_point--) {
                            right_edge_line[stop_point] = SEARCH_IMAGE_W - 1;
                        }
                        lost_right = row;
                        break;
                    }

                    if (temp1 < white_min_point) {
                        right_edge_line[row] = col;
                        break;
                    }

                    if (temp2 > white_max_point) {
                        continue;
                    }

                    contrast = (int16)get_contrast(temp1, temp2);
                    if (contrast > reference_contrast_ratio || col >= col_max - PIXEL_OFFSET) {
                        right_edge_line[row] = col;
                        right_start_col = (uint8)limit_i16((int16)col - SEARCH_RANGE, col_min, col);
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

    memcpy(image_copy, mt9v03x_image, sizeof(image_copy));

    img = &mt9v03x_image[0][0];

    th = otsuThreshold((uint8 *)img);
    get_reference_point(img);
    Search_reference_col(img);
    Search_line(img);

    /* 圆环路段边线会断开或跳变，这里统一交给 Ring() 补线。 */
    Ring();

    Fitted_Midline();

    /* 识别到目标后，tar_flag 会在 15 ms 中断里控制激光开关。 */
    tar_th = white_min_point;
    Pre_Scan();
    Target_find(pre_find_offset);
}
