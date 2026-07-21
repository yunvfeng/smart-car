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
#define REFERENCE_COL_STEP             8u
#define REFERENCE_REUSE_MAX_END_ROW     10u
#define REFERENCE_SAMPLE_STEP          4u
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
#define TARGET_FIND_FRAME_DIV          2u

uint8 reference_point;
uint8 white_max_point;
uint8 white_min_point;

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
uint16 camera_exposure_time = 512;
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
    uint8 reflect_limit;

    p = image + (SEARCH_IMAGE_H - REFRENCE_ROW) * SEARCH_IMAGE_W;
    count = REFRENCE_ROW * SEARCH_IMAGE_W;
    valid_count = 0;
    sum = 0;
    reflect_limit = reflect_point;

    for (i = 0; i < count; i += REFERENCE_SAMPLE_STEP) {
        pix = *(p + i);
        if (pix < reflect_limit) {
            sum += pix;
            valid_count++;
        }
    }

    if (!valid_count) {
        for (i = 0; i < count; i += REFERENCE_SAMPLE_STEP) {
            sum += *(p + i);
        }
        valid_count = count / REFERENCE_SAMPLE_STEP;
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
    static uint8 reference_ready = 0u;
    uint8 col;
    uint8 row;
    uint8 temp1;
    uint8 temp2;
    uint8 globe_remote;
    uint8 globe_remote_min;
    uint8 reflect_limit;
    uint8 white_min;
    uint8 white_max;
    uint8 contrast_ratio;
    uint16 row_step;
    const uint8 *row_ptr;
    const uint8 *bottom_ptr;

    row_step = (uint16)PIXEL_OFFSET * SEARCH_IMAGE_W;
    bottom_ptr = image + (uint16)(SEARCH_IMAGE_H - 1u) * SEARCH_IMAGE_W;
    reflect_limit = reflect_point;
    white_min = white_min_point;
    white_max = white_max_point;
    contrast_ratio = reference_contrast_ratio;

    /* Reuse a reference that still reaches the upper detection area. */
    if (reference_ready) {
        globe_remote = SEARCH_IMAGE_H;
        row_ptr = bottom_ptr + reference_col;

        for (row = SEARCH_IMAGE_H - 1; row > PIXEL_OFFSET; row -= PIXEL_OFFSET) {
            temp1 = *row_ptr;
            temp2 = *(row_ptr - row_step);

            if (temp1 >= reflect_limit || temp2 >= reflect_limit) {
                row_ptr -= row_step;
                continue;
            }

            if (temp2 > white_max) {
                row_ptr -= row_step;
                continue;
            } else if (temp1 < white_min) {
                globe_remote = row;
                break;
            }

            if (contrast_over_threshold(temp1, temp2, contrast_ratio)) {
                globe_remote = row;
                break;
            }

            row_ptr -= row_step;
        }

        if (globe_remote == SEARCH_IMAGE_H ||
            globe_remote <= REFERENCE_REUSE_MAX_END_ROW) {
            return;
        }
    }

    globe_remote_min = SEARCH_IMAGE_H;
    reference_col = SEARCH_IMAGE_W / 2;
    for (col = REFERENCE_COL_MIN; col <= REFERENCE_COL_MAX; col += REFERENCE_COL_STEP) {
        globe_remote = SEARCH_IMAGE_H;
        row_ptr = bottom_ptr + col;

        for (row = SEARCH_IMAGE_H - 1; row > PIXEL_OFFSET; row -= PIXEL_OFFSET) {
            temp1 = *row_ptr;
            temp2 = *(row_ptr - row_step);

            if (temp1 >= reflect_limit || temp2 >= reflect_limit) {
                row_ptr -= row_step;
                continue;
            }

            if (temp2 > white_max) {
                row_ptr -= row_step;
                continue;
            } else if (temp1 < white_min) {
                globe_remote = row;
                break;
            }

            /* 119 起按 2 递减不会等于 STOP_ROW(8)，省去恒假的判断。 */
            if (contrast_over_threshold(temp1, temp2, contrast_ratio)) {
                globe_remote = row;
                break;
            }

            row_ptr -= row_step;
        }

        if (globe_remote < globe_remote_min) {
            globe_remote_min = globe_remote;
            reference_col = col;

            /* 第 3 行已是当前扫描序列的最小可能值，后续列不可能更优。 */
            if (globe_remote_min <= PIXEL_OFFSET + 1u) {
                break;
            }
        }
    }
    reference_ready = 1u;
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
    uint8 reflect_limit;
    uint8 white_min;
    uint8 white_max;
    uint8 contrast_ratio;
    uint8 col;
    uint8 row;
    uint16 row_step;

    row_max = SEARCH_IMAGE_H - 1;
    row_min = STOP_ROW;
    col_max = SEARCH_IMAGE_W - 1;
    col_min = 0;
    reflect_limit = reflect_point;
    white_min = white_min_point;
    white_max = white_max_point;
    contrast_ratio = reference_contrast_ratio;
    row_step = (uint16)PIXEL_OFFSET * SEARCH_IMAGE_W;
    p = image + (uint16)row_max * SEARCH_IMAGE_W;

    left_start_col  = reference_col;
    right_start_col = reference_col;
    left_end_col    = col_min;
    right_end_col   = col_max;

    left_stop = 0;
    right_stop = 0;
    lost_left = STOP_ROW;
    lost_right = STOP_ROW;

    memset(&left_edge_line[row_min], 0, row_max - row_min + 1u);
    memset(&right_edge_line[row_min], SEARCH_IMAGE_W - 1, row_max - row_min + 1u);

    for (row = row_max; row >= row_min; row -= PIXEL_OFFSET) {
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

                    if (temp1 >= reflect_limit || temp2 >= reflect_limit) {
                        continue;
                    }

                    if (temp1 < white_min &&
                        col == left_start_col &&
                        left_start_col == reference_col) {
                        left_stop = 1;
                        search_time = 0;

                        memset(&left_edge_line[2], col_min, row - 1u);
                        lost_left = row;
                        break;
                    }

                    if (temp1 < white_min) {
                        left_edge_line[row] = col;
                        break;
                    }

                    if (temp2 > white_max) {
                        continue;
                    }

                    if (col == col_min ||
                        contrast_over_threshold(temp1, temp2, contrast_ratio)) {
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
             * Keep the right search on the right side of both the reference
             * column and the detected left edge.  Without this lower bound,
             * a lost right edge can make the local window lock onto an
             * internal dark mark and follow it toward the upper-left corner.
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

                    if (temp1 >= reflect_limit || temp2 >= reflect_limit) {
                        continue;
                    }

                    if (temp1 < white_min &&
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

                    if (temp1 < white_min) {
                        /*
                         * A black point found only by the tracked local
                         * window is not trusted.  Retry once from the safe
                         * lower bound; accept it only if the full right-side
                         * search finds it again.
                         */
                        if (search_time == 0) {
                            right_edge_line[row] = col;
                        }
                        break;
                    }

                    if (temp2 > white_max) {
                        continue;
                    }

                    if (col >= col_max - PIXEL_OFFSET ||
                        contrast_over_threshold(temp1, temp2, contrast_ratio)) {
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
        p -= row_step;
    }

    insert_val();

    memcpy(left_control_line, left_edge_line, sizeof(left_edge_line));
    memcpy(right_control_line, right_edge_line, sizeof(right_edge_line));
}

/* 对按 PIXEL_OFFSET 搜到的稀疏边线做行间插值。 */
void insert_val(void)
{
    uint8 i;
    uint8 front_p;
    uint8 cur_p;
    uint16 sum;

#if PIXEL_OFFSET != 2
#error insert_val is specialized for PIXEL_OFFSET == 2
#endif

    front_p = left_edge_line[SEARCH_IMAGE_H - 1];
    for (i = SEARCH_IMAGE_H - PIXEL_OFFSET - 1; i >= STOP_ROW; i -= PIXEL_OFFSET) {
        cur_p = left_edge_line[i];
        sum = (uint16)front_p + (uint16)cur_p;
        if (cur_p >= front_p) {
            sum++;
        }
        left_edge_line[i + 1u] = (uint8)(sum >> 1);
        front_p = cur_p;

        if (i < STOP_ROW + PIXEL_OFFSET) {
            break;
        }
    }

    front_p = right_edge_line[SEARCH_IMAGE_H - 1];
    for (i = SEARCH_IMAGE_H - PIXEL_OFFSET - 1; i >= STOP_ROW; i -= PIXEL_OFFSET) {
        cur_p = right_edge_line[i];
        sum = (uint16)front_p + (uint16)cur_p;
        if (cur_p >= front_p) {
            sum++;
        }
        right_edge_line[i + 1u] = (uint8)(sum >> 1);
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

/* 单帧图像处理总入口：阈值、寻线、圆环、中线和目标识别都在这里更新。 */
void Image_OldStyle_Process(uint8 target_detect_enable,
                            uint8 visual_avoid_enable)
{
    const uint8 *img;
#if IMAGE_OTSU_ENABLE
    static uint8 otsu_frame_count = 0;
#endif
    static uint8 target_find_frame_count = 0;

    img = &mt9v03x_image[0][0];

#if IMAGE_OTSU_ENABLE
    if (!otsu_frame_count) {
        th = otsuThreshold((uint8 *)img);
    }
    otsu_frame_count++;
    if (otsu_frame_count >= OTSU_FRAME_DIV) {
        otsu_frame_count = 0;
    }
#else
    /* 固定曝光下直接使用底部参考区，取消直方图和 32 位除法峰值。 */
    th = 0;
#endif
    get_reference_point(img);
    Search_reference_col(img);
    Search_line(img);

    zebra_flag = 0;
    Ring(visual_avoid_enable);

    Fitted_Midline();

    /* 关闭检测或进入特殊路段时立即清掉旧目标，不能留到下一检测帧。 */
    tar_th = white_min_point;
    if (!target_detect_enable || current_step >= 2) {
        target_find_frame_count = 0;
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
        return;
    }

    /* 靶点结果在跳过帧保留，避免 15 ms 激光任务看到隔帧闪烁。 */
    if (!target_find_frame_count) {
        Pre_Scan();
        Target_find(pre_find_offset);
    }
    target_find_frame_count++;
    if (target_find_frame_count >= TARGET_FIND_FRAME_DIV) {
        target_find_frame_count = 0;
    }
}
