#include "image2.h"

uint8 far base_image[MT9V03X_H][MT9V03X_W];
uint8 Threshold = 128;
static uint16 xdata s_otsu_hist[GrayScale];

/* 根据当前灰度图计算 Otsu 阈值，过滤过亮反光点后再统计直方图。 */
uint8 otsuThreshold(uint8 *mt9v03x_image_first)
{
    uint16 width;
    uint16 height;
    uint32 total;
    uint32 valid_total;
    uint32 sum_all;
    uint32 sum_bg;
    uint32 weight_bg;
    uint32 weight_fg;
    uint32 score;
    uint32 best_score;
    int16 diff;
    int16 mean_bg;
    int16 mean_fg;
    uint8 threshold;
    uint8 pixel_max;
    uint8 pixel_min;
    uint8 clip_min;
    uint8 clip_max;
    uint8 pix;
    uint16 clip_count;
    uint32 acc;
    uint16 i;
    uint16 j;
    uint8 *img_ptr;
#if OTSU_USE_LIMIT
    int16 threshold_delta;
    static uint8 last_threshold = 128;
    static uint8 threshold_ready = 0;
#endif

    width = MT9V03X_W;
    height = MT9V03X_H;
    total = 0;
    pixel_max = 0;
    pixel_min = 255;
    threshold = 128;

    for (i = 0; i < GrayScale; i++) {
        s_otsu_hist[i] = 0;
    }

    for (i = OTSU_START_ROW; i < height; i += OTSU_ROW_STEP) {
        img_ptr = mt9v03x_image_first + (uint32)i * width;

        for (j = 0; j < width; j += OTSU_COL_STEP) {
            pix = *(img_ptr + j);
            if (pix >= OTSU_REFLECT_POINT) {
                continue;
            }

            s_otsu_hist[pix]++;
            total++;
            if (pix > pixel_max) pixel_max = pix;
            if (pix < pixel_min) pixel_min = pix;
        }
    }

    if (!total) {
        return Threshold;
    }

    if ((uint16)pixel_max <= (uint16)pixel_min + OTSU_MIN_RANGE) {
        threshold = (uint8)(((uint16)pixel_min + pixel_max) >> 1);
    } else {
        clip_count = (uint16)(total * OTSU_CLIP_PERCENT / 100u);
        if (!clip_count) {
            clip_count = 1;
        }

        acc = 0;
        clip_min = pixel_min;
        for (i = pixel_min; i <= pixel_max; i++) {
            acc += s_otsu_hist[i];
            if (acc >= clip_count) {
                clip_min = (uint8)i;
                break;
            }
        }

        acc = 0;
        clip_max = pixel_max;
        for (i = pixel_max; i >= pixel_min; i--) {
            acc += s_otsu_hist[i];
            if (acc >= clip_count) {
                clip_max = (uint8)i;
                break;
            }
            if (!i) {
                break;
            }
        }

        if ((uint16)clip_max <= (uint16)clip_min + OTSU_MIN_RANGE) {
            threshold = (uint8)(((uint16)clip_min + clip_max) >> 1);
        } else {
            valid_total = 0;
            sum_all = 0;

            for (i = clip_min; i <= clip_max; i++) {
                valid_total += s_otsu_hist[i];
                sum_all += (uint32)i * (uint32)s_otsu_hist[i];
            }

            weight_bg = 0;
            sum_bg = 0;
            best_score = 0;

            for (i = clip_min; i <= clip_max; i++) {
                weight_bg += s_otsu_hist[i];
                sum_bg += (uint32)i * (uint32)s_otsu_hist[i];

                if (weight_bg == 0) {
                    continue;
                }

                weight_fg = valid_total - weight_bg;
                if (weight_fg == 0) {
                    break;
                }

                mean_bg = (int16)(sum_bg / weight_bg);
                mean_fg = (int16)((sum_all - sum_bg) / weight_fg);
                diff = mean_bg - mean_fg;
                if (diff < 0) diff = -diff;

                score = ((weight_bg * weight_fg) >> 12) * (uint32)diff * (uint32)diff;

                if (score > best_score) {
                    best_score = score;
                    threshold = (uint8)i;
                }
            }
        }
    }

#if OTSU_USE_LIMIT
    if (threshold < OTSU_LIMIT_MIN) {
        threshold = OTSU_LIMIT_MIN;
    } else if (threshold > OTSU_LIMIT_MAX) {
        threshold = OTSU_LIMIT_MAX;
    }

    if (threshold_ready) {
        threshold_delta = (int16)threshold - (int16)last_threshold;
        if (threshold_delta > OTSU_MAX_STEP) {
            threshold = last_threshold + OTSU_MAX_STEP;
        } else if (threshold_delta < -OTSU_MAX_STEP) {
            threshold = last_threshold - OTSU_MAX_STEP;
        }
    }

    last_threshold = threshold;
    threshold_ready = 1;
#endif

    Threshold = threshold;
    return threshold;
}

/* 按指定阈值把 base_image 转成黑白图，主要用于调试或备用处理。 */
void base_image_threshold(uint8 threshold)
{
    int i;
    int j;
    uint8 th_use;

    if (threshold > 8) {
        th_use = threshold - 8;
    } else {
        th_use = threshold;
    }

    for (i = 0; i < MT9V03X_H; i++) {
        for (j = 0; j < MT9V03X_W; j++) {
            base_image[i][j] = (base_image[i][j] < th_use) ? 0 : 255;
        }
    }
}
