#ifndef __IMAGE2_H_
#define __IMAGE2_H_

#include "zf_common_headfile.h"

#define GrayScale         256
#define OTSU_USE_LIMIT    0
#define OTSU_LIMIT_MIN    20
#define OTSU_LIMIT_MAX    175
#define OTSU_START_ROW    16
#define OTSU_ROW_STEP     2
#define OTSU_COL_STEP     2
#define OTSU_CLIP_PERCENT 2
#define OTSU_MIN_RANGE    24
#define OTSU_MAX_STEP     16
#define OTSU_REFLECT_POINT 215

#define threshold_max     (255 * 5)
#define threshold_min     (255 * 2)

extern uint8 Threshold;
extern uint8 far base_image[MT9V03X_H][MT9V03X_W];

/* 根据当前灰度图计算 Otsu 阈值。 */
uint8 otsuThreshold(uint8 *mt9v03x_image_first);
/* 按阈值把 base_image 转成黑白图。 */
void base_image_threshold(uint8 Threshold);

#endif
