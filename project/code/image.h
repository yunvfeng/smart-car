#ifndef __IMAGE_H_
#define __IMAGE_H_

#include "zf_common_headfile.h"

/* 图像处理直接使用 MT9V03X 的 188x120 原始灰度图。 */
#define SEARCH_IMAGE_W      188
#define SEARCH_IMAGE_H      120

#define BALACK_POINT        30
#define WHITE_MAX_MUL       13
#define WHITE_MIN_MUL       7

#define REFRENCE_ROW        4
#define SEARCH_RANGE        16
#define STOP_ROW            8
#define PIXEL_OFFSET        2
#ifndef controlReferenceLine
#define controlReferenceLine 80
#endif

#ifndef Mid_Col
#define Mid_Col 94
#endif

extern uint8 reference_point;
extern uint8 white_max_point;
extern uint8 white_min_point;

extern uint8 refenence_col_line[SEARCH_IMAGE_H];
extern uint8 reference_contrast_ratio;
extern uint8 reference_col;

extern uint8 left_edge_line[SEARCH_IMAGE_H];
extern uint8 right_edge_line[SEARCH_IMAGE_H];
extern uint8 mid_line[SEARCH_IMAGE_H];

extern uint8 left_control_line[SEARCH_IMAGE_H];
extern uint8 right_control_line[SEARCH_IMAGE_H];

extern uint8 lost_left;
extern uint8 lost_right;

extern uint8 cross_flag;
extern uint16 encoder_enter;

extern uint8 far image_copy[MT9V03X_H][MT9V03X_W];

extern uint8 th;
extern int32 err_sum;
extern uint16 camera_exposure_time;
extern uint8 camera_init_brightness;

/* 计算两个灰度值的对比度，用于判断边缘是否明显。 */
uint16 get_contrast(uint8 temp1, uint8 temp2);

/* 从图像底部参考区域估算黑白阈值。 */
void get_reference_point(const uint8 *image);
/* 寻找一条稳定的参考列，作为左右寻线起点。 */
void Search_reference_col(const uint8 *image);
/* 从参考列开始搜索左右边线。 */
void Search_line(const uint8 *image);
/* 根据左右控制线生成中线。 */
void Fitted_Midline(void);
/* 对间隔搜索得到的边线做插值补齐。 */
void insert_val(void);

/* 根据边线连续性切换直道和转弯舵机参数。 */
void straightAccelerate(void);
/* 计算中线整体偏移，供舵机和速度规划使用。 */
void Error_sum(void);

/* 自动调整摄像头曝光，让初始画面亮度更稳定。 */
void Camera_Auto_Exposure_Init(void);
/* 每来一帧图像，就调用一次这个入口完成整套图像处理。 */
void Image_OldStyle_Process(void);

#endif
