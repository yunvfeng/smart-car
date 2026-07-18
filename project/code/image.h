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
extern uint8 zebra_flag;
extern uint16 encoder_enter;

extern uint8 th;
extern uint16 camera_exposure_time;
extern uint8 camera_init_brightness;

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

/* 自动调整摄像头曝光，让初始画面亮度更稳定。 */
void Camera_Auto_Exposure_Init(void);
/* 每帧完成寻线；main.c 传入靶点/视觉避障运行门控。 */
void Image_OldStyle_Process(uint8 target_detect_enable,
                            uint8 inhibit_ring);

#endif
