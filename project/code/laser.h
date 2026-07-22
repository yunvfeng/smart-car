#ifndef __LASER_H_
#define __LASER_H_

#include "zf_common_headfile.h"

extern uint16 tar_i;
extern uint8 tar_flag;
extern uint8 tar_x;
extern uint8 tar_y;
extern uint8 tar_th;
extern int16 center_offset;
extern uint8 aim_ready_flag;

extern uint8 l_d;
extern uint8 r_d;
extern uint8 top_d;
extern uint8 under_d;
extern uint8 debug_stage;
extern uint8 pre_find_flag;
extern int8 pre_find_offset;

/* 初始化激光 GPIO。 */
void laser_init(void);
/* 打开激光。 */
void laser_on(void);
/* 关闭激光。 */
void laser_off(void);
/* 安全抑制使能；使能时立即关闭激光并清空发射时序。 */
void Laser_Set_Inhibit(uint8 inhibit);
void Laser_Task(void);

/* 预扫描目标位置，得到横向偏移。 */
void Pre_Scan(void);
/* 根据预扫描偏移精找目标区域。 */
void Target_find(int8 mid_offset);

#endif
