#ifndef __LASER_H_
#define __LASER_H_

#include "zf_common_headfile.h"

#define TARGET_LASER_NONE    0u
#define TARGET_LASER_LEFT    1u
#define TARGET_LASER_CENTER  2u
#define TARGET_LASER_RIGHT   3u

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
/* 安全抑制使能；使能时立即关闭激光并清空发射时序。 */
void Laser_Set_Inhibit(uint8 inhibit);
void Laser_Task(void);

/* 清空凹陷检测、单次发射锁存和待发请求。 */
void Target_Notch_Reset(void);
/* 每帧只读取 61～79 奇数行的左右边线，检测凹陷并发布单次请求。 */
void Target_Notch_ProcessFrame(void);

#endif
