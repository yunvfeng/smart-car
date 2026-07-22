#ifndef __RING_H_
#define __RING_H_

#include "zf_common_headfile.h"
#include "image.h"

extern uint8 ring_preMeet_flag;
extern uint8 first_meeting_flag;
extern uint8 ring_l;
extern uint8 ring_r;
extern uint8 ring_enter_flag;
extern uint8 ring_turn_flag;
extern uint8 Out_flag;
extern uint8 Straighten_flag;
extern uint8 ring_over_flag;
extern volatile uint8 current_step;

extern uint8 key_anlysis1;
extern uint8 key_anlysis2;
extern uint8 key_anlysis3;

/* 圆环状态机入口，每帧调用一次。 */
void Ring(uint8 visual_avoid_enable);
/* 在控制线之间补一条斜线。 */
void draw_line(uint8 left_point, uint8 right_point, uint8 to_flag);
/* 连接同一侧控制线上的上下两个点。 */
void connect_point(uint8 under, uint8 top, uint8 l_r);

/* 判断是否可能刚遇到圆环。 */
uint8 Ring_Pre_Meet(void);
/* 只返回圆环预判结果，不修改主状态。 */
uint8 Ring_Pre_Meet_use(void);
/* 处理第一次遇到圆环边线变化的阶段。 */
void Ring_First_meeting(void);
/* 处理进入圆环阶段。 */
void Ring_Enter(void);
/* 处理圆环转向阶段。 */
void Ring_Turing(void);
/* 处理车身位于圆环内部的阶段。 */
void Ring_Ring_Ring(void);
/* 处理出圆环阶段。 */
void Ring_Out(void);
/* 处理出圆环后的回正阶段。 */
void Ring_Straighten(void);
/* 清空圆环状态，回到普通寻线。 */
void Ring_Over(void);

/* 判断边线是否连续。 */
uint8 isContinueLine(uint8 *arr);
/* 限制列坐标不越过图像宽度。 */
uint8 clamp_col_int16(int16 val);

#endif
