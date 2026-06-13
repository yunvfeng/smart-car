#ifndef __KEY_H_
#define __KEY_H_

#include "zf_common_headfile.h"

#define KEY1_PIN        IO_PB2
#define KEY2_PIN        IO_PB3
#define KEY3_PIN        IO_PB4
#define KEY4_PIN        IO_P32

#define SWITCH1_PIN     IO_PB0
#define SWITCH2_PIN     IO_PB1

extern uint8 key1_flag;
extern uint8 key2_flag;
extern uint8 key3_flag;
extern uint8 key4_flag;

/* 初始化按键和拨码开关输入。 */
void Key_Init(void);
/* 读取按键状态并更新按键标志。 */
void Key_GetNum(void);
/* 处理按键标志，后续功能可在这里扩展。 */
void Key_Anlysis(void);

#endif
