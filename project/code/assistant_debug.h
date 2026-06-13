#ifndef __ASSISTANT_DEBUG_H_
#define __ASSISTANT_DEBUG_H_

#include "zf_common_headfile.h"

#ifndef ASSISTANT_DEBUG_ENABLE
#define ASSISTANT_DEBUG_ENABLE          1
#endif

#ifndef ASSISTANT_DEBUG_DEVICE
#define ASSISTANT_DEBUG_DEVICE          SEEKFREE_ASSISTANT_WIFI_SPI
#endif

#ifndef ASSISTANT_DEBUG_IMAGE_ENABLE
#define ASSISTANT_DEBUG_IMAGE_ENABLE    1
#endif

#ifndef ASSISTANT_DEBUG_SCOPE_ENABLE
#define ASSISTANT_DEBUG_SCOPE_ENABLE    1
#endif

#ifndef ASSISTANT_DEBUG_PARAM_ENABLE
#define ASSISTANT_DEBUG_PARAM_ENABLE    1
#endif

#ifndef ASSISTANT_DEBUG_IMAGE_DIV
#define ASSISTANT_DEBUG_IMAGE_DIV       1u
#endif

#ifndef ASSISTANT_DEBUG_SCOPE_DIV
#define ASSISTANT_DEBUG_SCOPE_DIV       1u
#endif

#ifndef ASSISTANT_DEBUG_BOUNDARY_ENABLE
#define ASSISTANT_DEBUG_BOUNDARY_ENABLE 1
#endif

#ifndef ASSISTANT_DEBUG_IMAGE_OVERLAY_ENABLE
#define ASSISTANT_DEBUG_IMAGE_OVERLAY_ENABLE 1
#endif

typedef enum
{
    ASSISTANT_PARAM_SERVO_KP = 1,
    ASSISTANT_PARAM_SERVO_KD,
    ASSISTANT_PARAM_SERVO_KF,
    ASSISTANT_PARAM_MOTOR_KP,
    ASSISTANT_PARAM_MOTOR_KI,
    ASSISTANT_PARAM_MOTOR_KD,
    ASSISTANT_PARAM_LINE_CONTRAST,
    ASSISTANT_PARAM_CAMERA_EXPOSURE
} assistant_param_channel_enum;

void Assistant_Debug_Init(void);
void Assistant_Debug_Task(void);
void Assistant_Debug_On_Frame(void);
void Assistant_Debug_Send_Now(void);
void Assistant_Debug_Timebase_Tick(void);
uint16 Assistant_Debug_Timebase_Now(void);
void Assistant_Debug_Set_Image_Process_Time(uint16 time_100us);

#endif
