#ifndef __WIFI_ASSISTANT_H_
#define __WIFI_ASSISTANT_H_

#include "zf_common_headfile.h"

typedef enum
{
    WIFI_ASSISTANT_IDLE = 0,
    WIFI_ASSISTANT_WIFI_FAIL,
    WIFI_ASSISTANT_SOCKET_FAIL,
    WIFI_ASSISTANT_READY
} wifi_assistant_status_enum;

void Wifi_Assistant_Init(void);
uint8 Wifi_Assistant_Ready(void);
wifi_assistant_status_enum Wifi_Assistant_Status(void);

#endif
