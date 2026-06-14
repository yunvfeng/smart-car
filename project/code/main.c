#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
/* #include "zf_device_ips200.h" */
#include "wifi_assistant.h"
#include "assistant_debug.h"
#include "image.h"
#include "image2.h"
#include "ring.h"
#include "pid.h"
#include "motor.h"
#include "servo.h"
#include "laser.h"
#include "key.h"
/* #include "ips_ui.h" */
#include <stdio.h>


/*
 * 运行方式：
 * 1. 主循环等待摄像头完成一帧图像；
 * 2. 图像处理函数更新边线、中线、圆环状态和目标点；
 * 3. 15 ms 定时器根据最新中线控制舵机和电机。
 */

#define CONTROL_PERIOD_MS      15

/* 15 ms 定时任务：读取编码器、刷新舵机/电机输出，并按目标识别结果控制激光。 */
static void Timer0_Callback(void)
{
    /* 中断里只放轻量控制任务，避免影响下一帧图像采集。 */
    Encoder_GetValue();

    Servo_Loop();
    Motor_Loop();

    Laser_Task();
}

/* 程序入口：完成硬件初始化，之后在主循环中等待并处理摄像头图像。 */
void main(void)
{
    uint8 wifi_ready;
    uint8 assistant_debug_ready;
    uint8 wifi_assistant_started;
    uint8 car_started;

    clock_init(SYSTEM_CLOCK_96M);
    debug_init();

    Motor_Init();
    Encoder_Init();
    servo_init();
    laser_init();
    Key_Init();

    mt9v03x_init();
    mt9v03x_set_exposure_time(camera_exposure_time);
    /* ips200_init(); */
    assistant_debug_ready = 0;
    wifi_assistant_started = 0;
    car_started = 0;

    /* Control timer starts after KEY1/PB2 is pressed. */
		

    while (1) {
        if (!car_started && !gpio_get_level(KEY1_PIN)) {
            system_delay_ms(20);
            if (!gpio_get_level(KEY1_PIN)) {
                pit_ms_init(TIM0_PIT, CONTROL_PERIOD_MS, Timer0_Callback);
                car_started = 1;
            }
        }

        wifi_ready = 0;
        if (!gpio_get_level(SWITCH2_PIN)) {
            if (!wifi_assistant_started) {
                Wifi_Assistant_Init();
                assistant_debug_ready = 0;
                wifi_assistant_started = 1;
            }

            wifi_ready = Wifi_Assistant_Ready();
            if (wifi_ready) {
                if (!assistant_debug_ready) {
                    Assistant_Debug_Init();
                    assistant_debug_ready = 1;
                }
                Assistant_Debug_Task();
            }
        } else {
            assistant_debug_ready = 0;
            wifi_assistant_started = 0;
        }

        /* 摄像头完成一帧后再处理图像，处理完再等下一帧。 */
        if (mt9v03x_finish_flag) {
            mt9v03x_finish_flag = 0;

            Image_OldStyle_Process();

            if (wifi_ready) {
                Assistant_Debug_On_Frame();
            }

            /* 按下 SWITCH2 时显示调试画面。 */
            /*
            if (!gpio_get_level(SWITCH2_PIN)) {
                IPS_Show_Image_And_Line(0, 0);
            }
            */
        }
    }
}
