#include "zf_common_headfile.h"
#include "param.h"
#include "image.h"
#include "control.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"
#include "wifi_app.h"
#include "ips_ui.h"
#include <stdlib.h>
#include "zf_driver_gpio.h"



uint8 timer_task_flag = 0;

// 15 ms 控制周期：先刷新轮速，再执行舵机和电机闭环。
void Timer0_15ms_Callback(void) {
    Get_Speed(&g_State.speed_L, &g_State.speed_R);
    Steer_Control_Task();
    Motor_Control_Task();
}

void main(void)
{
    // 系统时钟与调试串口初始化。
    clock_init(SYSTEM_CLOCK_96M);       // 设置系统主频为 96MHz
    debug_init();                       // 初始化调试串口 (用于 printf)
    
    // EEPROM 与运行参数初始化。
    iap_init();                         // 初始化 EEPROM 模块

    Params_Load();                      // 从 EEPROM 加载参数，若初次开机则载入默认值

    // 执行机构初始化。
    My_Motor_Init();                       
    My_Servo_Init();                       
    
    // 传感器与显示屏初始化。
    My_Encoder_Init();                  // 左右编码器初始化
    mt9v03x_init();                     // 摄像头初始化
    ips200_init();

    // 通信调参入口默认关闭，需要时取消注释。
    //My_WiFi_Init();

    // 定时控制中断负责固定周期闭环。
    pit_ms_init(TIM0_PIT, 15, Timer0_15ms_Callback);

    // 本次调试覆盖值；最终参数可通过 EEPROM 保存。
    g_Params.Kp_Steer          = 7500;
    g_Params.Kd_Steer          = 1000;
    g_Params.Kf_Curve          = 700;
    g_Params.target_speed = 150;

    while(1)
    {
        //WiFi_Command_Handler();

        // 摄像头 DMA 完成一帧后，刷新视觉结果和屏幕显示。
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0; // 尽早清除标志位，允许底层接收下一帧。

            // 图像管线会更新 g_State.center_offset 与 g_State.curvature。
            Image_Process_Integrated();

            // 将灰度图、边线和中线显示到屏幕左上角。
            IPS_Show_Image_And_Line(0, 0);
				
        }
    }
}
