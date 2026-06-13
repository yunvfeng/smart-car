/*********************************************************************************************************************
* STC32G144K Opensourec Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-11-20        大W            first version
********************************************************************************************************************/


#ifndef __ZF_DRIVER_EXTI_H
#define __ZF_DRIVER_EXTI_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"


//此枚举定义不允许用户修改
typedef enum
{
	//此为8051传统外部中断
	INT0_P32 = 0x0100 | IO_P32,	    // 支持边沿，下降沿中断
	INT1_P33 = 0x0200 | IO_P33,		// 支持边沿，下降沿中断
	INT2_P36 = 0x0300 | IO_P36,		// 支持下降沿中断		
 	INT3_P37 = 0x0400 | IO_P37,		// 支持下降沿中断
	INT4_P30 = 0x0500 | IO_P30,		// 支持下降沿中断
	
	
	//此为GPIO外部中断，所有枚举的中断模式都支持
	GPIO_INIT_P00 = 0X1000 | IO_P00,
	GPIO_INIT_P01 = 0X1000 | IO_P01,
	GPIO_INIT_P02 = 0X1000 | IO_P02,
	GPIO_INIT_P03 = 0X1000 | IO_P03,
	GPIO_INIT_P04 = 0X1000 | IO_P04,
	GPIO_INIT_P05 = 0X1000 | IO_P05,
	GPIO_INIT_P06 = 0X1000 | IO_P06,
	GPIO_INIT_P07 = 0X1000 | IO_P07,

	GPIO_INIT_P10 = 0X1100 | IO_P10,
	GPIO_INIT_P11 = 0X1100 | IO_P11,
	GPIO_INIT_P12 = 0X1100 | IO_P12,
	GPIO_INIT_P13 = 0X1100 | IO_P13,
	GPIO_INIT_P14 = 0X1100 | IO_P14,
	GPIO_INIT_P15 = 0X1100 | IO_P15,
	GPIO_INIT_P16 = 0X1100 | IO_P16,
	GPIO_INIT_P17 = 0X1100 | IO_P17,

	GPIO_INIT_P20 = 0X1200 | IO_P20,
	GPIO_INIT_P21 = 0X1200 | IO_P21,
	GPIO_INIT_P22 = 0X1200 | IO_P22,
	GPIO_INIT_P23 = 0X1200 | IO_P23,
	GPIO_INIT_P24 = 0X1200 | IO_P24,
	GPIO_INIT_P25 = 0X1200 | IO_P25,
	GPIO_INIT_P26 = 0X1200 | IO_P26,
	GPIO_INIT_P27 = 0X1200 | IO_P27,

	GPIO_INIT_P30 = 0X1300 | IO_P30,
	GPIO_INIT_P31 = 0X1300 | IO_P31,
	GPIO_INIT_P32 = 0X1300 | IO_P32,
	GPIO_INIT_P33 = 0X1300 | IO_P33,
	GPIO_INIT_P34 = 0X1300 | IO_P34,
	GPIO_INIT_P35 = 0X1300 | IO_P35,
	GPIO_INIT_P36 = 0X1300 | IO_P36,
	GPIO_INIT_P37 = 0X1300 | IO_P37,

	GPIO_INIT_P40 = 0X1400 | IO_P40,
	GPIO_INIT_P41 = 0X1400 | IO_P41,
	GPIO_INIT_P42 = 0X1400 | IO_P42,
	GPIO_INIT_P43 = 0X1400 | IO_P43,
	GPIO_INIT_P44 = 0X1400 | IO_P44,
	GPIO_INIT_P45 = 0X1400 | IO_P45,
	GPIO_INIT_P46 = 0X1400 | IO_P46,
	GPIO_INIT_P47 = 0X1400 | IO_P47,

	GPIO_INIT_P50 = 0X1500 | IO_P50,
	GPIO_INIT_P51 = 0X1500 | IO_P51,
	GPIO_INIT_P52 = 0X1500 | IO_P52,
	GPIO_INIT_P53 = 0X1500 | IO_P53,
	GPIO_INIT_P54 = 0X1500 | IO_P54,
	GPIO_INIT_P55 = 0X1500 | IO_P55,
	GPIO_INIT_P56 = 0X1500 | IO_P56,
	GPIO_INIT_P57 = 0X1500 | IO_P57,

	GPIO_INIT_P60 = 0X1600 | IO_P60,
	GPIO_INIT_P61 = 0X1600 | IO_P61,
	GPIO_INIT_P62 = 0X1600 | IO_P62,
	GPIO_INIT_P63 = 0X1600 | IO_P63,
	GPIO_INIT_P64 = 0X1600 | IO_P64,
	GPIO_INIT_P65 = 0X1600 | IO_P65,
	GPIO_INIT_P66 = 0X1600 | IO_P66,
	GPIO_INIT_P67 = 0X1600 | IO_P67,

	GPIO_INIT_P70 = 0X1700 | IO_P70,
	GPIO_INIT_P71 = 0X1700 | IO_P71,
	GPIO_INIT_P72 = 0X1700 | IO_P72,
	GPIO_INIT_P73 = 0X1700 | IO_P73,
	GPIO_INIT_P74 = 0X1700 | IO_P74,
	GPIO_INIT_P75 = 0X1700 | IO_P75,
	GPIO_INIT_P76 = 0X1700 | IO_P76,
	GPIO_INIT_P77 = 0X1700 | IO_P77,

	GPIO_INIT_P80 = 0X1800 | IO_P80,
	GPIO_INIT_P81 = 0X1800 | IO_P81,
	GPIO_INIT_P82 = 0X1800 | IO_P82,
	GPIO_INIT_P83 = 0X1800 | IO_P83,
	GPIO_INIT_P84 = 0X1800 | IO_P84,
	GPIO_INIT_P85 = 0X1800 | IO_P85,
	GPIO_INIT_P86 = 0X1800 | IO_P86,
	GPIO_INIT_P87 = 0X1800 | IO_P87,

	GPIO_INIT_P90 = 0X1900 | IO_P90,
	GPIO_INIT_P91 = 0X1900 | IO_P91,
	GPIO_INIT_P92 = 0X1900 | IO_P92,
	GPIO_INIT_P93 = 0X1900 | IO_P93,
	GPIO_INIT_P94 = 0X1900 | IO_P94,
	GPIO_INIT_P95 = 0X1900 | IO_P95,
	GPIO_INIT_P96 = 0X1900 | IO_P96,
	GPIO_INIT_P97 = 0X1900 | IO_P97,

	GPIO_INIT_PA0 = 0X1A00 | IO_PA0,
	GPIO_INIT_PA1 = 0X1A00 | IO_PA1,
	GPIO_INIT_PA2 = 0X1A00 | IO_PA2,
	GPIO_INIT_PA3 = 0X1A00 | IO_PA3,
	GPIO_INIT_PA4 = 0X1A00 | IO_PA4,
	GPIO_INIT_PA5 = 0X1A00 | IO_PA5,
	GPIO_INIT_PA6 = 0X1A00 | IO_PA6,
	GPIO_INIT_PA7 = 0X1A00 | IO_PA7,

	GPIO_INIT_PB0 = 0X1B00 | IO_PB0,
	GPIO_INIT_PB1 = 0X1B00 | IO_PB1,
	GPIO_INIT_PB2 = 0X1B00 | IO_PB2,
	GPIO_INIT_PB3 = 0X1B00 | IO_PB3,
	GPIO_INIT_PB4 = 0X1B00 | IO_PB4,
	GPIO_INIT_PB5 = 0X1B00 | IO_PB5,
	GPIO_INIT_PB6 = 0X1B00 | IO_PB6,
	GPIO_INIT_PB7 = 0X1B00 | IO_PB7,
	
}INTN_enum;

#define INT0_CLEAR_FLAG (TCON &= (~(0x01 << 1)))		// 外部中断 0 中断请求标志。中断服务程序中，硬件自动清零。
#define INT1_CLEAR_FLAG (TCON &= (~(0x01 << 3)))		// 外部中断 1 中断请求标志。中断服务程序中，硬件自动清零。
                                                           
#define INT2_CLEAR_FLAG (AUXINTIF &= (~(0x10 << 0)))	// 外部中断 2 中断请求标志。中断服务程序中，硬件自动清零。
#define INT3_CLEAR_FLAG (AUXINTIF &= (~(0x10 << 1)))	// 外部中断 3 中断请求标志。中断服务程序中，硬件自动清零。
#define INT4_CLEAR_FLAG (AUXINTIF &= (~(0x10 << 2)))	// 外部中断 4 中断请求标志。中断服务程序中，硬件自动清零。

//其他的GPIO中断标准默认在中断里面清除，用户不用管

typedef enum    	
{
    BOTH = 0,			//边沿
    
	FALLING_EDGE,	    //下降沿
	
	RISING_EDGE,	    //上升沿
	
	LOW_LEVEL,		    //低电平
	
	HIGH_LEVEL,		    //高电平

}INT_MODE_enum;



extern callback_function int_irq_handlers[5];

extern callback_function gpio_int_irq_handlers[12][8];

void exit_init(INTN_enum int_n,INT_MODE_enum mode);


#endif