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

#include "zf_driver_exti.h"
#include "zf_common_debug.h"

callback_function int_irq_handlers[5] = {0};

callback_function gpio_int_irq_handlers[12][8] = {0};

//-------------------------------------------------------------------------------------------------------------------
//  @brief      外部中断初始化
//  @param      NULL            	
//  @return     void
//  Sample usage:               exit_init(INT0_P32,BOTH) //初始化P32 作为外部中断引脚，双边沿触发。
//-------------------------------------------------------------------------------------------------------------------
void exit_init(INTN_enum int_n,INT_MODE_enum mode)
{
	if(INT0_P32 == int_n)
	{
        switch(mode)
		{
            case BOTH: 
                IT0 = 0;
                break;
            
			case FALLING_EDGE: 
                IT0 = 1;
				break;
            
			default:
                zf_assert(0);
				break;
		}
		EX0 = 1; 		    //使能INT0中断
	}
	else if(INT1_P33 == int_n)
	{
        switch(mode)
		{
            case BOTH: 
                IT1 = 0;
                break;
			case FALLING_EDGE: 
                IT1 = 1;
				break;
			default:
                zf_assert(0);
				break;
		}
		EX1 = 1; 		    //使能INT1中断
	}
	else if(INT2_P36 == int_n)
	{
        switch(mode)
		{
            case BOTH: 
                IT1 = 0;
                break;
			case FALLING_EDGE: 
                IT1 = 1;
				break;
			default:
                zf_assert(0);
				break;
		}
		INTCLKO |= 1<<4;	//使能INT2中断
	}
	else if(INT3_P37 == int_n)
	{
        zf_assert(mode == FALLING_EDGE);
		INTCLKO |= 1<<5;	//使能INT3中断
	}
	else if(INT4_P30 == int_n)
	{
        zf_assert(mode == FALLING_EDGE);
		INTCLKO |= 1<<6;	//使能INT4中断
	}
	else
	{
		(*(unsigned char volatile far *)(0X7EFD00 + ((int_n >> 8) & 0X0F) )) |= (1 << (int_n & 0x000F));//开启中断功能
		
		switch(mode)//初始化中断模式
		{
			case FALLING_EDGE: 
					(*(unsigned char volatile far *)(0X7EF4E0 + ((int_n >> 8) & 0X0F) )) &= ~(1 << (int_n & 0x000F));
					(*(unsigned char volatile far *)(0X7EFD30 + ((int_n >> 8) & 0X0F) )) &= ~(1 << (int_n & 0x000F));
					(*(unsigned char volatile far *)(0X7EFD20 + ((int_n >> 8) & 0X0F) )) &= ~(1 << (int_n & 0x000F));
				break;
			case RISING_EDGE: 
					(*(unsigned char volatile far *)(0X7EF4E0 + ((int_n >> 8) & 0X0F) )) &= ~(1 << (int_n & 0x000F));
					(*(unsigned char volatile far *)(0X7EFD30 + ((int_n >> 8) & 0X0F) )) &= ~(1 << (int_n & 0x000F));
					(*(unsigned char volatile far *)(0X7EFD20 + ((int_n >> 8) & 0X0F) )) |= (1 << (int_n & 0x000F));
				break;
			case LOW_LEVEL: 
					(*(unsigned char volatile far *)(0X7EF4E0 + ((int_n >> 8) & 0X0F) )) &= ~(1 << (int_n & 0x000F));
					(*(unsigned char volatile far *)(0X7EFD30 + ((int_n >> 8) & 0X0F) )) |= (1 << (int_n & 0x000F));
					(*(unsigned char volatile far *)(0X7EFD20 + ((int_n >> 8) & 0X0F) )) &= ~(1 << (int_n & 0x000F));
				break;
			case HIGH_LEVEL: 
					(*(unsigned char volatile far *)(0X7EF4E0 + ((int_n >> 8) & 0X0F) )) &= ~(1 << (int_n & 0x000F));
					(*(unsigned char volatile far *)(0X7EFD30 + ((int_n >> 8) & 0X0F) )) |= (1 << (int_n & 0x000F));
					(*(unsigned char volatile far *)(0X7EFD20 + ((int_n >> 8) & 0X0F) )) |= (1 << (int_n & 0x000F));
				break;
			case BOTH: 
					(*(unsigned char volatile far *)(0X7EF4E0 + ((int_n >> 8) & 0X0F) )) |= (1 << (int_n & 0x000F));
					(*(unsigned char volatile far *)(0X7EFD30 + ((int_n >> 8) & 0X0F) )) &= ~(1 << (int_n & 0x000F));
					(*(unsigned char volatile far *)(0X7EFD20 + ((int_n >> 8) & 0X0F) )) &= ~(1 << (int_n & 0x000F));
				break;
		}
	}
}







