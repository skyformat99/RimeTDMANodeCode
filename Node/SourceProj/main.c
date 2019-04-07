/*
*********************************************************************************************************
*                                        IAR Embedded Workbench for STMicroelectronics STM8 IED
*                                            MCU=STM8L151C8T6D RF=Sx1278 OS=Contiki V2.7
*
* Filename   	: main.h
* Programmer : JiangJun
* Description	: Entry of the C program
* Date           : 2015-05-27
* Copyright    : (c) RimeLink (www.rimelink.com)
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             				INCLUDE FILES
*********************************************************************************************************
*/
#include <string.h>
#include <stdio.h>
#include "contiki.h"
#include "netstack.h"
#include "ctimer.h"
#include "abc.h"
#include "main.h"
#include "Dbg.h"
#include "Chip.h"
#include "Comm2Trm.h"
#include "Network.h"
#include "SystSettings.h"
#include "sx1278_ports.h"
#include "PwrManage.h"
#include "RTC.h"
#include "CommPC.h"
#if (!REL_VER)
#include "Monitor.h"
#include "DebugPort.h"
#endif

/*
*********************************************************************************************************
*                                            			        DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            				  FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             				LOCAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             			      GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              					main()
* Description: This is the 'standard' C startup entry point.
* Arguments: None
* Returns    : main() should NEVER return
* Note(s)    : None
*********************************************************************************************************
*/
void main(void)
{
    /* Initialize hardware */
	/*
	设置STM8L151C8的系统时钟为16MHz，把所有引脚设置为低功耗模式
	初始化两个LED灯用到的引脚，控制LED灯闪烁表示节点已经启动
	设置LSE作为RTC的时钟源，设置LSI作为IWDG的时钟源
	读取STM8L151C8的唯一ID，设置在停机模式时独立看门狗（IWDG)停止运行
    超低功耗模式使能和快速唤醒使能
	*/
    chip_Init();
    /*
    初始化STM8L151上和SX1278进行通信的SPI1，通信速率为8MHz，软件片选，PB4为片选引脚
    设置连接DIO0的引脚PD6为外部中断，上升沿触发
    不使能发射和接收，设RF为运行模式
    */
    SX1278InitPins();
    /*
    设置定时器TIM5，每10ms中断一次
    用来喂狗WWDG
    */
    clock_init();
    /*
    初始化TIM1,设置为每255*65536ms溢出一次
    */
    rtimer_init();
    /*
    系统初始化配置，设置帧结构并把帧结构保存到EEPROM中
    */
    ss_Init();
    /*
    RTC初始化，设置LSE(32.768KHz)为其时钟源,32768分频，因此计数频率为1Hz
    */
    rtc_Init();
    /*
    设置USARTx和DMA
    */
    cpc_Init(); /* Do NOT masked! need this to send frame even in DEBUG mode. */
#if (!REL_VER)
    //设置调试用的USARTx和DMA
    dp_Init();
#endif

    /* EXPLAIN: enable INT before initialize process! */
    HAL_ENABLE_INTERRUPTS();

    /* Initialize components */
    pm_Init();
	
    /* Initialize contiki system */	
    energest_init();
    process_init();
    process_start(&etimer_process, NULL);

    /* Start process of user */
    comm2trm_Init();
    /*
    初始化SX1278,设置使用TCXO晶振作为SX1278的时钟源，将SX1278切换成LoRa模式
    设置PA_BOOST引脚作为射频信号输出引脚
    设置为连续接收模式
    */
    network_Init();
#if (!REL_VER)
    monitor_Init();
#endif

    while (1) 
    {
        while (process_run() > 0)
        {
            null();
        }

    #if (REL_VER)
        /* EXPLAIN: Set MCU to low power mode if have nothing to do. */
        pm_EnterLowPwr();
        pm_ExitLowPwr();
    #endif
    }
}


/*--------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/

