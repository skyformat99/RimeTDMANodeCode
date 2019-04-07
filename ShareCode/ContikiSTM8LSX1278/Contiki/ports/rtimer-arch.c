/**
* \file
*			Real-timer specific implementation for STM8L151C8.
* \author
*			JiangJun <jiangjunjie_2005@126.com>
*/

#include "sys/energest.h"
#include "sys/rtimer.h"
#include "rtimer-arch.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_tim1.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
/**
 * Handle interrupt of real time timer.
 *
 * This function was invoked when timer of real time is expired.
 *
 */
void
rtimer_irq_handler(void)
{
    /* Clear interrupt pending bit */
    TIM1_ClearITPendingBit(TIM1_IT_CC1);
  
    /* Do interrupt only once! */
    TIM1_ITConfig(TIM1_IT_CC1, DISABLE);

    ENERGEST_ON(ENERGEST_TYPE_IRQ);
    rtimer_run_next();
    ENERGEST_OFF(ENERGEST_TYPE_IRQ);

    return;	
}


/*---------------------------------------------------------------------------*/
/**
 * Initialize timer for real time timer.
 *
 * This function would initialize TIM1 for real time timer.
 *
 */
void rtimer_arch_init(void)
{
    /* TIM1 configuration:
         - TIM1CLK is set to 16 MHz, set the TIM1 frequency to 10kHz by config prescaler
         - With 1000 Hz we can generate time base:
        max time base is 65535ms if TIM1_PERIOD = 65535 --> (65535 + 1) / 1000 = 65535ms
        min time base is 2 ms if TIM1_PERIOD = 1   --> (1 + 1) / 1000 = 2ms */
    #define TIM1_PRESCALER    (16000000 / RTIMER_ARCH_SECOND - 1)
	
    /* Enable TIM1 CLK */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, ENABLE);
    //每255*65535ms中断一次
    TIM1_TimeBaseInit(TIM1_PRESCALER, TIM1_CounterMode_Up, 0xFFFF, 0xFF);

    /* Enable TIM1 */
    TIM1_Cmd(ENABLE);

    return;
}

/**
 * \brief    Get the current clock time
 * \return  The current time
 *
 *            This function returns what the real-time module thinks
 *            is the current time. The current time is used to set
 *            the timeouts for real-time tasks.
 *
 * \hideinitializer
 */
rtimer_clock_t
rtimer_arch_now(void)
{
    rtimer_clock_t    tT1, tT2;

    do
    {
        /* Avoid race condition on reading counter of TIM1 */
        tT1 = TIM1_GetCounter();
        tT2 = TIM1_GetCounter();
    } while (tT1 != tT2);

    return tT1;
}

/**
 * \brief      Set an interrupt to timeout event.
 * \param    rtimer_clock_t t    the quantity of timeout, unit is ms.
 *
 *              This function schedules a real-time task at a specified
 *              time in the future.
 */
void
rtimer_arch_schedule(rtimer_clock_t t)
{
    /* Sets the TIM1 Capture Compare1 Register value */
    TIM1_SetCompare1(t);

    /* MUST clear the remained flag of TIM1 compare */
    TIM1_ClearFlag(TIM1_FLAG_CC1);

    /* Enable interrupt of Capture compare 1 */
    TIM1_ITConfig(TIM1_IT_CC1, ENABLE);

    return;
}

/**
 * \brief    Disable interrupt of rtimer timeout.
 *
 *            This function disable interrupt of real timer for removing timer.
 */
void
rtimer_arch_disable_irq(void)
{  
    TIM1_ITConfig(TIM1_IT_CC1, DISABLE);

    return;
}

/**
 * \brief    Enable interrupt of rtimer timeout.
 *
 *            This function enable interrupt of real timer for restarting timer.
 */
void
rtimer_arch_enable_irq(void)
{  
    /* MUST clear the remained flag of TIM1 compare */
    TIM1_ClearFlag(TIM1_FLAG_CC1);

    /* Enable interrupt of Capture compare 1 */
    TIM1_ITConfig(TIM1_IT_CC1, ENABLE);

    return;
}

/**
 * \brief      turn off the real timer.
 * \param    none
 *
 *              This function would turn off the real timer for saved energy.
 */
void
rtimer_arch_TurnOff(void)
{
    /* Disable TIM1 */
    TIM1_Cmd(DISABLE);

    /* Disable TIM1 CLK */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, DISABLE);

    return;
}

/**
 * \brief      turn on the real timer.
 * \param    none
 *
 *              This function would turn on the real timer.
 */
void
rtimer_arch_TurnOn(void)
{
    /* Enable TIM1 CLK */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, ENABLE);

    /* Enable TIM1 */
    TIM1_Cmd(ENABLE);

    return;
}


/*--------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/

