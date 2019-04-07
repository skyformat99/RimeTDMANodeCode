#include <sys/clock.h>
#include <sys/cc.h>
#include <sys/etimer.h>
#include "stm8l15x_tim5.h"
#include "Wdg.h"
#include "Chip.h"

/* EXPLAIN: prevent race condition since variables shared by process and ISR of SysTick */
static volatile clock_time_t    current_clock = 0;
static volatile unsigned long    current_seconds = 0;

static unsigned int    second_countdown = CLOCK_SECOND;

/*---------------------------------------------------------------------------*/
/**
 * Handle interrupt of system tick.
 *
 * This function was invoked when timer of system tick is expired.
 *
 */
void
SysTick_handler(void)
{
    /* Clear interrupt pending bit */
    TIM5_ClearITPendingBit(TIM5_IT_Update);
  
    ++current_clock;

    /* Check whether some event timer was expired */  
    if (etimer_pending() && etimer_next_expiration_time() <= current_clock) 
    {
        etimer_request_poll();
    }
  
    if (--second_countdown == 0) 
    {
        ++current_seconds;
        second_countdown = CLOCK_SECOND;
    }

    /* EXPLAIN: feed watch-dog in ISR to prevent overflow since that some operation 
        like as write of EEPROM would consume too long time */
    if (0 == current_clock % (CLOCK_SECOND / 2)) /* 500ms expired */
    {
        wdg_Feed();
    }

    return;	
}

/*---------------------------------------------------------------------------*/
/**
 * Initialize timer for system tick.
 *
 * This function would initialize TIM5 for systerm tick.
 *
 */
void clock_init(void)
{
    /* Counter clock=125000Hz, Interval=(1/CLOCK_SECOND)S, Period=Interval * 125000 */
    #define TIM5_PERIOD    (125000 / CLOCK_SECOND - 1) /* Subtract 1 for register configuration */

    /* Enable TIM5 CLK */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM5, ENABLE);
    /* TIM5 configuration:
         - TIM5CLK is set to 16 MHz, the TIM5 Prescaler is equal to 128 so the TIM5 counter
         clock used is 16 MHz / 128 = 125 000 Hz
         - With 125 000 Hz we can generate time base:
        max time base is 524288 us if TIM5_PERIOD = 65535 --> (65535 + 1) / 125000 = 524 288us
        min time base is 16 us if TIM5_PERIOD = 1   --> (1 + 1) / 125000 = 16us */
    TIM5_TimeBaseInit(TIM5_Prescaler_128, TIM5_CounterMode_Up, TIM5_PERIOD); //设置TIM5每10ms中断一次

    /* Clear TIM5 update flag */
    TIM5_ClearFlag(TIM5_FLAG_Update);

    /* Enable update interrupt */
    TIM5_ITConfig(TIM5_IT_Update, ENABLE);

    /* Enable TIM5 */
    TIM5_Cmd(ENABLE);

    /* Feed watch-dog by ISR of TIM5 */
    wdg_InitAndEnable();	

    return;
}

/*---------------------------------------------------------------------------*/
/**
 * Get count of system tick.
 *
 * This function used a trick to avoid race condition on "current_clock" 
 * that accessed by process and ISR of SysTick.
 *
 * \param none
 *
 * \return count of system tick.
 *
 */
clock_time_t
clock_time(void)
{
    clock_time_t    t1, t2;

    /* Prevent race condition on "current_clock" that shared by process and ISR of SysTick */
    do
    {
        t1 = current_clock;
        t2 = current_clock;
    } while(t1 != t2);

    return t1; 
}

/*---------------------------------------------------------------------------*/
/**
 * Get current seconds of system.
 *
 * This function used a trick to avoid race condition on "current_seconds" 
 * that accessed by process and ISR of SysTick.
 *
 * \param none
 *
 * \return count of current seconds.
 *
 */
unsigned long
clock_seconds(void)
{
    unsigned long    t1, t2;

    /* Prevent race condition on "current_seconds" that shared by process and ISR of SysTick */
    do
    {
        t1 = current_seconds;
        t2 = current_seconds;
    } while(t1 != t2);

    return t1; 
}

/**
  * @brief  Start the clock timer for etimer-process.
  * @note  That need call this to start clock after wake up from HALT.
  * @param  None.
  */
void clock_start(void)
{
    /* Enable TIM5 CLK */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM5, ENABLE);

    /* Enable update interrupt */
    TIM5_ClearFlag(TIM5_FLAG_Update);
    TIM5_ITConfig(TIM5_IT_Update, ENABLE);

    /* Enable TIM5 */
    TIM5_Cmd(ENABLE);

    return;	
}

/**
  * @brief  Stop the clock timer for saved energy and enter HALT.
  * @note  Insure clear all INT before enter HALT, so need to stop the clock.
  * @param  None.
  */
void clock_stop(void)
{
    /* Disable TIM5 */
    TIM5_Cmd(DISABLE);
    TIM5_SetCounter(0); /* Clear counter otherwise disturb the next period. */

    /* EXPLAIN: MUST disable IRQ of TIM5 otherwise the interrupt issured once turn on TIM5. */
    TIM5_ClearFlag(TIM5_FLAG_Update);
    TIM5_ITConfig(TIM5_IT_Update, DISABLE);

    /* Disable TIM5 CLK */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM5, DISABLE);

    return;
}


