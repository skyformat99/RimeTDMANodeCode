/**
* \file
*			Hardware abstraction layer for STM8L151C8.
* \author
*			JiangJun <jiangjunjie_2005@126.com>
*/


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
void watchdog_start(void)
{
    return;
}


void watchdog_stop(void)
{
    return;
}

void leds_on(void)
{
    return;
}

void leds_off(void)
{
    return;
}

/*--------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/


