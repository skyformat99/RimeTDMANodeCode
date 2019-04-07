/*
************************************************************************************************
* Filename   	: TempTimer.c
* Programmer : JiangJun
* Description	: Operation of a temp timer 
* Date           : 2015-08-31 15:00
* Copyright    : (c) RimeLink (www.rimelink.com)
************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include <stdlib.h>
#include <stdint.h>
#include "Dbg.h"
#include "stm8l15x_tim3.h"
#include "TempTimer.h"


/*
*********************************************************************************************************
*                                                                        COMPILE SWITCH
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                                                      	MACRO DEFINITION
*********************************************************************************************************
*/
/**
 * @brief  the MAX period of Temp-Timer, 524 ms.
 */
#define TEMP_TIMER_MAX_PERIOD    524

/**
 * @brief  convert millisecond to period of Temp-Timer, 125=16000000/128/1000.
 */
#define CONVERT_MS_2_PERIOD(Ms)    ((Ms) * 125 - 1)


/*
*********************************************************************************************************
*                                            				     DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                   GLOBAL VARIABLE & STRUCTURE
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                   LOCAL VARIABLE & STRUCTURE
*********************************************************************************************************
*/
/**
 * @brief  Call back function when the Temp-Timer timeout.
 */
static TEMP_TIMER_CB    lpfnTempTimerCB = NULL;

/**
 * @brief  Delay time of Temp-Timer, the unit is millisecond.
 */
static volatile uint16_t    s_wTempTimerDelayMs = 0;

/**
 * @brief  Avoid repeat turned off timer.
 */
static volatile bool    s_bTempTimerIsOn = FALSE;
 

/*
*********************************************************************************************************
*                                                                        FUNCTION PROTOTYPE
*********************************************************************************************************
*/

/*---------------------------------------------------------------------------*/
/**
  * @brief  Initialize the temp timer.
  * @param  none
  */
void temptimer_Init(void)
{
    /* ATTENTION: MUST enable TIM3 CLK before access its registers. */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);

    TIM3_DeInit();

    /* TIM3 configuration:
         - TIM3CLK is set to 16MHz, the TIM3 Prescaler is equal to 128 so the TIM3 counter
         clock used is 16MHz / 128 = 125kHz
         - With 125kHz we can generate time base:
        max time base is 524ms if TIM3_PERIOD = 65535 --> (65535 + 1) / 125000 = 524ms
        min time base is 16 us if TIM3_PERIOD = 1   --> (1 + 1) / 125000 = 16us */
    TIM3_TimeBaseInit(TIM3_Prescaler_128, TIM3_CounterMode_Up, 0xFFFF);

    /* Disable TIM3 CLK */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, DISABLE);

    return;
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  Start a temp timer.
  * @param  wMs    count of millisecond.
  *              lpfnTimeOut    callback function for timeout.
  */
void temptimer_Start(uint16_t wMs, TEMP_TIMER_CB lpfnTimeOut)
{
    uint16_t    wPeriod;

    ASSERT(wMs > 0 && lpfnTimeOut);

    s_wTempTimerDelayMs = wMs;
    lpfnTempTimerCB = lpfnTimeOut;

    /* Whether need to set 1 or multi period to Temp-Timer. */
    if (s_wTempTimerDelayMs > TEMP_TIMER_MAX_PERIOD)
    {
        wPeriod = TEMP_TIMER_MAX_PERIOD;
        s_wTempTimerDelayMs -= TEMP_TIMER_MAX_PERIOD;
    }
    else
    {
        wPeriod = s_wTempTimerDelayMs;
        s_wTempTimerDelayMs = 0;
    }

    s_bTempTimerIsOn = TRUE;

    /* Enable TIM3 CLK */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);

    wPeriod = CONVERT_MS_2_PERIOD(wPeriod);    
    TIM3_SetAutoreload(wPeriod);

    /* Enable update interrupt */
    TIM3_ClearFlag(TIM3_FLAG_Update);
    TIM3_ITConfig(TIM3_IT_Update, ENABLE);

    /* Enable TIM3 */
    TIM3_Cmd(ENABLE);

    return;	
}

/*---------------------------------------------------------------------------*/
void temptimer_Stop(void)
{
    if (!s_bTempTimerIsOn)
    {
        return; /* Avoid repeat turned off */
    }
    else
    {
        s_bTempTimerIsOn = FALSE;
    }

    s_wTempTimerDelayMs = 0;

    /* Disable TIM3 */
    TIM3_Cmd(DISABLE);
    TIM3_SetCounter(0); /* Clear counter otherwise disturb the next period. */

    /* EXPLAIN: MUST disable IRQ of TIM3 otherwise the interrupt issured once turn on TIM3. */
    TIM3_ClearFlag(TIM3_FLAG_Update);
    TIM3_ITConfig(TIM3_IT_Update, DISABLE);

    /* Disable TIM3 CLK */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, DISABLE);

    return;
}

/*---------------------------------------------------------------------------*/
void temptimer_ISR(void)
{
    uint16_t    wPeriod;

    /* Clear interrupt pending bit */
    TIM3_ClearITPendingBit(TIM3_IT_Update);

    if (s_wTempTimerDelayMs > 0)
    {
        if (s_wTempTimerDelayMs > TEMP_TIMER_MAX_PERIOD)
        {
            wPeriod = TEMP_TIMER_MAX_PERIOD;
            s_wTempTimerDelayMs -= TEMP_TIMER_MAX_PERIOD;
        }
        else
        {
            wPeriod = s_wTempTimerDelayMs;
            s_wTempTimerDelayMs = 0;
        }
        wPeriod = CONVERT_MS_2_PERIOD(wPeriod);    
        TIM3_SetAutoreload(wPeriod);
    }
    else
    {
        ASSERT(NULL != lpfnTempTimerCB);
        (* lpfnTempTimerCB)();
    }

    return;	
}

#if 0
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
{
    rtimer_clock_t    s_tWaitMs;	
    while (1)
    {
        uint16_t    wCnt;

        wCnt = SEND_INTERVAL * 10;
	
        temptimer_Start(wCnt, InformTimeout);
        s_tWaitMs = RTIMER_NOW();

        s_bRTCIssured = FALSE;
        while (!s_bRTCIssured)
            nop();

        s_tWaitMs = RTIMER_NOW() - s_tWaitMs;
        temptimer_Stop(); /* Stop timer for saved energy. */

        int16_t    nVal;
        nVal = s_tWaitMs - wCnt;
        if ((nVal > 1) || (nVal < -1))
            nop();
    }
}
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
#endif

