/**
 * \file
 *         Time4UART.c
 * \description
 *         Timer for timeout of UART RX
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-12-04 13:39
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */


/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include "Dbg.h"
#include "stm8l15x_tim4.h"
#include "CommPC.h"
#include "Tim4UART.h"
#include "Comm2Trm.h"


/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/**
* @brief  16MHz / 32768 = 488Hz
*/
#define MIN_MS_TIM4UART    2 /* 2ms */
#define MAX_MS_TIM4UART    ((255 + 1) * 1000ul / 488) /* 524ms */
#define MS_2_TIM4UART_CNT(ms)    (((ms) * 488ul + 500) / 1000 - 1)


/* Private variables ---------------------------------------------------------*/
/**
 * @brief  Avoid repeat turned off timer.
 */
static volatile bool    s_bTim4UARTIsOn = FALSE;


/* Private function prototypes -----------------------------------------------*/


/* Private Constants ---------------------------------------------------------*/


/**
  * @brief  Initialize TIM4 for timeout of UART RX.
  * @param  None.
  * @retval  None
  */
void tim4uart_Init(void)
{
    /* ATTENTION: MUST enable TIM4 CLK before access its registers. */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);

    TIM4_DeInit();

    /* TIM4 configuration:
         - TIM4CLK is set to 16MHz, the TIM4 Prescaler is equal to 32768 so the TIM4 counter
         clock used is 16MHz / 32768 = 488Hz
         - With 488Hz we can generate time base:
        max time base is 524ms if TIM4_PERIOD = 255 --> (255 + 1) / 488 = 524ms
        min time base is 4ms if TIM4_PERIOD = 1   --> (1 + 1) / 125000 = 4ms */
    TIM4_TimeBaseInit(TIM4_Prescaler_32768, 255);

    /* Disable clock of TIM4 for saving energy. */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);

    return;
}

/**
  * @brief  Start TIM4 for timeout of UART RX.
  * @param  uint16_t wMs    milliseconds=[2, 524].
  * @retval  int8_t    0=OK, -1=Benn used, -2=ms too large.
  */
int8_t tim4uart_Start(uint16_t wMs)
{
    ASSERT((MIN_MS_TIM4UART < wMs) && (wMs < MAX_MS_TIM4UART));

    if (s_bTim4UARTIsOn)
    {
        return -1; /* Been used!*/
    }

    if (!((MIN_MS_TIM4UART < wMs) && (wMs < MAX_MS_TIM4UART)))
    {
        return -2; /* ms too large. */
    }

    s_bTim4UARTIsOn = TRUE;

    /* ATTENTION: MUST enable TIM4 CLK before access its registers. */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);

    TIM4_SetCounter(0); /* Start from 0 for accuracy. */
    TIM4_SetAutoreload(MS_2_TIM4UART_CNT(wMs));
	
    /* Clear update flag */
    TIM4_ClearFlag(TIM4_FLAG_Update);
    TIM4_ITConfig(TIM4_IT_Update, ENABLE);
	
    /* Enable TIM4 */
    TIM4_Cmd(ENABLE);

    return 0;
}

/**
  * @brief  Stop the TIM4.
  * @param  None.
  * @retval  None
  */
void tim4uart_Stop(void)
{
    if (!s_bTim4UARTIsOn)
    {
        return; /* Avoid repeat turned off */
    }
    else
    {
        s_bTim4UARTIsOn = FALSE;
    }

    /* Disable TIM4 */
    TIM4_Cmd(DISABLE);

    /* EXPLAIN: MUST disable IRQ of TIM4 otherwise the interrupt issured once turn on TIM4. */
    TIM4_ClearFlag(TIM4_FLAG_Update);
    TIM4_ITConfig(TIM4_IT_Update, DISABLE);
	
    /* Disable clock of TIM4 for saving energy. */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);

    return;
}

/**
  * @brief  Clear INT flag and stop TIM4.
  * @param  None.
  * @retval  None
  */
void tim4uart_ISR(void)
{
    tim4uart_Stop(); /* Stop timer for saving energy. */
#if EXTI_HALT_UART
    cpc_DisRx(); /* Disable RX of UART prevent disturbed. */
    exti4uart_End(); /* Agree HALT and enable next RX. */
#else
    comm2trm_BeginRxData();
#endif

    return;
}


/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

