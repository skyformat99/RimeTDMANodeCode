/**
 * \file
 *         Time4UART.h
 * \description
 *         Timer for timeout of UART RX
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-12-04 14:20
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */
 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_4_UART_H__
#define __TIM_4_UART_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported variables ------------------------------------------------------- */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void tim4uart_Init(void);
int8_t tim4uart_Start(uint16_t wMs);
void tim4uart_Stop(void);
void tim4uart_ISR(void);


#endif

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

