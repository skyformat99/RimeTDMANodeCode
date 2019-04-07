/**
 * \file
 *         Head file for codes based on specified MCU to operate sx1278
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-04-09 15:59
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SX1278_PORTS_H__
#define __SX1278_PORTS_H__

/* Includes ------------------------------------------------------------------*/
#include "rtimer.h"
#include "rtimer-arch.h"

/* Exported variables ------------------------------------------------------- */

/* Exported types ------------------------------------------------------------*/

/**
   * @brief  Convert 1 millisecond to LoRa clock.
*/
#define MS_2_LORA_CLOCK    (RTIMER_ARCH_SECOND / 1000)

/**
   * @brief  Get current value of LoRa clock.
*/
#define GET_LORA_CLOCK_NOW()    RTIMER_NOW()

/**
   * @brief  Type of LoRa clock.
*/
typedef rtimer_clock_t    lora_clock_t;


/* Exported constants --------------------------------------------------------*/
#define FXOSC_STD    32 /* 32MHz */
#define FXOSC_UNIT    1000000 /* MHz */

/* The SX1278 PLL uses a 19-bit sigma-delta modulator whose frequency resolution 
    is given by: Fstep=Fxosc/(2^19). */
#if ((iWL881A == CUR_PRODUCT) || (iWL882A == CUR_PRODUCT))
#define FXOSC    26 /* 32MHz */
#define FREQ_STEP    49.59106445
#else
#define FXOSC    32 /* 32MHz */
#define FREQ_STEP    61.03515625
#endif


/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void SX1278WriteBuffer(uint8_t addr, const uint8_t *buffer, uint8_t size);
void SX1278ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size);

void SX1278InitPins(void);
void SX1278PullLowResetPin(void);
void SX1278SetInputResetPin(void);
void SX1278SetOutputResetPin(void);

void DelayMs(uint16_t wMs);
void SX1278InitTimer(void (*Callback)(void));
void SX1278StartTimer(uint16_t wMs);
void SX1278StopTimer(void);

void SX1278EnterLowPower(void);
void SX1278ExitLowPower(void);
void SX1278SetSwPin2Tx(void);
void SX1278SetSwPin2Rx(void);

#endif
/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/
