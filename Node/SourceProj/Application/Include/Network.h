/**
 * \file
 *         Network.h
 * \description
 *         Operation of communicated to Sink
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-11-17 10:02
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */
 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NETWORK_H__
#define __NETWORK_H__

/* Includes ------------------------------------------------------------------*/
#include "SystSettings.h"

/* Exported variables ------------------------------------------------------- */


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Imported macros -----------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
void network_Init(void);
void network_UpdateNetSettings(const NetSettings_t *p_stNew);
void network_AlarmByRTC(void);
void network_UpdateTxPwr(int8_t chTxPwr);
int16_t network_GetPktRSSI(void);
int8_t network_GetPktSNR(void);
bool network_IsJoinNet(void);

#if (!REL_VER)
void network_UpdateBROnAir(BR_ON_AIR_Type tBROnAir);
void network_UpdateFreq(uint32_t ulFreq);
void network_TogglePrintWakeCnt(void);
void network_Poll(void);
#endif


#endif

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/



