/**
 * \file
 *         SystSettings.h
 * \description
 *         Head file of System Settings
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-11-13 12:09
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */
 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYST_SETTINGS_H__
#define __SYST_SETTINGS_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "RTC.h"


/* Exported variables ------------------------------------------------------- */

/* Exported macros --------------------------------------------------------*/
#define RF_FREQ_DEFAULT    ((int32_t)470000000) /* 470MHz */
#define RF_FREQ_MIN    ((int32_t)410000000) /* 410MHz */
#define RF_FREQ_MAX    ((int32_t)525000000) /* 525MHz */

#define RF_TX_PWR_MIN    -1
#define RF_TX_PWR_MAX    20


/* Exported types ------------------------------------------------------------*/
/**
* @brief  type of network address.
*/
#define NET_ADDR_INVALID    0x0000
#define NET_ADDR_BROADCAST    0xFFFF
typedef uint16_t    NetAddr_t;

/**
* @brief  Bit rate on air
*/
typedef enum
{
    BR_ON_AIR_START = (uint8_t)1, /* Start from 1 */
    BR_ON_AIR_66 = BR_ON_AIR_START, /* 1=66.2 bps */
    BR_ON_AIR_132, /* 2=132.5 bps */
    BR_ON_AIR_243, /* 3=243.9 bps */
    BR_ON_AIR_443, /* 4=443.8 bps */
    BR_ON_AIR_887, /* 5=887.6 bps */
    BR_ON_AIR_1602, /* 6=1602 bps */
    BR_ON_AIR_2876, /* 7=2876 bps */
    BR_ON_AIR_5084, /* 8=5084 bps */
    BR_ON_AIR_10168, /* 9=10168 bps */
    BR_ON_AIR_20334, /* 10=20334.8 bps */
    BR_ON_AIR_END = BR_ON_AIR_20334,    
} BR_ON_AIR_Type;

/**
* @brief  Configure for wireless network.
*/
typedef struct
{
    uint8_t    byUplinkPayload; /* Bytes of the uplink payload of node. */
    int8_t    chRepeatNum; /* Repeat number of node uplink. */
    uint8_t    byWakeDataSize; /* Size of wake data. */
    uint8_t    byWakeAckSize; /* Size of wake ack. */
    int16_t    nNodeNum; /* Number of node in this subnet. */
    uint16_t    wSlotLen; /* Slot length, the unit is millisecond. */
    int32_t    lUplinkPeriod; /* milliseconds, the interval between 2 uplinked. */
    int32_t    lWakeInterval; /* milliseconds, the interval between 2 waked. */
} WNetConf_t;

/**
* @brief  Network settings
*/
typedef struct
{
    /* Settings of star network. */
#if (UPLINK == CUR_SYST)
    uint8_t    byMaxPayload; /* Bytes of the MAX uplink payload. */
    NetAddr_t    tNetAddr;
    int16_t    nNodeNum; /* Number of node in this subnet. */
    uint16_t    wJoinNetFlag; /* 0x1234=Join network; otherwise=Have not join. */
    uint16_t    wSlotLen; /* Slot length, the unit is millisecond. */
    int32_t    lSlotOffset; /* milliseconds, the first uplink time. */
    int32_t    lSlotInterval; /* milliseconds, the interval between 2 uplinked. */
#else
    uint16_t    wJoinNetFlag; /* 0x1234=Join network; otherwise=Have not join. */
    NetAddr_t    tNetAddr;
    WNetConf_t    stWNetConf;
    uint8_t    byNetVer;    
#endif
    /* Settings of RF. */
    int8_t    chTxPwr; /* Valid is [-1~20] */
    BR_ON_AIR_Type    tBROnAir; /* BW+SF+FEC */
    uint32_t    lFreq; /* Frequency */
} NetSettings_t;


/* Exported constants --------------------------------------------------------*/


/* Imported macros -----------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
int8_t ss_Init(void);
int16_t ss_FetchNetSettings(NetSettings_t *p_stSettings);
int8_t ss_SaveNetSettings(const NetSettings_t *p_stSettings);


#endif

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/


