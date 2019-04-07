/**
 * \file
 *         Communicate to terminal like as PC or external device.
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-05-25 12:17
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMM_2_TRM_H__
#define __COMM_2_TRM_H__


/* Includes ------------------------------------------------------------------*/
#include "Main.h"
#include "SystSettings.h"


/* Exported types ------------------------------------------------------------*/
/**
* @brief  Type of received communication frame
*/
typedef enum
{
    TYPE_INVALID_MIN = (uint8_t)0,
    TYPE_GET_VER, /*!< User System get version of this node. */
    TYPE_TX_RF_DATA, /*!< User System send data that need to TX by RF. */
    TYPE_SET_RF_SETTINGS, /*!< User System set settings of RF. */
    TYPE_GET_RF_SETTINGS, /*!< User System get settings of RF. */
    TYPE_GET_NET_SETTINGS, /*!< User System get network settings of this node. */
    TYPE_QUIT_NET, /*!< User System set this node quit from current network. */
    TYPE_SET_TX_PWR, /*!< User System set TX power of RF. */
    TYPE_GET_TX_PWR, /*!< User System get TX power of RF. */
    TYPE_WAKE_ACK, /*!< User System send wake ack to node. */
    TYPE_GET_PKT_RSSI, /*!< User System get the RSSI of the latest packet received. */
    TYPE_INVALID_MAX,

    TYPE_WAKE_DATA = ((uint8_t)0xC0), /*!< Node send wake data to User System. */
} COMM_FRAME_TYPE_TypeDef;


/* Exported variables ------------------------------------------------------- */
typedef struct
{
    uint8_t    byHead;
    COMM_FRAME_TYPE_TypeDef    eType;
    uint8_t    byDataSize;
} COMM_FRAME_HEAD;

typedef struct
{
    uint8_t    byCS;
    uint8_t    byTail;
} COMM_FRAME_TAIL;


/* Private macros ------------------------------------------------------------*/
#define MAX_LEN_COMM_TRM_DATA    255u
#if (UPLINK == CUR_SYST)
#define MAX_LEN_UART_FRAME_DATA    \
    ( MAX_LEN_COMM_TRM_DATA - sizeof(COMM_FRAME_HEAD) -    \
      sizeof(COMM_FRAME_TAIL) - sizeof(NetAddr_t) ) /* 2 bytes for NodeAddr. */
#else
#define MAX_LEN_UART_FRAME_DATA    \
    ( MAX_LEN_COMM_TRM_DATA - sizeof(COMM_FRAME_HEAD) -    \
      sizeof(COMM_FRAME_TAIL) - sizeof(NetAddr_t) - sizeof(int8_t)) /* 2 bytes for NodeAddr, 1 byte for RSSI. */
#endif


/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void comm2trm_Init(void);
void comm2trm_RxUartData(uint8_t byData);
uint8_t comm2trm_GetUplinkData(void *p_vSaveBuf, uint8_t byDesiredSize);
void comm2trm_UpdateNetSettings(const NetSettings_t *p_stNew);
void comm2trm_BeginRxData(void);
void comm2trm_QuitNet(void);
#if (UPLINK_WAKE == CUR_SYST)
int8_t comm2trm_RxWakeData(const void *p_vData, uint8_t bySize);
uint8_t comm2trm_GetWakeAck(void *p_vSaveBuf, uint8_t byDesiredSize);
#endif


#endif /*#ifndef __COMM_2_TRM_H__*/

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

