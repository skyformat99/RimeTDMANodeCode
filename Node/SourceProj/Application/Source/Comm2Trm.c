/**
 * \file
 *         Comm2Trm.c
 * \description
 *         Communicate to terminal like as User System.
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-11-20 14:22
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "process.h"
#include "main.h"
#include "Chip.h"
#include "Util.h"
#include "Dbg.h"
#include "SystSettings.h"
#include "CommPC.h"
#include "Network.h"
#include "Tim4UART.h"
#include "PwrManage.h"
#include "Comm2Trm.h"


/* Private macro -------------------------------------------------------------*/
#define COMM_TRM_HEAD    0x3Cu
#define COMM_TRM_TAIL    0x0Du

/**
* @brief  Flag for set RF settings
*/
#define RF_SETTINGS_FLAG1    0x55u
#define RF_SETTINGS_FLAG2    0xAAu

/**
* @brief  Event for Comm2Trm process
*/
#define C2T_EVENT_RX_WAKE_DATA    (PROCESS_EVENT_MAX + 1)


/* Private typedef -----------------------------------------------------------*/
/**
* @brief  Status of received communication frame
*/
typedef enum
{
    STATUS_IDLE = (uint8_t)0,
    STATUS_HEAD, /* Rx Head=0x3C */
    STATUS_TYPE, /* Rx Type */
    STATUS_DATA, /* Data filed */
    STATUS_TAIL, /* Tail=0x0D */
    STATUS_END, /* End of this frame */
} COMM_TRM_STATUS_TypeDef;


/**
* @brief  Data object for received communication frame
*/
typedef struct
{
    uint8_t    byCnt; /* Count of 1 field */
    uint8_t    byDataLen; /* Length of data field */
    uint8_t    byFrameLen; /* Length of frame */
    COMM_TRM_STATUS_TypeDef    eRxStatus;
    uint8_t    a_byRxBuf[MAX_LEN_COMM_TRM_DATA];	
} COMM_TRM_DATA;


/**
* @brief  User System set or get RF settings.
*/
typedef struct
{
    uint8_t    a_byFlag[2]; /* 0x55 0xAA */
    BR_ON_AIR_Type    tBROnAir; /* BW+SF+FEC */
    uint32_t    lFreq; /* Frequency */
    uint16_t    wCRC16;
} COMM_FRAME_RF_SETTINGS;

/**
* @brief  User System get network settings.
*/
typedef struct
{
#if (UPLINK == CUR_SYST)
    uint8_t    byMaxPayload; /* Bytes of the MAX uplink payload. */
    NetAddr_t    tNetAddr;
    uint16_t    wSlotLen; /* Slot length, the unit is millisecond. */
    int32_t    lSlotOffset; /* milliseconds, the first uplink time. */
    int32_t    lSlotInterval; /* milliseconds, the interval between 2 uplinked. */
#else
    NetAddr_t    tNetAddr;
    WNetConf_t    stWNetConf;
#endif
} COMM_FRAME_NET_SETTINGS;

/**
* @brief  Uplink data buffer accessed by Comm2Trm and Network process.
*/
typedef struct
{
    uint8_t    byDataSize;
    uint8_t    a_byDataBuf[MAX_LEN_UART_FRAME_DATA];
} UPLINK_DATA_BUF;


/* Private Constants ---------------------------------------------------------*/


/* Private variables ----------------------------------------------------------*/
PROCESS_NAME(Comm2TrmProcess);

/**
* @brief  Data object for received communication frame.
* @note  Prevent race condition that accessed by both ISR and process.
*/
static COMM_TRM_DATA    s_stComm2TrmData;

/**
* @brief  MUST lock this buffer when make responsed frame that accessed by ISR.
*/
static uint8_t* const    s_pbyRespBuf = (uint8_t *)&s_stComm2TrmData;

/**
* @brief  Uplink data buffer accessed by Comm2Trm and Network process.
*/
static UPLINK_DATA_BUF    s_stUplinkDataBuf;

/**
* @brief  Settings of network.
*/
static NetSettings_t    s_stNetSettingsC2T;

#if (UPLINK_WAKE == CUR_SYST)
static uint8_t    s_abyWakeDataBuf[MAX_LEN_COMM_TRM_DATA];
static UPLINK_DATA_BUF    s_stWakeAckBuf;
#endif


/* Private function prototypes -------------------------------------------------*/
#if (UPLINK_WAKE == CUR_SYST)
static void SendWakeAck(void);
#endif

/**
* @brief  Make response type of UART frame. 
*/
#define MAKE_UART_TYPE_RESP(byType)    (0x80u + (byType))

/**
* @brief  Lock for protected buffer of received communication frame. 
*/
static volatile bool    s_bIsLockedComm2TrmBuf = FALSE;
#define IS_LOCKED_COMM_2_TRM_BUF()    s_bIsLockedComm2TrmBuf
#define LOCK_COMM_2_TRM_BUF()    do { s_bIsLockedComm2TrmBuf = TRUE; } while (0)
#define UNLOCK_COMM_2_TRM_BUF()    do { s_bIsLockedComm2TrmBuf = FALSE; } while (0)


/*-------------------------------------------------------------------------*/
static uint8_t *GetRespBufDataPtr(void)
{
    return s_pbyRespBuf + sizeof(COMM_FRAME_HEAD);
}

/*-------------------------------------------------------------------------*/
static void SetRespBufDataSize(uint8_t bySize)
{
    ((COMM_FRAME_HEAD *)s_pbyRespBuf)->byDataSize = bySize;

    return;
}

/*-------------------------------------------------------------------------*/
static void MakeTxRespBuf(COMM_FRAME_TYPE_TypeDef eType)
{
    uint16_t    wHeadDataSize;

    ((COMM_FRAME_HEAD *)s_pbyRespBuf)->byHead = COMM_TRM_HEAD;
    ((COMM_FRAME_HEAD *)s_pbyRespBuf)->eType =    \
        (COMM_FRAME_TYPE_TypeDef)MAKE_UART_TYPE_RESP(eType);

    /* Calculate CheckSum */
    wHeadDataSize = ((COMM_FRAME_HEAD *)s_pbyRespBuf)->byDataSize;
    wHeadDataSize += sizeof(COMM_FRAME_HEAD);
    s_pbyRespBuf[wHeadDataSize] = util_CalcCS(s_pbyRespBuf, wHeadDataSize);

    s_pbyRespBuf[wHeadDataSize + 1] = COMM_TRM_TAIL; /* Add 1 for CS */

    /* Send this UART frame to terminal */
    cpc_Tx(s_pbyRespBuf, wHeadDataSize + sizeof(COMM_FRAME_TAIL));
	
    return;
}

/*-------------------------------------------------------------------------*/
static void GetID(void)
{
#if CATCH_NET_BLOCK
    extern void network_GetLC(void *p_vPrintBuf);
    network_GetLC(GetRespBufDataPtr());

    SetRespBufDataSize(strlen((char *)GetRespBufDataPtr()) + 1); /* Add 1 for '\0' */
    MakeTxRespBuf(TYPE_GET_VER);
#else
    strcpy((char *)GetRespBufDataPtr(), CUR_VER);
  #if (UPLINK_WAKE == CUR_SYST)
    strcat((char *)GetRespBufDataPtr(), ", ID=");
    strcat((char *)GetRespBufDataPtr(), chip_GetID());
  #endif
    SetRespBufDataSize(strlen((char *)GetRespBufDataPtr()) + 1); /* Add 1 for '\0' */
    MakeTxRespBuf(TYPE_GET_VER);
#endif

    return;	
}

/*-------------------------------------------------------------------------*/
static void SendRFPacket(void)
{
    uint8_t    *p_byBuf;

    /* Save data into uplink data buffer. */
    p_byBuf = sizeof(COMM_FRAME_HEAD) + s_stComm2TrmData.a_byRxBuf;	
    s_stUplinkDataBuf.byDataSize = s_stComm2TrmData.byDataLen;
    memcpy(s_stUplinkDataBuf.a_byDataBuf, p_byBuf, s_stComm2TrmData.byDataLen);

    /* Response the string of "TX OK" */
    strcpy((char *)GetRespBufDataPtr(), "TX OK");
    SetRespBufDataSize(6);
    MakeTxRespBuf(TYPE_TX_RF_DATA);

    return;
}

/*-------------------------------------------------------------------------*/
static void SetRFSettings(void)
{
    INT16U    wTemp;
    const char    *p_chStr;
    const COMM_FRAME_RF_SETTINGS    *p_stRF;

    p_chStr = "OK";
    p_stRF = (const COMM_FRAME_RF_SETTINGS *)(sizeof(COMM_FRAME_HEAD) + s_stComm2TrmData.a_byRxBuf);

    /* Check whether need to set settings of network. */
    if (p_stRF->tBROnAir == s_stNetSettingsC2T.tBROnAir && p_stRF->lFreq == s_stNetSettingsC2T.lFreq)
    {
        goto resp_set_rf; /* Don't need set */
    }

    /* Check flag */
    if (!(RF_SETTINGS_FLAG1 == p_stRF->a_byFlag[0] && RF_SETTINGS_FLAG2 == p_stRF->a_byFlag[1]))
    {
        p_chStr = "FLAG is error";
        goto resp_set_rf;
    }

    /* Check CRC16 */
    wTemp = GET_ST_FLD_OFFSET(COMM_FRAME_RF_SETTINGS, wCRC16);
    wTemp = util_CRC16((const INT8U *)p_stRF, wTemp);
    if (p_stRF->wCRC16 != wTemp)
    {
        p_chStr = "CRC16 error";
        goto resp_set_rf;
    }

    /* bps is valid? */
    if (!( BR_ON_AIR_443 == p_stRF->tBROnAir ||
            BR_ON_AIR_2876 == p_stRF->tBROnAir ||
            BR_ON_AIR_20334 == p_stRF->tBROnAir ))
    {
        p_chStr = "Bad bps, valid is[4, 7, 10]";
        goto resp_set_rf;
    }

    /* Frequency is valid? */
    if (!( RF_FREQ_MIN <= p_stRF->lFreq && p_stRF->lFreq <= RF_FREQ_MAX))
    {
        p_chStr = "Bad frequency, valid is[410000000~525000000]";
        goto resp_set_rf;
    }

    /* Save RF settings into Net-Settings and EEPROM. */
    s_stNetSettingsC2T.tBROnAir = p_stRF->tBROnAir;
    s_stNetSettingsC2T.lFreq = p_stRF->lFreq;
    ss_SaveNetSettings(&s_stNetSettingsC2T);

    /* Update the Net-Settings of Network-Process. */
    network_UpdateNetSettings(&s_stNetSettingsC2T);
	
resp_set_rf:
    strcpy((char *)GetRespBufDataPtr(), p_chStr);
    SetRespBufDataSize(strlen(p_chStr) + 1); /* Add 1 for '\0' */
    MakeTxRespBuf(TYPE_SET_RF_SETTINGS);
	
    return;    
}

/*---------------------------------------------------------------------------------------------*/
static void GetRFSettings(void)
{
    uint16_t    wTemp; 
    COMM_FRAME_RF_SETTINGS    *p_stRF;	

    p_stRF = (COMM_FRAME_RF_SETTINGS *)GetRespBufDataPtr();

    p_stRF->a_byFlag[0] = RF_SETTINGS_FLAG1;
    p_stRF->a_byFlag[1] = RF_SETTINGS_FLAG2;
    p_stRF->tBROnAir = s_stNetSettingsC2T.tBROnAir;
    p_stRF->lFreq = s_stNetSettingsC2T.lFreq;

    /* Calculate CRC16 */
    wTemp = GET_ST_FLD_OFFSET(COMM_FRAME_RF_SETTINGS, wCRC16);
    p_stRF->wCRC16 = util_CRC16((const INT8U *)p_stRF, wTemp);
    
    /* Make UART frame */
    SetRespBufDataSize(sizeof(COMM_FRAME_RF_SETTINGS));
    MakeTxRespBuf(TYPE_GET_RF_SETTINGS);

    return;	
}

/*---------------------------------------------------------------------------------------------*/
static void GetNetSettings(void)
{
    COMM_FRAME_NET_SETTINGS    *p_stNet;	

    p_stNet = (COMM_FRAME_NET_SETTINGS *)GetRespBufDataPtr();

    if (network_IsJoinNet())
    {
        p_stNet->tNetAddr = s_stNetSettingsC2T.tNetAddr;
    }
    else
    {
        p_stNet->tNetAddr = NET_ADDR_INVALID;
    }

#if (UPLINK == CUR_SYST)
    p_stNet->byMaxPayload = s_stNetSettingsC2T.byMaxPayload;
    p_stNet->wSlotLen = s_stNetSettingsC2T.wSlotLen;
    p_stNet->lSlotOffset = s_stNetSettingsC2T.lSlotOffset;
    p_stNet->lSlotInterval = s_stNetSettingsC2T.lSlotInterval;
#else
    memcpy(&p_stNet->stWNetConf, &s_stNetSettingsC2T.stWNetConf, sizeof(WNetConf_t));
#endif

    /* Make UART frame */
    SetRespBufDataSize(sizeof(COMM_FRAME_NET_SETTINGS));
    MakeTxRespBuf(TYPE_GET_NET_SETTINGS);

    return;	
}


/*---------------------------------------------------------------------------------------------*/
#if (UPLINK == CUR_SYST)
static void QuitNet(void)
{
    comm2trm_QuitNet();

    /* Response the string of "OK" */
    strcpy((char *)GetRespBufDataPtr(), "OK");
    SetRespBufDataSize(2 + 1); /* Add 1 for '\0' */
    MakeTxRespBuf(TYPE_QUIT_NET);

    return;
}
#endif

/*---------------------------------------------------------------------------------------------*/
static void SetTxPwr(void)
{
    #define BAD_TX_PWR    "Bad TX power, valid is[-1~20]"

    int8_t    chTxPwr;

    /* Check validity of the TX power. */
    chTxPwr = s_stComm2TrmData.a_byRxBuf[sizeof(COMM_FRAME_HEAD)];
    if (RF_TX_PWR_MIN <= chTxPwr && chTxPwr <= RF_TX_PWR_MAX)
    {
        if (chTxPwr != s_stNetSettingsC2T.chTxPwr)
        {
            s_stNetSettingsC2T.chTxPwr = chTxPwr;
            ss_SaveNetSettings(&s_stNetSettingsC2T);
            network_UpdateTxPwr(chTxPwr);
        }
        strcpy((char *)GetRespBufDataPtr(), "OK");
        SetRespBufDataSize(2 + 1); /* Add 1 for '\0' */
    }
    else
    {
        strcpy((char *)GetRespBufDataPtr(), BAD_TX_PWR);
        SetRespBufDataSize(strlen(BAD_TX_PWR) + 1); /* Add 1 for '\0' */
    }
    MakeTxRespBuf(TYPE_SET_TX_PWR);

    return;
}

/*---------------------------------------------------------------------------------------------*/
static void GetTxPwr(void)
{
    *(int8_t *)GetRespBufDataPtr() = s_stNetSettingsC2T.chTxPwr;
    SetRespBufDataSize(sizeof(s_stNetSettingsC2T.chTxPwr));
    MakeTxRespBuf(TYPE_GET_TX_PWR);

    return;
}

/*---------------------------------------------------------------------------------------------*/
static void GetPktRSSI(void)
{
    int16_t    *p_nRSSI;

    p_nRSSI = (int16_t *)GetRespBufDataPtr();
    *p_nRSSI = network_GetPktRSSI();

#if (COMPANY_ShuWei == CUR_COMPANY)
    int8_t    *p_chSNR;
    p_chSNR = (int8_t *)(GetRespBufDataPtr() + sizeof(int16_t));
    *p_chSNR = network_GetPktSNR();
    SetRespBufDataSize(sizeof(int16_t) + sizeof(int8_t));
#else
    SetRespBufDataSize(sizeof(int16_t));
#endif

    MakeTxRespBuf(TYPE_GET_PKT_RSSI);

    return;
}

/*-------------------------------------------------------------------------*/
static int8_t ProcessUartFrame(void)
{
    uint8_t    byHeadDataSize, byCS;
    COMM_FRAME_TYPE_TypeDef    eType;	

    LOCK_COMM_2_TRM_BUF(); /* Exclude ISR of UART */

    /* Check whether the CS is valid */
    byHeadDataSize = s_stComm2TrmData.byDataLen + sizeof(COMM_FRAME_HEAD);
    byCS = util_CalcCS(&s_stComm2TrmData.a_byRxBuf[0], byHeadDataSize);
    if (byCS != s_stComm2TrmData.a_byRxBuf[byHeadDataSize])
    {
        UNLOCK_COMM_2_TRM_BUF();
    #if EXTI_HALT_UART
        exti4uart_End(); /* Agree HALT and enable next RX. */
    #else
        comm2trm_BeginRxData(); /* For RX the next UART frame. */
    #endif
        return -1; /* CS is invalid */
    }

    eType = ((COMM_FRAME_HEAD *)&s_stComm2TrmData.a_byRxBuf[0])->eType;
    switch (eType)
    {
        case TYPE_GET_VER:
            GetID();			
            break;
        case TYPE_TX_RF_DATA:
            SendRFPacket();
            break;
        case TYPE_SET_RF_SETTINGS:
            SetRFSettings();			
            break;
        case TYPE_GET_RF_SETTINGS:
            GetRFSettings();			
            break;
        case TYPE_GET_NET_SETTINGS:
            GetNetSettings();
            break;
    #if (UPLINK == CUR_SYST)
        case TYPE_QUIT_NET:
            QuitNet();
            break;
    #endif
        case TYPE_SET_TX_PWR:
            SetTxPwr();
            break;
        case TYPE_GET_TX_PWR:
            GetTxPwr();
            break;
    #if (UPLINK_WAKE == CUR_SYST)
        case TYPE_WAKE_ACK:
            SendWakeAck();
            break;
    #endif
        case TYPE_GET_PKT_RSSI:
            GetPktRSSI();
            break;
        default:
            ASSERT(!"Bad type of comm frame.\r\n");			
            break;    
    }

    UNLOCK_COMM_2_TRM_BUF();
#if (!EXTI_HALT_UART)
    comm2trm_BeginRxData(); /* For RX the next UART frame. */
#endif

    return 0;    	
}


#if (UPLINK_WAKE == CUR_SYST)
/*-------------------------------------------------------------------------*/
static void SendWakeData(void)
{
    uint8_t    byDataSize;
    COMM_FRAME_HEAD    *p_stHead;
    COMM_FRAME_TAIL    *p_stTail;

    /* Add data of "COMM_FRAME_HEAD" */
    p_stHead = (COMM_FRAME_HEAD *)&s_abyWakeDataBuf[0];
    p_stHead->byHead = COMM_TRM_HEAD;
    p_stHead->eType = TYPE_WAKE_DATA;
    byDataSize = p_stHead->byDataSize;

    /* Add data of "COMM_FRAME_TAIL" */
    p_stTail = (COMM_FRAME_TAIL *)&s_abyWakeDataBuf[sizeof(COMM_FRAME_HEAD) + byDataSize];
    p_stTail->byCS = util_CalcCS(s_abyWakeDataBuf, byDataSize + sizeof(COMM_FRAME_HEAD));
    p_stTail->byTail = COMM_TRM_TAIL;

    /* Send this UART frame to terminal */
    cpc_Tx( s_abyWakeDataBuf, 
                 byDataSize + sizeof(COMM_FRAME_HEAD) + sizeof(COMM_FRAME_TAIL) );

    return;
}

/*-------------------------------------------------------------------------*/
static void SendWakeAck(void)
{
    uint8_t    *p_byBuf;

    /* Save data into uplink data buffer. */
    p_byBuf = sizeof(COMM_FRAME_HEAD) + s_stComm2TrmData.a_byRxBuf;	
    s_stWakeAckBuf.byDataSize = s_stComm2TrmData.byDataLen;
    memcpy(s_stWakeAckBuf.a_byDataBuf, p_byBuf, s_stComm2TrmData.byDataLen);

    /* Response the string of "OK" */
    strcpy((char *)GetRespBufDataPtr(), "OK");
    SetRespBufDataSize(3);
    MakeTxRespBuf(TYPE_WAKE_ACK);

    return;
}
#endif


/*---------------------------------------------------------------------------*/
PROCESS(Comm2TrmProcess, "communicate to terminal process");

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(Comm2TrmProcess, ev, data)
{
    PROCESS_BEGIN();

#if EXTI_HALT_UART
    pm_AgrHalt(PWR_ID_COMM2TRM);
#endif

    while (1)
    {
        /* Block process until received an event */
        PROCESS_YIELD();

        if (PROCESS_EVENT_POLL == ev)
        {
            ProcessUartFrame();
        }
    #if (UPLINK_WAKE == CUR_SYST) 
        else if (C2T_EVENT_RX_WAKE_DATA == ev)
        {
        #if EXTI_HALT_UART
            /* Don't HALT that need TX data, the UART_TX_ISR would agree HALT. */
            pm_OpposeHalt(PWR_ID_COMM2TRM);
        #endif
            SendWakeData();
        }
    #endif
        else
        {
            ASSERT(!"Comm2TrmProcess: Bad event!.\r\n");
        }
    }

    PROCESS_END();
}

/**
  * @brief  Put a data that received by UART into buffer.
  * @note  Prevent race condition this called by ISR. 
  * @param  uint8_t byData: the data received by UART.
  * @retval  None
  */
void comm2trm_RxUartData(uint8_t byData)
{
    if (IS_LOCKED_COMM_2_TRM_BUF())
    {
        PRINTF("Error: Comm to trm buffer is locked.\r\n", 0, PRINTF_FORMAT_NONE);
        return; /* Exit that buffer is locked by Comm2TrmProcess */
    }

    /* Update status according to the received data */
    switch (s_stComm2TrmData.eRxStatus)
    {
        case STATUS_IDLE:
            if (COMM_TRM_HEAD == byData) /* Is Head */
            {
                s_stComm2TrmData.eRxStatus = STATUS_HEAD;
            #if (!EXTI_HALT_UART)
                tim4uart_Start(MAX_TIME_UART_RX); /* Enable TIM for timeout of UART RX. */
            #endif
            }
            else
            {
                goto rx_exception;
            }
            break;
        case STATUS_HEAD:
            if (TYPE_INVALID_MIN < byData && byData < TYPE_INVALID_MAX) /* Valid type */
            {
                s_stComm2TrmData.eRxStatus = STATUS_TYPE;
            }
            else
            {
                goto rx_exception;
            }
            break;
        case STATUS_TYPE:
            if (byData <= MAX_LEN_UART_FRAME_DATA) /* Valid data size */
            {
                s_stComm2TrmData.eRxStatus = STATUS_DATA;
                s_stComm2TrmData.byDataLen = byData;
            }
            else
            {
                goto rx_exception;
            }
            break;
        case STATUS_DATA:
            if (s_stComm2TrmData.byCnt < s_stComm2TrmData.byDataLen)
            {
                ++s_stComm2TrmData.byCnt;
            }
            else
            {
                s_stComm2TrmData.eRxStatus = STATUS_TAIL;
            }
            break;
        case STATUS_TAIL:
            if (COMM_TRM_TAIL == byData)
            {
            #if EXTI_HALT_UART
                cpc_DisRx(); /* Disable RX of UART prevent disturbed. */
            #endif
                tim4uart_Stop(); /* Stop the TIM that RX of UART is done. */
                process_poll(&Comm2TrmProcess); /* Tell process to deal with the received frame. */
            }
            else
            {
                goto rx_exception;
            }
            break;
        default:
            ASSERT(!"Error: Bad status of comm2trm_RxUartData().\r\n");
            break;
    }

    /* Save the received data */
    s_stComm2TrmData.a_byRxBuf[s_stComm2TrmData.byFrameLen++] = byData;
    return;

rx_exception:
    tim4uart_Stop(); /* Stop the TIM that RX of UART is done. */	
#if EXTI_HALT_UART
    cpc_DisRx(); /* Disable RX of UART prevent disturbed. */
    exti4uart_End(); /* Agree HALT and enable next RX. */
#else
    comm2trm_BeginRxData();
#endif

    return;	
}

/**
  * @brief  Fetch Net-Settings from EEPROM and start Comm2Trm-Process.
  * @param  None.
  * @retval  None.
  */
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
#if 0
static uint8_t    s_abyTestBuf[MAX_LEN_UART_FRAME_DATA];
#endif
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

void comm2trm_Init(void)
{
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
#if 0
    uint8_t    byCopySize;
    for (byCopySize = 0; byCopySize < sizeof(s_abyTestBuf); ++byCopySize)
    {
        s_abyTestBuf[byCopySize] = byCopySize + 1; /* Start from 1 */
    }
#endif
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

    /* Fetch settings from EEPROM. */
    ss_FetchNetSettings(&s_stNetSettingsC2T);

    process_start(&Comm2TrmProcess, NULL);

    return;
}

/**
  * @brief  Get uplink data that delivered by User-System.
  * @param  void *p_vSaveBuf:  point to buffer that saved uplink data by this pointer.
  * @param  uint8_t byDesiredSize:  size of the desired copyed data.
  * @retval  size of the copyed data.
  */
uint8_t comm2trm_GetUplinkData(void *p_vSaveBuf, uint8_t byDesiredSize)
{
    uint8_t    byCopySize;

    ASSERT(p_vSaveBuf);

#if 1
    byCopySize = 0;
    if (0 < s_stUplinkDataBuf.byDataSize)
    {
        byCopySize = MIN(byDesiredSize, s_stUplinkDataBuf.byDataSize); /* Prevent overflow */
        memcpy(p_vSaveBuf, s_stUplinkDataBuf.a_byDataBuf, byCopySize);
        s_stUplinkDataBuf.byDataSize = 0; /* This uplink have finished. */
    }
#if 0 /* TX the accumulate counter to indicate have no data. */
    else
    {
        static uint32_t    s_ulC2TCnt = 0;
        ++s_ulC2TCnt;
        memcpy(p_vSaveBuf, &s_ulC2TCnt, sizeof(s_ulC2TCnt));
        byCopySize = sizeof(s_ulC2TCnt);
    }
#endif
#endif

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
#if 0
    #if 0 /* random length */
    byCopySize = util_GetRand16() % sizeof(s_abyTestBuf);
    if (0 == byCopySize)
    {
        byCopySize = 1; /* 1 ~ (sizeof(s_abyTestBuf) - 1) */
    }
    #endif

    #if 0 /* MAX length */
    byCopySize = MAX_LEN_UART_FRAME_DATA;
    #endif

    #if 1 /* desired length */
    byCopySize = byDesiredSize;
    #endif

    #if 0 /* accumulate data */
    static uint32_t    s_ulC2TCnt = 0;
    ++s_ulC2TCnt;
    *(uint32_t *)&s_abyTestBuf[0] = s_ulC2TCnt;
    byCopySize = sizeof(s_ulC2TCnt);
    #endif

    memcpy(p_vSaveBuf, s_abyTestBuf, byCopySize);
#endif
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

    return byCopySize;
}

/**
  * @brief  Update the Net-Settings of Comm2Trm-Process.
  * @param  const NetSettings_t *p_stNew    point to the new Net-Settings.
  * @retval  None.
  */
void comm2trm_UpdateNetSettings(const NetSettings_t *p_stNew)
{
    ASSERT(p_stNew);

    memcpy(&s_stNetSettingsC2T, p_stNew, sizeof(s_stNetSettingsC2T));

    return;
}

/**
  * @brief  Begin RX a frame of UART data by clear control data structure.
  * @param  None.
  * @retval  None.
  */
void comm2trm_BeginRxData(void)
{
     /* Clear buffer for the arrival frame */	
    s_stComm2TrmData.byCnt = 0;
    s_stComm2TrmData.byDataLen = 0;
    s_stComm2TrmData.byFrameLen = 0;
    s_stComm2TrmData.eRxStatus = STATUS_IDLE;

    return;
}


#if (UPLINK == CUR_SYST)
/**
  * @brief  Set this node quit from network by clear falg.
  * @param  None.
  * @retval  None.
  */
void comm2trm_QuitNet(void)
{
    s_stNetSettingsC2T.wJoinNetFlag = 0xFFFF;
    ss_SaveNetSettings(&s_stNetSettingsC2T);

    return;
}
#endif


#if (UPLINK_WAKE == CUR_SYST)
/*---------------------------------------------------------------------------------------------*/
int8_t comm2trm_RxWakeData(const void *p_vData, uint8_t bySize)
{
    ASSERT(p_vData);

    if (MAX_LEN_UART_FRAME_DATA < bySize)
    {
        return -1; /* Bad size of data. */
    }

    ((COMM_FRAME_HEAD *)s_abyWakeDataBuf)->byDataSize = bySize;
    memcpy(&s_abyWakeDataBuf[sizeof(COMM_FRAME_HEAD)], p_vData, bySize);

    process_post(&Comm2TrmProcess, C2T_EVENT_RX_WAKE_DATA, NULL);

    return 0;
}

/*---------------------------------------------------------------------------------------------*/
uint8_t comm2trm_GetWakeAck(void *p_vSaveBuf, uint8_t byDesiredSize)
{
    uint8_t    byCopySize;

    ASSERT(p_vSaveBuf);

#if 1
    byCopySize = 0;
    if (0 < s_stWakeAckBuf.byDataSize)
    {
        byCopySize = MIN(byDesiredSize, s_stWakeAckBuf.byDataSize); /* Prevent overflow */
        memcpy(p_vSaveBuf, s_stWakeAckBuf.a_byDataBuf, byCopySize);
        s_stWakeAckBuf.byDataSize = 0; /* This wake ack have finished. */
    }
#endif

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
#if 0 /* Ack the desired length */
    byCopySize = byDesiredSize;
    memcpy(p_vSaveBuf, s_abyTestBuf, byCopySize);
#endif
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

    return byCopySize;
}
#endif/*#if (UPLINK_WAKE == CUR_SYST)*/


/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
#if 0
void comm2trm_Test(void)
{
    uint8_t    bySize;
    int16_t    nCnt;
    static uint8_t    s_abyTest[5 + 0] =
    {
        0x3C, TYPE_GET_TX_PWR, 0x00,
    };

    s_abyTest[sizeof(s_abyTest) - 2] = util_CalcCS(s_abyTest, sizeof(s_abyTest) - 2);
    s_abyTest[sizeof(s_abyTest) - 1] = COMM_TRM_TAIL;

    s_stComm2TrmData.byDataLen = sizeof(s_abyTest) - 5;
    memcpy(s_stComm2TrmData.a_byRxBuf, s_abyTest, sizeof(s_abyTest));

    s_stNetSettingsC2T.chTxPwr = 20;

    ProcessUartFrame();

#if 0
    bySize = comm2trm_GetWakeAck(s_abyTest, sizeof(s_abyTest));
    comm2trm_RxWakeData(s_abyTest, bySize);
    SendWakeData();
    bySize = comm2trm_GetWakeAck(s_abyTest, sizeof(s_abyTest));
#endif

#if 0
#if (UPLINK == CUR_SYST)
    s_stNetSettingsC2T.byMaxPayload = MAX_LEN_UART_FRAME_DATA;
    s_stNetSettingsC2T.tNetAddr = 0x0000;
    s_stNetSettingsC2T.nNodeNum = 100;
    s_stNetSettingsC2T.wJoinNetFlag = 0xFFFF; /* Have not join. */
    s_stNetSettingsC2T.wSlotLen = 500; /* 500ms */
    s_stNetSettingsC2T.lSlotOffset = (int32_t)1000 * HHMMSS_2SEC(0, 0, 0); /* 00:00:00.000 */
    s_stNetSettingsC2T.lSlotInterval = (int32_t)1000 * HHMMSS_2SEC(12, 0, 0); /* 12:00:00 */
#else
    s_stNetSettingsC2T.wJoinNetFlag = 0xFFFF; /* Have not join. */
    s_stNetSettingsC2T.tNetAddr = 0x0000;
    s_stNetSettingsC2T.stWNetConf.byUplinkPayload = 4;
    s_stNetSettingsC2T.stWNetConf.chRepeatNum = 0;
    s_stNetSettingsC2T.stWNetConf.byWakeDataSize = 4;
    s_stNetSettingsC2T.stWNetConf.byWakeAckSize = 0;
    s_stNetSettingsC2T.stWNetConf.nNodeNum = 3;
    s_stNetSettingsC2T.stWNetConf.wSlotLen = 1500; /* 1500ms */
    s_stNetSettingsC2T.stWNetConf.lUplinkPeriod = 8000; /* 8s */
    s_stNetSettingsC2T.stWNetConf.lWakeInterval = 8000; /* 8s */
    s_stNetSettingsC2T.byNetVer = 1;
#endif
    s_stNetSettingsC2T.tBROnAir = BR_ON_AIR_443;
    s_stNetSettingsC2T.lFreq = RF_FREQ_DEFAULT; /* 470MHz */

    GetNetSettings();
#endif

#if 0
    uint8_t    byCnt, byLoop;
    static const uint8_t    s_aabyTestData[][7] =
    {
        {0x3C, 0x04, 0x00, 0xFF, 0x0D},
        {0x3C, 0x04, 0x00, 0xFF, 0x1D},
        {0x3C, 0x01, 0x02, 0x12, 0x34, 0xFF, 0x0D},
    };

    for (byCnt = 0; byCnt < SIZE_OF_ARRAY(s_aabyTestData); ++byCnt)
    {
        for (byLoop = 0; byLoop < SIZE_OF_ARRAY(s_aabyTestData[0]); ++byLoop)
        {
            comm2trm_RxOneData(s_aabyTestData[byCnt][byLoop]);
        }
    }
#endif

    return;
}
#endif

#if 0
void comm2trm_SendUART(const void *p_vData, uint8_t byLen)
{
    LOCK_COMM_2_TRM_BUF(); /* Exclude ISR of UART */

    memcpy(GetRespBufDataPtr(), p_vData, byLen);
    SetRespBufDataSize(byLen);
    MakeTxRespBuf(TYPE_TX_RF_DATA);

    ClearCommFrame(); /* For RX next frame */
    UNLOCK_COMM_2_TRM_BUF();

    return;
}
#endif
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/


/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/
