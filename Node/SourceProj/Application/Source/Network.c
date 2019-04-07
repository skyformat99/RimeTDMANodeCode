/**
 * \file
 *         Network.c
 * \description
 *         Operation of communicated to Sink
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-11-13 22:32
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "pt.h"
#include "process.h"
#include "packetbuf.h"
#include "rtimer.h"
#include "etimer.h"
#include "main.h"
#include "SystSettings.h"
#include "Dbg.h"
#include "Comm2Trm.h"
#include "sx1278_src.h"
#include "Chip.h"
#include "RTC.h"
#include "PwrManage.h"


/* Compile switch-------------------------------------------------------------*/
#define UPLINK_DBG    RIME_DBG_OFF
#define CORRECT_RTC_DBG    RIME_DBG_ON
#define CALC_UPLINK_DBG    RIME_DBG_OFF
#define CALC_WAKE_DBG    RIME_DBG_OFF
#define WAKE_DBG    RIME_DBG_OFF
#define CAD_DBG    RIME_DBG_OFF
#define REPAIR_BROKEN_DBG    RIME_DBG_OFF
#define JOIN_DBG    RIME_DBG_OFF


/* Private typedef ------------------------------------------------------------*/
/**
* @brief  SX1278 DIO pins I/O definitions
*/
#define NUM_RADIO_BUF    1 /* Number of unit of radio buffer */
typedef struct 
{
    uint8_t    bySize; /* Size of valid data */
    uint8_t    a_byBuf[RF_FIFO_SIZE]; /* Data buffer */
    int32_t    lRxTime; /* The unit is millisecond */
} RADIO_BUF;

/**
* @brief  Result of radio operation
*/
typedef enum
{
    RF_None = (uint8_t)0,
    RF_Tx_Done,
    RF_Tx_Timeout,
    RF_Rx_Done,
    RF_Rx_Timeout,
    RF_Rx_Error,
    RF_Cad_Done,
    RF_Cad_Timeout,
} RF_Result_Typedef;

/**
* @brief  Node request join into network.
*/
typedef struct
{
    uint8_t    byFlag;
#if (UPLINK == CUR_SYST)
    uint8_t    a_bySubnet[4]; /* Always is "RIME" */
#else
    uint8_t    byNetVer;    
#endif
    DEV_ID    tDevID;
    uint16_t    wCRC16;    
} RF_FRAME_REQ_JOIN;

/**
* @brief  Sink response the Node join into network.
*/
typedef struct
{
    uint8_t    byFlag;
#if (UPLINK == CUR_SYST)
    uint8_t    byMaxPayload; /* Bytes of the MAX uplink payload. */
    DEV_ID    tDevID; /* Equal to the requested node. */
    NetAddr_t    tAddr;
    int16_t    nNodeNum; /* Number of node in this subnet. */
    uint16_t    wSlotLen; /* the unit is ms */
    int32_t    lSlotOffset; /* milliseconds, Time of the 1st uplink. */
    int32_t    lSlotInterval; /* milliseconds, Interval between 2 uplink. */
    int32_t    lCurRTC; /* milliseconds, Current RTC of Sink. */
#else
    uint8_t    byNetVer;
    NetAddr_t    tAddr;
    DEV_ID    tDevID;
    int32_t    lCurRTC; /* milliseconds, Current RTC of Sink. */
#endif
    uint16_t    wCRC16;    
} RF_FRAME_RESP_JOIN;

/**
* @brief  Sink response the Node join into network that have different version.
*/
typedef struct
{
    uint8_t    byFlag;
    uint8_t    byNetVer;
    NetAddr_t    tAddr;
    DEV_ID    tDevID;
    int32_t    lCurRTC; /* milliseconds, Current RTC of Sink. */
    WNetConf_t    stWNetConf;
    uint16_t    wCRC16;    
} RF_FRAME_RESP_JOIN_DIFF_VER;

/**
* @brief  Node uplink data to Sink.
*/
typedef struct
{
    uint8_t    byFlag;
#if (UPLINK_WAKE == CUR_SYST)
    uint8_t    byNetVer;    
#endif
    NetAddr_t    tAddr;
    uint8_t    a_byDataBuf[MAX_LEN_UART_FRAME_DATA];
} RF_FRAME_UPLINK_DATA;

/**
* @brief  Sink ACK to node that uplink data.
*/
typedef struct
{
    uint8_t    byFlag;
#if (UPLINK_WAKE == CUR_SYST)
    uint8_t    byNetVer;    
#endif
    NetAddr_t    tAddr;
    int32_t    lCurRTC; /* milliseconds, Current RTC of Sink. */
    uint16_t    wCRC16;    
} RF_FRAME_UPLINK_ACK;

/**
* @brief  Sink ACK to node that uplink data, have different network version.
*/
typedef struct
{
    uint8_t    byFlag;
    uint8_t    byNetVer;
    NetAddr_t    tAddr;
    int32_t    lCurRTC; /* milliseconds, Current RTC of Sink. */
    WNetConf_t    stWNetConf;
    uint16_t    wCRC16;    
} RF_FRAME_UPLINK_ACK_DIFF_VER;

/**
* @brief  Node request the Slot and RTC.
*/
typedef struct
{
    uint8_t    byFlag;
    NetAddr_t    tAddr;
} RF_FRAME_REQ_SLOT_RTC;

/**
* @brief  Sink response the Slot and RTC to Node.
*/
typedef struct
{
    uint8_t    byFlag;
    uint8_t    byMaxPayload; /* Bytes of the MAX uplink payload. */
    NetAddr_t    tAddr;
    int16_t    nNodeNum; /* Number of node in this subnet. */
    uint16_t    wSlotLen; /* the unit is ms */
    int32_t    lSlotOffset; /* milliseconds, Time of the 1st uplink. */
    int32_t    lSlotInterval; /* milliseconds, Interval between 2 uplink. */
    int32_t    lCurRTC; /* milliseconds, Current RTC of Sink. */
    uint16_t    wCRC16;    
} RF_FRAME_RESP_SLOT_RTC;

/**
* @brief  Type of RF frame that Node send to Sink.
*/
typedef enum
{
    RF_TYPE_INVALID_MIN = (uint8_t)0,
    RF_TYPE_REQ_JOIN, /* Node request join network. */
    RF_TYPE_UPLINK_DATA, /* Node uplink data to Sink. */
    RF_TYPE_REQ_SLOT_RTC, /* Node request correct the RTC. */
#if (UPLINK_WAKE == CUR_SYST)
    RF_TYPE_UC_DATA_ACK = (uint8_t)0xC1, /* !< Node response wake data ack frame to Sink. */
#endif
    RF_TYPE_INVALID_MAX,
    
#if (UPLINK_WAKE == CUR_SYST)
    RF_TYPE_BC_DATA = (uint8_t)0x21, /* !< Sink send broadcast data frame to Node. */
    RF_TYPE_BC_TIME, /* !< Sink send broadcast time frame to Node. */
    RF_TYPE_BC_NET_SETTINGS, /* !< Sink send broadcast network settings frame to Node. */

    RF_TYPE_UC_DATA = (uint8_t)0x41, /* !< Sink send unicast data frame to Node. */
#endif
} RF_FRAME_TYPE_TypeDef;

typedef struct
{
    RadioBW_t    eBW;
    RadioSF_t    eSF;
    RadioFEC_t    eFEC;
} BW_SF_FEC;

/**
* @brief  sort of current thread.
*/
typedef enum
{
    NETWORK_THREAD_INVALID_MIN = (uint8_t)0,
    NETWORK_THREAD_CSMA,
    NETWORK_THREAD_UPLINK,
    NETWORK_THREAD_WAKE,
    NETWORK_THREAD_INVALID_MAX,
} NETWORK_THREAD_Typedef;

/**
* @brief  Re-Calculate this when RF-Speed was changed.
*/
typedef struct
{
    uint16_t    wJoinReq;
    uint16_t    wJoinResp;
    uint16_t    wJoinWin;
    uint16_t    wUplinkRespOnAir;
    uint16_t    wUplinkResp;
#if (UPLINK_WAKE == CUR_SYST)
    uint16_t    wUplinkRespDiffOnAir;
    uint16_t    wWakeAddr;
    uint16_t    wBCTimeOnAir;
    uint16_t    wMinWakeExchange;
#endif
    uint16_t    wCADPeriod;
} TimeRFSpeed_t;

/**
* @brief  wake address frame.
*/
typedef struct
{
    NetAddr_t    tAddr; /* 0x0000=reserved, 0xFFFF=broadcast, 0x0001~0xFFFE=unicast. */
    uint8_t    byDataTime; /* 1bit=32ms, for time of wake data frame. */
} RF_FRAME_WAKE_ADDR;

/**
* @brief  Sink send broadcast data frame to Node.
*/
typedef struct
{
    uint8_t    byFlag;
    uint8_t    a_byDataBuf[MAX_LEN_UART_FRAME_DATA];
} RF_FRAME_BC_DATA;

/**
* @brief  Sink send broadcast time frame to Node.
*/
typedef struct
{
    uint8_t    byFlag;
    int32_t    lCurRTC; /* milliseconds, Current RTC of Sink. */
    uint16_t    wCRC16;    
} RF_FRAME_BC_TIME;

/**
* @brief  Sink send broadcast network settings frame to Node.
*/
typedef struct
{
    uint8_t    byFlag;
    uint8_t    byNetVer;    
    WNetConf_t    stWNetConf;
    uint16_t    wCRC16;    
} RF_FRAME_BC_NET_SETTINGS;

/**
* @brief  Sink send unicast data frame to Node.
*/
typedef RF_FRAME_BC_DATA    RF_FRAME_UC_DATA;

/**
* @brief  Node response wake data ack frame to Sink.
*/
typedef struct
{
    uint8_t    byFlag;
    NetAddr_t    tAddr;
    uint8_t    a_byDataBuf[MAX_LEN_UART_FRAME_DATA];
} RF_FRAME_UC_DATA_ACK;

/**
* @brief  sort of wake up.
*/
typedef enum
{
    WAKE_SORT_INVALID_MIN = (uint8_t)0,
    WAKE_SORT_NO_NEED,
    WAKE_SORT_NO_FLY_ZONE,
    WAKE_SORT_FREE,
    WAKE_SORT_INVALID_MAX,
} WAKE_SORT_Typedef;

/**
* @brief  status of RF RX ISR.
*/
typedef enum
{
    RF_RX_INVALID_MIN = (uint8_t)0,
    RF_RX_WAKE_ADDR,
    RF_RX_WAKE_DATA,
    RF_RX_INVALID_MAX,
} STATUS_RF_RX_Typedef;

/**
* @brief  result of RF RX done.
*/
typedef enum
{
    RF_RX_DONE_INVALID_MIN = (uint8_t)0,
    RF_RX_DONE_WAKE_DATA,
    RF_RX_DONE_BAD_ADDR,
    RF_RX_DONE_INVALID_MAX,
} RF_RX_DONE_Typedef;

/**
* @brief  Re-Calculate this when RF-Speed and Net-Settings were changed.
*/
typedef struct
{
    uint16_t    wUplinkReqTime;
#if (UPLINK_WAKE == CUR_SYST)
    WAKE_SORT_Typedef    tWakeSort;
    int8_t    chPreambleSize;
    int16_t    nQuotientBeaconWake;
    uint16_t    wPreambleTime;
    uint16_t    wUplinkExchange;
    uint16_t    wSlotUsed;
    uint16_t    wSlotIdle;
    uint16_t    wWakeExchange;
    uint16_t    wWakeGuard;
    int32_t    lWakeNoFly;
    int32_t    lReservedSlot;
#endif
} WNetRunSettings_t;

typedef int8_t (* PFN_THREAD)(void);


/* Private macro -------------------------------------------------------------*/
/* Separate time for 2 RF communication as: UPLINK<->WAKE, WAKE<->WAKE. */
#define SEPARATE_TIME    100 /* 100ms */

#define JOIN_NET_FLAG    0x1234


/* Private function prototypes --------------------------------------------------*/
static void RadioTxDone(void);
static void RadioTxTimeout(void);
static void RadioRxDone(uint16_t size, int16_t rssi, int8_t snr);
static void RadioRxTimeout(void);
static void RadioRxError(void);
static void RadioCadDone(bool bIsCadDetected);
static void RadioCadTimeout(void);

static void *GetRadioBufPtr(void);
static void PutRadioBufSize(uint8_t bySize);
static uint16_t GetRadioBuf(void *p_vBuf, uint16_t wBufSize, int32_t *p_lRxTime);

static void CalcTimeRFSpeed(void);
static void CalcNetRunSettings(void);

#if (UPLINK == CUR_SYST)
static void ChgRFSpeedFreq(void);
#else
static void ChgRFFreq(void);
static void ChgRFSpeed(void);
static uint16_t GetTime4WakeExchange(uint8_t byWakeDataSize, uint8_t byWakeAckSize);
static int32_t CalcWakeTime(int32_t lRTC);
#endif


/* Private Constants ----------------------------------------------------------*/
/**
* @brief  Table for mapping speed and BW+SF_FEC.
*/
const static BW_SF_FEC    s_astBwSfFec[] =
{
    {RF_BW_31250, RF_SF_12, RF_FEC_4_5}, /* BR_ON_AIR_66 */
    {RF_BW_62500, RF_SF_12, RF_FEC_4_5}, /* BR_ON_AIR_132 */
    {RF_BW_62500, RF_SF_11, RF_FEC_4_5}, /* BR_ON_AIR_243 */
    {RF_BW_62500, RF_SF_10, RF_FEC_4_5}, /* BR_ON_AIR_443 */
    {RF_BW_125000, RF_SF_10, RF_FEC_4_5}, /* BR_ON_AIR_887 */
    {RF_BW_125000, RF_SF_9, RF_FEC_4_5}, /* BR_ON_AIR_1602 */
    {RF_BW_125000, RF_SF_8, RF_FEC_4_5}, /* BR_ON_AIR_2876 */
    {RF_BW_125000, RF_SF_7, RF_FEC_4_5}, /* BR_ON_AIR_5084 */
    {RF_BW_250000, RF_SF_7, RF_FEC_4_5}, /* BR_ON_AIR_10168 */
    {RF_BW_500000, RF_SF_7, RF_FEC_4_5}, /* BR_ON_AIR_20334 */
};

/**
* @brief  Callback functions for radio events.
*/
const static RadioEvents_t    s_stRFEvents =
{
    .TxDone = RadioTxDone,
    .TxTimeout = RadioTxTimeout,
    .RxDone = RadioRxDone,
    .GetBufPtr = GetRadioBufPtr,
    .RxTimeout = RadioRxTimeout,
    .RxError = RadioRxError,
    .CadDone = RadioCadDone,
    .CadTimeout = RadioCadTimeout,    
};


/* Private variables -----------------------------------------------------------*/
PROCESS_NAME(NetworkProcess);

/**
* @brief  Cycle buffer to save radio packets.
* @note  MUST avoid race condition on "s_astRadioBuf" when (NUM_RADiO_BUF==1) that 
*            accessed both by ISR and process.
*/
static RADIO_BUF    s_astRadioBuf[NUM_RADIO_BUF];

#if (NUM_RADIO_BUF > 1)
static volatile int8_t    s_chWriteIndex = 0, s_chReadIndex = 0;
#endif

/**
* @brief  Save RSSI and SNR of the last received packet.
*/
static int16_t    s_nPacketRssi = 0;
static int8_t    s_chPacketSnr = 0;

/**
* @brief  TRUE=channel is clear, FALSE=channel is activity.
*/
static bool    s_bChClear;

/**
* @brief  Result of radio operation
*/
static volatile RF_Result_Typedef    s_tRFResult = RF_None;

/**
* @brief  Settings of Network-Process.
*/
static NetSettings_t    s_stNetSettings;

/**
* @brief  Count of uplink failed.
*/
static int8_t    s_chCorrectRTCFailed = 0;

/**
* @brief  Buffer for parsing the received packet by RF.
*/
static RADIO_BUF    s_stParseRFBuf;

static volatile NETWORK_THREAD_Typedef    s_tNetworkThread = NETWORK_THREAD_CSMA;

static TimeRFSpeed_t    s_stTimeRFSpeed;

static WNetRunSettings_t    s_stWNetRunSettings;

static PFN_THREAD    s_pfnThread = NULL;

#if (UPLINK == CUR_SYST)
static bool    s_bAddInterval = FALSE;
#else
static rtimer_clock_t    s_tCADEndTime;
static volatile STATUS_RF_RX_Typedef    s_tStatusRFRx = RF_RX_WAKE_ADDR;
static volatile RF_RX_DONE_Typedef    s_tRFRxDone = RF_RX_DONE_WAKE_DATA;
#endif

#if (!REL_VER)
static bool    s_bEnPrintWakeCnt = TRUE;
static uint32_t    s_ulRxWakeDataCnt = 0;
#endif

#if CATCH_NET_BLOCK
static int8_t    s_chCADCnt = 0;
static int16_t    s_nCADTOCnt = 0;
static int32_t    s_lAlarmMs;
static int32_t    s_lCurMs;
#endif


/**
* @brief  Get responsed type of RF frame. 
*/
#define GET_RF_TYPE_RESP(byType)    (0x80u + (byType))

/* Calculate RTC period between start time and end time. */
#define CALC_RTC_PERIOD(start, end)    \
    (((start) <= (end)) ? ((end) - (start)) : ((end) + MAX_MS_OF_DAY - (start)))

/* 1bit=32ms */
#define CONVERT_WAKE_DATA_TIME_2_MS(t)    ((t) << 5)


/*-------------------------------------------------------------------------*/
static void RadioTxDone(void)
{
    s_tRFResult = RF_Tx_Done;
    if (NETWORK_THREAD_UPLINK == s_tNetworkThread)
    {
        /* EXPLAIN:  for NO delay that turn on RF_RX by ISR is NOT process. */
        ASSERT(RF_IDLE == SX1278GetStatus());
        SX1278Receive(s_stTimeRFSpeed.wUplinkResp);
    }
    else
    {
        process_poll(&NetworkProcess);
    }

    return;
}

/*-------------------------------------------------------------------------*/
static void RadioTxTimeout(void)
{
    s_tRFResult = RF_Tx_Timeout;
    process_poll(&NetworkProcess); /* Inform Network-Process */
    PRINT_STR("radio TX timeout.\r\n");

    return;
}

/*-------------------------------------------------------------------------*/
static void RadioRxDone(uint16_t size, int16_t rssi, int8_t snr)
{
#if (UPLINK_WAKE == CUR_SYST)
    uint16_t    wWakeDataMs;
    const RF_FRAME_WAKE_ADDR    *p_stWakeAddr;
#endif

    SX1278SetSleep(); /* MUST stop RxContinuous mode manually. */
    s_tRFResult = RF_Rx_Done;

    /* Save data, RSSI and SNR of this received packet */
    PutRadioBufSize(size);
    s_nPacketRssi = rssi;
    s_chPacketSnr = snr;

#if (UPLINK == CUR_SYST)
    process_poll(&NetworkProcess);
#else
    if (NETWORK_THREAD_WAKE == s_tNetworkThread)
    {
        if (RF_RX_WAKE_ADDR == s_tStatusRFRx)
        {
            /* Check validity of the Wake Address frame. */
            s_stParseRFBuf.bySize = GetRadioBuf(s_stParseRFBuf.a_byBuf, RF_FIFO_SIZE, &s_stParseRFBuf.lRxTime);
            p_stWakeAddr = (const RF_FRAME_WAKE_ADDR *)s_stParseRFBuf.a_byBuf;
            if ( (p_stWakeAddr->tAddr == s_stNetSettings.tNetAddr) || 
                 (NET_ADDR_BROADCAST == p_stWakeAddr->tAddr) )
            {
                /* Address is valid, RX the Wake Data frame. */
                s_tStatusRFRx = RF_RX_WAKE_DATA;
                wWakeDataMs = CONVERT_WAKE_DATA_TIME_2_MS(p_stWakeAddr->byDataTime);
                ASSERT(RF_IDLE == SX1278GetStatus());
                SX1278Receive(wWakeDataMs);
                RIME_DBG(WAKE_DBG, "RF_ISR: valid addr: 0x%04X, Rx Wake Data.\r\n", p_stWakeAddr->tAddr);
            }
            else
            {
                /* Address is invalid, exit the Wake Communication. */
                s_tRFRxDone = RF_RX_DONE_BAD_ADDR;
                process_poll(&NetworkProcess);            
                RIME_DBG(WAKE_DBG, "RF_ISR: invalid addr: 0x%04X, Exit Wake Comm!\r\n", p_stWakeAddr->tAddr);
            }
        }
        else
        {
            /* Have RX the Wake Data frame, deliver it to process. */
            s_tStatusRFRx = RF_RX_WAKE_ADDR;
            s_tRFRxDone = RF_RX_DONE_WAKE_DATA;
            process_poll(&NetworkProcess);        
            RIME_DBG(WAKE_DBG, "RF_ISR: Received Wake Data, deliver to process.\r\n");
        }
    }
    else
    {
        process_poll(&NetworkProcess);
    }
#endif/*#if (UPLINK == CUR_SYST)*/

    return;
}

/*-------------------------------------------------------------------------*/
static void RadioRxTimeout(void)
{
    SX1278SetSleep(); /* MUST stop RxContinuous mode manually. */
    s_tRFResult = RF_Rx_Timeout;
    UNPRINT_STR("radio RX timeout.\r\n");

#if (UPLINK == CUR_SYST)
    process_poll(&NetworkProcess);
#else
    if (NETWORK_THREAD_WAKE == s_tNetworkThread)
    {
        /* Rx timeout, exit the Wake Communication. */
        s_tStatusRFRx = RF_RX_WAKE_ADDR;
        process_poll(&NetworkProcess);
        RIME_DBG(WAKE_DBG, "RF_ISR: Rx Timeout, Exit Wake Comm.\r\n");
    }
    else
    {
        process_poll(&NetworkProcess);
    }
#endif

    return;
}

/*-------------------------------------------------------------------------*/
static void RadioRxError(void)
{
    SX1278SetSleep(); /* MUST stop RxContinuous mode manually. */
    s_tRFResult = RF_Rx_Error;
    UNPRINT_STR("radio RX error.\r\n");

#if (UPLINK == CUR_SYST)
    process_poll(&NetworkProcess);
#else
    if (NETWORK_THREAD_WAKE == s_tNetworkThread)
    {
        /* Rx error, exit the Wake Communication. */
        s_tStatusRFRx = RF_RX_WAKE_ADDR;
        process_poll(&NetworkProcess);
        RIME_DBG(WAKE_DBG, "RF_ISR: Rx Error, Exit Wake Comm.\r\n");
    }
    else
    {
        process_poll(&NetworkProcess);
    }
#endif

    return;
}

/*-------------------------------------------------------------------------*/
static void RadioCadDone(bool bIsCadDetected)
{
    s_tRFResult = RF_Cad_Done;
    s_bChClear = bIsCadDetected ? FALSE: TRUE;

#if (UPLINK == CUR_SYST)
    process_poll(&NetworkProcess);
#else
    if (NETWORK_THREAD_WAKE == s_tNetworkThread)
    {
        /* Start CAD or RF_RX that triggered by RTC_ISR. */
        if (bIsCadDetected)
        {
            /* CAD OK, RX the WakeAddr frame. */
            ASSERT(RF_IDLE == SX1278GetStatus());
            SX1278Receive(s_stTimeRFSpeed.wWakeAddr);
            RIME_DBG(CAD_DBG | WAKE_DBG, "CAD_ISR: OK!  (start RX Wake Addr on %u)\r\n", RTIMER_NOW());
        }
        else
        {
            if (RTIMER_CLOCK_LT(RTIMER_NOW(), s_tCADEndTime))
            {
                /* Have time to start the next CAD. */
                ASSERT(RF_IDLE == SX1278GetStatus());
                SX1278SetCAD(s_stTimeRFSpeed.wCADPeriod);
                RIME_DBG(CAD_DBG, "CAD_ISR: now=%u, end=%u.\r\n", RTIMER_NOW(), s_tCADEndTime);
            #if CATCH_NET_BLOCK
                s_chCADCnt += 1;
            #endif
            }
            else
            {
                /* CAD Failed, exit the Wake Communication. */
                process_poll(&NetworkProcess);
                RIME_DBG( CAD_DBG | WAKE_DBG, 
                                  "CAD_ISR: FAILED!  (now=%u, end=%u)\r\n", RTIMER_NOW(), s_tCADEndTime );
            }
        }
    }
    else
    {
        process_poll(&NetworkProcess);
    }
#endif

    return;
}

/*-------------------------------------------------------------------------*/
static void RadioCadTimeout(void)
{
    s_tRFResult = RF_Cad_Timeout;
    process_poll(&NetworkProcess); /* Inform Network-Process */
    PRINT_STR("radio CAD timeout.\r\n");
#if CATCH_NET_BLOCK
    ++s_nCADTOCnt;
#endif

    return;
}

/**
  * @brief  Get the pointer of buffer saved the received packet.
  * @note  MUST call "PutRadioBufSize()" after call this function.
  * @note  Use this awkward procedure for saved RAM.
  * @param  None
  * @retval  void *: point to the buffer by this pointer.
  */
static void *GetRadioBufPtr(void)
{
#if (NUM_RADIO_BUF > 1)
    ASSERT(util_GetRoomCBuf(s_chWriteIndex, s_chReadIndex, NUM_RADIO_BUF) > 0);
    return s_astRadioBuf[s_chWriteIndex].a_byBuf;
#else
    return s_astRadioBuf[0].a_byBuf;
#endif
}

/*-------------------------------------------------------------------------*/
static void PutRadioBufSize(uint8_t bySize)
{
    ASSERT(bySize > 0);

#if (NUM_RADIO_BUF > 1)
    ASSERT(util_GetRoomCBuf(s_chWriteIndex, s_chReadIndex, NUM_RADIO_BUF) > 0);

    s_astRadioBuf[s_chWriteIndex].lRxTime = rtc_GetTimeMs();
    s_astRadioBuf[s_chWriteIndex].bySize = bySize;
    if (++s_chWriteIndex >= NUM_RADIO_BUF)
    {
        s_chWriteIndex = 0;
    }
#else
    halIntState_t    intState;

    /*@note MUST avoid race condition on "s_astRadioBuf" when (NUM_RADiO_BUF==1) that
	 accessed both by ISR and process. */
    HAL_ENTER_CRITICAL_SECTION(intState);
    s_astRadioBuf[0].lRxTime = rtc_GetTimeMs();
    s_astRadioBuf[0].bySize = bySize;
    HAL_EXIT_CRITICAL_SECTION(intState);
#endif

    return;
}

/*-------------------------------------------------------------------------*/
static uint16_t GetRadioBuf(void *p_vBuf, uint16_t wBufSize, int32_t *p_lRxTime)
{
    uint16_t    wCopySize;

    ASSERT(p_vBuf && p_lRxTime);

    wCopySize = 0;

#if (NUM_RADIO_BUF > 1)
    if (s_chWriteIndex != s_chReadIndex)
    {
        *p_lRxTime = s_astRadioBuf[s_chReadIndex].lRxTime;
        wCopySize = MIN(wBufSize, s_astRadioBuf[s_chReadIndex].bySize); /* Prevent overflow */
        memcpy(p_vBuf, s_astRadioBuf[s_chReadIndex].a_byBuf, wCopySize);
	
        if (++s_chReadIndex >= NUM_RADIO_BUF)
        {
            s_chReadIndex = 0;
        }
    }
#else
    halIntState_t    intState;

    /*@note MUST avoid race condition on "s_astRadioBuf" when (NUM_RADiO_BUF==1) that
	 accessed both by ISR and process. */
    HAL_ENTER_CRITICAL_SECTION(intState);
    if (s_astRadioBuf[0].bySize > 0)
    {
        *p_lRxTime = s_astRadioBuf[0].lRxTime;
        wCopySize = MIN(s_astRadioBuf[0].bySize, wBufSize); /* Prevent overflow */
        memcpy(p_vBuf, s_astRadioBuf[0].a_byBuf, wCopySize);
        s_astRadioBuf[0].bySize = 0; /* Set empty flag */
    }
    HAL_EXIT_CRITICAL_SECTION(intState);
#endif

    chip_LED2Off();

    return wCopySize;	
}

/*-------------------------------------------------------------------------*/
static uint16_t GetAckDelay(void)
{
    static const uint16_t    s_wAckDelay[10] = 
    {
        287, 156, 83, 50, 31, 22, 17, 16, 15, 15
    };

    return s_wAckDelay[s_stNetSettings.tBROnAir - BR_ON_AIR_START];
}

/*-------------------------------------------------------------------------*/
static uint16_t GetCorrectDelay(void)
{
    static const uint16_t    s_wCorrectDelay[10] = 
    {
        143, 78, 42, 25, 20, 16, 12, 9, 6, 3
    };

    return s_wCorrectDelay[s_stNetSettings.tBROnAir - BR_ON_AIR_START];
}

/**
  * @brief  Send frame to request join network or correct RTC.
  * @param  None
  * @retval  None
  */
static void ReqJoinOrRTC(void)
{
    RF_FRAME_REQ_SLOT_RTC    *p_stReqRTC;
    RF_FRAME_REQ_JOIN    *p_stReqJoin;

    packetbuf_clear(); /* ATTENTION: MUST call this before copy data otherwise may make error. */
    if (JOIN_NET_FLAG == s_stNetSettings.wJoinNetFlag)
    {
        /* Request slot and RTC. */
        p_stReqRTC = (RF_FRAME_REQ_SLOT_RTC *)packetbuf_dataptr();
        p_stReqRTC->byFlag = RF_TYPE_REQ_SLOT_RTC;
        p_stReqRTC->tAddr = s_stNetSettings.tNetAddr;
        packetbuf_set_datalen(sizeof(RF_FRAME_REQ_SLOT_RTC));
    }
    else
    {
        /* Request join into network. */
        p_stReqJoin = (RF_FRAME_REQ_JOIN *)packetbuf_dataptr();
        p_stReqJoin->byFlag = RF_TYPE_REQ_JOIN;
    #if (UPLINK == CUR_SYST)
        memcpy(p_stReqJoin->a_bySubnet, "RIME", 4);
    #else
        p_stReqJoin->byNetVer = s_stNetSettings.byNetVer;
    #endif
        p_stReqJoin->tDevID = chip_GetDevID();
        p_stReqJoin->wCRC16 =    \
            util_CRC16(p_stReqJoin, GET_ST_FLD_OFFSET(RF_FRAME_REQ_JOIN, wCRC16));
        packetbuf_set_datalen(sizeof(RF_FRAME_REQ_JOIN));    
    }

    return;
}

/**
  * @brief  Time of wait frame of join network or correct RTC.
  * @param  None
  * @retval  millisecond of wait time.
  */
static uint16_t WaitJoinOrRTC(void)
{
    /* EXPLAIN: add 80ms for Sink allocate addr and slot. */
    #define ALLOC_ADDR_TIME    80

#if (UPLINK == CUR_SYST)
    if (JOIN_NET_FLAG == s_stNetSettings.wJoinNetFlag)
    {
        return GetAckDelay() + SX1278GetTimeOnAir(sizeof(RF_FRAME_RESP_SLOT_RTC));
    }
    else
    {
        return GetAckDelay() + ALLOC_ADDR_TIME + SX1278GetTimeOnAir(sizeof(RF_FRAME_RESP_JOIN));
    }
#else
    return GetAckDelay() + ALLOC_ADDR_TIME + SX1278GetTimeOnAir(sizeof(RF_FRAME_RESP_JOIN_DIFF_VER));
#endif
}

/**
  * @brief  Check the responsed frame of join network or correct RTC.
  * @param  None
  * @retval  0=OK, -1=have no data, -2=flag error, -3=DevID error, -4=Bad RTC time, -5=CRC16 error.
  */
static int8_t CheckRespJoinOrRTC(void)
{
    bool    bNeedUpdate;
    int32_t    lRxTime;
    int32_t    lSinkMs;
    halIntState_t    intState;
    const RF_FRAME_RESP_JOIN    *p_stRespJoin;
#if (UPLINK== CUR_SYST)
    const RF_FRAME_RESP_SLOT_RTC    *p_stRespRTC;
#else
    const RF_FRAME_RESP_JOIN_DIFF_VER    *p_stRespDiff;
#endif

    /* Copy data from Radio Buffer into PacketBuf. */
    s_stParseRFBuf.bySize = GetRadioBuf(s_stParseRFBuf.a_byBuf, RF_FIFO_SIZE, &s_stParseRFBuf.lRxTime);
    if (0 == s_stParseRFBuf.bySize)
    {
        return -1; /* Have no data */
    }

    /* Deal with the responsed frame. */
    bNeedUpdate = FALSE;
#if (UPLINK == CUR_SYST)
    if (JOIN_NET_FLAG == s_stNetSettings.wJoinNetFlag)
    {
        p_stRespRTC = (const RF_FRAME_RESP_SLOT_RTC *)s_stParseRFBuf.a_byBuf;
        if (!( GET_RF_TYPE_RESP(RF_TYPE_REQ_SLOT_RTC) == p_stRespRTC->byFlag &&
                 p_stRespRTC->tAddr == s_stNetSettings.tNetAddr ))
        {
            return -2; /* Flag or Address error */
        }

        if ( p_stRespRTC->wCRC16 !=    \
             util_CRC16(p_stRespRTC, GET_ST_FLD_OFFSET(RF_FRAME_RESP_SLOT_RTC, wCRC16)) )
        {
            return -5; /* CRC16 error */
        }

        /* Check whether need to update the settings. */
        if (!( p_stRespRTC->byMaxPayload == s_stNetSettings.byMaxPayload &&
                p_stRespRTC->nNodeNum == s_stNetSettings.nNodeNum &&
                p_stRespRTC->wSlotLen == s_stNetSettings.wSlotLen &&
                p_stRespRTC->lSlotOffset == s_stNetSettings.lSlotOffset &&
                p_stRespRTC->lSlotInterval == s_stNetSettings.lSlotInterval ))
        {
            /* Set Net-Settings to this responsed value. */
            s_stNetSettings.byMaxPayload = p_stRespRTC->byMaxPayload;
            s_stNetSettings.nNodeNum = p_stRespRTC->nNodeNum;
            s_stNetSettings.wSlotLen = p_stRespRTC->wSlotLen;
            s_stNetSettings.lSlotOffset = p_stRespRTC->lSlotOffset;
            s_stNetSettings.lSlotInterval = p_stRespRTC->lSlotInterval;
            bNeedUpdate = TRUE;
        }

        lSinkMs = p_stRespRTC->lCurRTC;
    }
    else
#endif/*#if (UPLINK == CUR_SYST)*/
    {
        p_stRespJoin = (const RF_FRAME_RESP_JOIN *)s_stParseRFBuf.a_byBuf;
        if (GET_RF_TYPE_RESP(RF_TYPE_REQ_JOIN) != p_stRespJoin->byFlag)
        {
            return -2; /* Flag error */
        }
        if (chip_GetDevID() != p_stRespJoin->tDevID)
        {
            return -3; /* DevID error */
        }

        /* Set Net-Settings to this responsed value. */
    #if (UPLINK == CUR_SYST)
        if ( p_stRespJoin->wCRC16 !=    \
             util_CRC16(p_stRespJoin, GET_ST_FLD_OFFSET(RF_FRAME_RESP_JOIN, wCRC16)) )
        {
            return -5; /* CRC16 error */
        }

        s_stNetSettings.byMaxPayload = p_stRespJoin->byMaxPayload;
        s_stNetSettings.tNetAddr = p_stRespJoin->tAddr;
        s_stNetSettings.nNodeNum = p_stRespJoin->nNodeNum;
        s_stNetSettings.wJoinNetFlag = JOIN_NET_FLAG;
        s_stNetSettings.wSlotLen = p_stRespJoin->wSlotLen;
        s_stNetSettings.lSlotOffset = p_stRespJoin->lSlotOffset;
        s_stNetSettings.lSlotInterval = p_stRespJoin->lSlotInterval;
        bNeedUpdate = TRUE;
    #else
        if (sizeof(RF_FRAME_RESP_JOIN) == s_stParseRFBuf.bySize)
        {
            if ( p_stRespJoin->wCRC16 !=    \
                 util_CRC16(p_stRespJoin, GET_ST_FLD_OFFSET(RF_FRAME_RESP_JOIN, wCRC16)) )
            {
                return -5; /* CRC16 error */
            }

            ASSERT(p_stRespJoin->byNetVer == s_stNetSettings.byNetVer);
            if (p_stRespJoin->tAddr != s_stNetSettings.tNetAddr)
            {
                s_stNetSettings.tNetAddr = p_stRespJoin->tAddr;
                bNeedUpdate = TRUE;
            }
        }
        else
        {
            p_stRespDiff = (const RF_FRAME_RESP_JOIN_DIFF_VER *)p_stRespJoin;
            if ( p_stRespDiff->wCRC16 !=    \
                 util_CRC16(p_stRespDiff, GET_ST_FLD_OFFSET(RF_FRAME_RESP_JOIN_DIFF_VER, wCRC16)) )
            {
                return -5; /* CRC16 error */
            }

            s_stNetSettings.byNetVer = p_stRespDiff->byNetVer;
            s_stNetSettings.tNetAddr = p_stRespDiff->tAddr;
            memcpy(&s_stNetSettings.stWNetConf, &p_stRespDiff->stWNetConf, sizeof(WNetConf_t));
            bNeedUpdate = TRUE;
        }
    #endif

        lSinkMs = p_stRespJoin->lCurRTC;
    }    

    if (MAX_MS_OF_DAY <= lSinkMs)
    {
        return -4; /* Bad RTC time. */
    }

    /* Synchronize RTC to sink. */
    lSinkMs += SX1278GetTimeOnAir(s_stParseRFBuf.bySize);
    lSinkMs += GetCorrectDelay();

    /* ATTENTION: do NOT break the "GetRTC->CalculateRTC->SetRTC"! */
    HAL_ENTER_CRITICAL_SECTION(intState);
    lRxTime = rtc_GetTimeMs() - s_stParseRFBuf.lRxTime; /* Compensate time tetween RX to current. */
    if (lRxTime < 0)
    {
        lRxTime += MAX_MS_OF_DAY; /* Wrap on 00:00:00.000 */
    }

    lSinkMs += lRxTime;
    if (MAX_MS_OF_DAY <= lSinkMs)
    {
        lSinkMs -= MAX_MS_OF_DAY; /* Wrap on 00:00:00.000 */
    }

    if (rtc_SetTimeSec(lSinkMs / 1000) < 0)
    {
        RIME_DBG(RIME_DBG_ON | RIME_DBG_LEVEL_SERIOUS, "CheckRespJoinOrRTC(): Set Sec error\r\n");
    }
    if (rtc_SetMs(lSinkMs % 1000) < 0)
    {
        RIME_DBG(RIME_DBG_ON | RIME_DBG_LEVEL_SERIOUS, "CheckRespJoinOrRTC(): Set Ms error\r\n");
    }
    HAL_EXIT_CRITICAL_SECTION(intState);

    /* Save into EEPROM and update settings of Comm2Trm-Process. */
    if (bNeedUpdate)
    {
        CalcNetRunSettings();
        ss_SaveNetSettings(&s_stNetSettings);
        comm2trm_UpdateNetSettings(&s_stNetSettings);
    }
	
    return 0; /* OK */
}

/**
  * @brief  Calculate the NEXT uplink time.
  * @param  None.
  * @retval  uint32_t  milliseconds of the NEXT uplink time.
  */
static int32_t CalcUplinkTime(int32_t lRTCMs)
{
    #define MIN_ALARM_TH    10 /* 10ms */

    int32_t    lAlarmMs, lCurMs;

    lCurMs = lRTCMs;

#if (UPLINK == CUR_SYST)
    /* Calculate the NEXT alarm time by: "(addr -1 ) * slot + offset + interval * N" */
    if (lCurMs <= s_stNetSettings.lSlotOffset)
    {
        lAlarmMs = 0;
    }
    else
    {
        lAlarmMs = (lCurMs - s_stNetSettings.lSlotOffset) / s_stNetSettings.lSlotInterval; /* N */
    }
    lAlarmMs = (s_stNetSettings.tNetAddr - 1) * s_stNetSettings.wSlotLen +    \
                     s_stNetSettings.lSlotOffset + s_stNetSettings.lSlotInterval * lAlarmMs;
    if (lAlarmMs <= lCurMs)
    {
        lAlarmMs += s_stNetSettings.lSlotInterval;
    }
    ASSERT(lAlarmMs > lCurMs); /* Catch the error of theory. */

    /* Check whether need to keep silence for 1 interval. */
    if (s_bAddInterval)
    {
        s_bAddInterval = FALSE; /* Clear flag */
        lAlarmMs += s_stNetSettings.lSlotInterval;
        RIME_DBG(CALC_UPLINK_DBG, "CalcUplinkTime(): Add 1 interval to keep silence!\r\n");
    }
#else
    /* Next uplink time = (addr -1 ) * slot + uplink period * N */
    lAlarmMs = lCurMs / s_stNetSettings.stWNetConf.lUplinkPeriod; /* N */
    lAlarmMs = (s_stNetSettings.tNetAddr - 1) * s_stNetSettings.stWNetConf.wSlotLen +    \
                     s_stNetSettings.stWNetConf.lUplinkPeriod * lAlarmMs;
    if (lAlarmMs <= lCurMs)
    {
        lAlarmMs += s_stNetSettings.stWNetConf.lUplinkPeriod;
    }
    ASSERT(lCurMs < lAlarmMs); /* Catch the error of theory. */
#endif

    /* Check whether the Alarm-Period(AlarmMs - CurrentMs) is too short. */
    if ((lAlarmMs - lCurMs) <= MIN_ALARM_TH)
    {
    #if (UPLINK == CUR_SYST)
        lAlarmMs += s_stNetSettings.lSlotInterval;
    #else
        lAlarmMs += s_stNetSettings.stWNetConf.lUplinkPeriod;
    #endif
        RIME_DBG(CALC_UPLINK_DBG, "CalcUplinkTime(): Add 1 interval that alarm period too short!\r\n");
    }

    /* Check whether wrap on 00:00:00.000. */
    if (MAX_MS_OF_DAY <= lAlarmMs)
    {
        lAlarmMs -= MAX_MS_OF_DAY;
    }

    RIME_DBG(CALC_UPLINK_DBG, "CalcUplinkTime(): CurMs=%ld, AlarmMs=%ld.\r\n", lCurMs, lAlarmMs);

    return lAlarmMs;
}

/**
  * @brief  Correct RTC of node to sink.
  * @param  int32_t lSinkMs    sink time on the RF packet.
  * @param  int32_t lRxTime    time of received the RF packet.
  * @param  uint16_t wTimeOnAir    time on air of the RF packet.
  * @retval  none.
  */
static void CorrectRTC(int32_t lSinkMs, int32_t lRxTime, uint16_t wTimeOnAir)
{
    #define CORRECT_RTC_TH    10 /* 10ms */

    int32_t    lCurMs;
    int32_t    lSetMs;
    int32_t    lDelayMs;
    int32_t    lDiff;
    int32_t    lDiffAbs;
    halIntState_t    intState;
#if (UPLINK == CUR_SYST)
    int16_t    nSafeGuard;
#endif

    static const int32_t    s_lDiffMin = (int32_t)(-1) * (MAX_MS_OF_DAY - 1000);
    static const int32_t    s_lDiffMax = (int32_t)(MAX_MS_OF_DAY - 1000);

    ASSERT( (0 <= lSinkMs && lSinkMs < MAX_MS_OF_DAY) && 
                  (0 <= lRxTime && lRxTime < MAX_MS_OF_DAY) );

    /* ATTENTION: do NOT break the "Get->Calc->Set RTC Alarm"(needs 1ms) by ISR! */
    HAL_ENTER_CRITICAL_SECTION(intState);

    lCurMs = rtc_GetTimeMs();

    /* Calculate the right set time. */
    lDelayMs = lCurMs - lRxTime;
    if (lDelayMs < 0)
    {
        lDelayMs += MAX_MS_OF_DAY; /* Wrap on 00:00:00.000 */
    }

    lSetMs = lSinkMs + lDelayMs + wTimeOnAir + GetCorrectDelay();
    if (MAX_MS_OF_DAY <= lSetMs)
    {
        lSetMs -= MAX_MS_OF_DAY; /* Wrap on 00:00:00.000 */
    }
    RIME_DBG( RIME_DBG_OFF,
                      "SetMs=%ld, SinkMs=%ld, DelayMs=%ld, TimeOnAir=%d, Compensate=%d.\r\n",
                      lSetMs, lSinkMs, lDelayMs, wTimeOnAir, GetCorrectDelay() );

    /* Calculate the differece between set time and current time. */
    lDiff = lSetMs - lCurMs;
    if (s_lDiffMax < lDiff)
    {
        lDiff -= MAX_MS_OF_DAY; /* Node wrap on 00:00:00.000 */
    }
    else if (lDiff < s_lDiffMin)
    {
        lDiff += MAX_MS_OF_DAY; /* Sink wrap on 00:00:00.000 */
    }
    lDiffAbs = labs(lDiff);

#if (UPLINK == CUR_SYST)
    nSafeGuard = s_stNetSettings.wSlotLen -    \
                        (s_stWNetRunSettings.wUplinkReqTime + s_stTimeRFSpeed.wUplinkResp);
    if (nSafeGuard < CORRECT_RTC_TH)
    {
        nSafeGuard = CORRECT_RTC_TH;
    }
	
    if (nSafeGuard < lDiffAbs)
    {
        /* EXCEPTION: Sink resetted RTC or both RTC is failed, synchronize RTC of node and 
             keep silence on 1 interval for don't disturb other nodes. */
        s_bAddInterval = TRUE;
    }
#endif

    /* Set RTC according to differece. */
    if (lDiffAbs < CORRECT_RTC_TH) /* 0~(CORRECT_RTC_TH-1) */
    {
        null();
    }
    else if (lDiffAbs < 1000) /* CORRECT_RTC_TH~999 */
    {
        /* Performs a shift to correct ms to Sink. */
        if (rtc_SetMs(lDiff) < 0)
        {
            RIME_DBG( RIME_DBG_ON | RIME_DBG_LEVEL_SERIOUS, 
                              "CorrectRTC: rtc_SetMs(lDiff) error.\r\n" );
        }
        RIME_DBG( CORRECT_RTC_DBG, "SetMs=%ld, CurMs=%ld, Diff=%ld, set ms=%ld\r\n", 
                          lSetMs, lCurMs, lDiff, lDiff );
    }
    else /* 1000~MAX */
    {
        /* e.g SinkMs=1100, set Sec=1. */
        if (rtc_SetTimeSec(lSetMs / 1000) < 0) 
        {
            RIME_DBG( RIME_DBG_ON | RIME_DBG_LEVEL_SERIOUS, 
                              "CorrectRTC: rtc_SetTimeSec(lSetMs / 1000) error\r\n" );
        }

        /* e.g Performs a positive of 100ms */
        if (rtc_SetMs(lSetMs % 1000) < 0)
        {
            RIME_DBG( RIME_DBG_ON | RIME_DBG_LEVEL_SERIOUS, 
                              "CorrectRTC: rtc_SetMs(lSetMs % 1000) error\r\n" );
        }
        RIME_DBG( CORRECT_RTC_DBG,
                           "SetMs=%ld, CurMs=%ld, Diff=%ld, set s=%ld, set ms=%d\r\n",
                           lSetMs, lCurMs, lDiff, (lSetMs / 1000), (int16_t)(lSetMs % 1000) );
    }

    HAL_EXIT_CRITICAL_SECTION(intState);

    return;
}

/*-------------------------------------------------------------------------*/
static void SetNextRTCAlarm(void)
{
    /* Prevent error of the RTC may miss alarm if the interval between current time and next
        alarm is too short, like as: current=4570ms, wake=4577ms, the interval=7ms is too short. */
    #define COMPENSATE_CALC_ERR    20 /* 20ms */

    int32_t    lAlarmMs, lCurMs;
    halIntState_t    intState;

    /* ATTENTION: do NOT break the "Get->Calc->Set RTC Alarm" by ISR! */
    HAL_ENTER_CRITICAL_SECTION(intState);

    lCurMs = rtc_GetTimeMs();

#if (UPLINK == CUR_SYST)
    lAlarmMs = CalcUplinkTime(lCurMs + COMPENSATE_CALC_ERR);
    s_tNetworkThread = NETWORK_THREAD_UPLINK;
#else
    int32_t    lUplinkMs, lWakeMs, lUplinkPeriod, lWakePeriod;

    lUplinkPeriod = LONG_MAX;
    lWakePeriod = LONG_MAX;
    if (0 != s_stNetSettings.stWNetConf.byUplinkPayload)
    {
        lUplinkMs = CalcUplinkTime(lCurMs + COMPENSATE_CALC_ERR);
        lUplinkPeriod = CALC_RTC_PERIOD(lCurMs, lUplinkMs);
    }
    if (WAKE_SORT_NO_NEED != s_stWNetRunSettings.tWakeSort)
    {
        lWakeMs = CalcWakeTime(lCurMs + COMPENSATE_CALC_ERR);
        lWakePeriod = CALC_RTC_PERIOD(lCurMs, lWakeMs);
    }
    ASSERT(lUplinkPeriod != lWakePeriod);

    if (lUplinkPeriod < lWakePeriod)
    {
        lAlarmMs = lUplinkMs;
        s_tNetworkThread = NETWORK_THREAD_UPLINK;
    }
    else
    {
        lAlarmMs = lWakeMs;
        s_tNetworkThread = NETWORK_THREAD_WAKE;
    }
#endif

    if (rtc_SetAlarm(lAlarmMs) < 0)
    {
        RIME_DBG(RIME_DBG_ON | RIME_DBG_LEVEL_SERIOUS, "SetNextRTCAlarm(): set RTC alarm error!\r\n");
    }

    HAL_EXIT_CRITICAL_SECTION(intState);
    RIME_DBG( CALC_UPLINK_DBG | CALC_WAKE_DBG,
                      "current=%ldms, next alarm=%ldms, for %s.\r\n",
                      lCurMs, 
                      lAlarmMs,
                      (NETWORK_THREAD_UPLINK == s_tNetworkThread) ? "UPLINK" : "WAKE" );

#if CATCH_NET_BLOCK
    s_lCurMs = lCurMs;
    s_lAlarmMs = lAlarmMs;
#endif

    return;
}

/*-------------------------------------------------------------------------*/
static uint16_t CalcRandDelay(int8_t chBE, uint16_t wWinTime)
{
    uint16_t    wDiv;
    uint16_t    wRand;
    uint16_t    wRandMs;

    ASSERT(chBE <= 16);

    wDiv = (1 << chBE) - 1;
    wRand = util_GetRand16() % wDiv;
    wRandMs = wRand * wWinTime;

    return wRandMs;
}

/**
  * @brief  Thread for join into network or correct the RTC.
  * @param  none.
  * @retval  PT_ENDED=end of thread; PT_YIELDED=yield of thread.
  */
static int8_t JoinOrRTCThread(int8_t *p_chJoinStatus)
{
    #define MAX_JOIN_NUM    5
    #define MAX_TRY_CAD    10
    #define CAD_OK_NUM    1
    #define MIN_BE    3 /* ConcurrentRequestNumber=2^3-1=7 */
    #define MAX_BE    7 /* ConcurrentRequestNumber=2^7-1=127, MAX=65535/500=131 */

    /* ATTENTION: stack variables can NOT cross any "XX_YIELD()". */
    uint16_t    l_wDelayMs;

    static int8_t    s_chCADOkNum;
    static int8_t    s_chTryCADNum;
    static int8_t    s_chCADBE;
    static int8_t    s_chJoinBE;
    static int8_t    s_chRxNoise;
    static int8_t    s_chJoinNum;
    static rtimer_clock_t    s_tTimeEnd;
    static struct pt    s_tPT; /* For this protothread */
    static struct etimer    s_stETimer; /* For delay */

    PT_BEGIN(&s_tPT);

    s_tNetworkThread = NETWORK_THREAD_CSMA;

    /* Make the request frame based on packetbuf. */
    ReqJoinOrRTC();
    RIME_DBG(JOIN_DBG, "Join window=%dms\r\n", s_stTimeRFSpeed.wJoinWin);

    /* Join network or Correct RTC. */
    s_chJoinBE = MIN_BE;
    for (s_chJoinNum = 0; s_chJoinNum < MAX_JOIN_NUM; ++s_chJoinNum)
    {
        /* LBT(Listen Before Talk) */
        s_chTryCADNum = 0;
        s_chCADBE = MIN_BE;
        s_chCADOkNum = CAD_OK_NUM;
        while (0 < s_chCADOkNum)
        {
            ASSERT(RF_IDLE == SX1278GetStatus());		
            SX1278SetCAD(s_stTimeRFSpeed.wCADPeriod);
            PT_YIELD(&s_tPT);
            if ((RF_Cad_Done == s_tRFResult) && s_bChClear)
            {
                --s_chCADOkNum; /* Channel is clear! */
            }
            else
            {
                if (MAX_TRY_CAD < ++s_chTryCADNum)
                {
                    *p_chJoinStatus = -1; /* Failed on CAD, exit thread. */
                    RIME_DBG(JOIN_DBG, "Join: failed, MAX(%d) < Try CAD!\r\n", MAX_TRY_CAD);
                    goto  exit_join;
                }

                /* Get random value between (2^BE - 1), multi TX window. */
                l_wDelayMs = CalcRandDelay(s_chCADBE, s_stTimeRFSpeed.wJoinWin);
                etimer_set(&s_stETimer, MS_2_CLOCK(l_wDelayMs));
                RIME_DBG( JOIN_DBG,
                                  "Join: CAD dirty, rand=%d, delay=%ums, BE=%d\r\n", 
                                  l_wDelayMs / s_stTimeRFSpeed.wJoinWin, l_wDelayMs, s_chCADBE );

                PT_YIELD(&s_tPT);
                s_chCADOkNum = CAD_OK_NUM;
                s_chCADBE = MIN(s_chCADBE + 1, MAX_BE);
            }
        }/*while (0 < s_chCADOkNum)*/

        /* TX the request frame since the channel is clear. */
        ASSERT(RF_IDLE == SX1278GetStatus());
        SX1278Send(packetbuf_dataptr(), packetbuf_datalen());
        PT_YIELD(&s_tPT);
        
        /* RX the responsed frame with timeout. */
        s_chRxNoise = 0;
        s_tTimeEnd = RTIMER_NOW() + s_stTimeRFSpeed.wJoinResp;
        while (RTIMER_CLOCK_LT(RTIMER_NOW(), s_tTimeEnd)) /* TimeOut<=32768 */
        {
            ASSERT(RF_IDLE == SX1278GetStatus());
            SX1278Receive(s_stTimeRFSpeed.wJoinResp);
            PT_YIELD(&s_tPT);

            if (RF_Rx_Done == s_tRFResult)
            {
                if (0 <= CheckRespJoinOrRTC())
                {
                    if (0 < s_chRxNoise)
                    {
                        RIME_DBG(JOIN_DBG, "Join: OK, have RX noise=%d\r\n", s_chRxNoise);
                    }
                    else
                    {
                        RIME_DBG(JOIN_DBG, "Join: OK!\r\n");
                    }

                    *p_chJoinStatus = 0; /* OK!Get the desired responsed frame, exit request! */
                    goto  exit_join;
                }
                else
                {
                    ++s_chRxNoise;
                }
            }
            else if (RF_Rx_Timeout == s_tRFResult)
            {
                RIME_DBG(JOIN_DBG, "Join: Rx TimeOut.\r\n");
            }
        }/*while (RTIMER_CLOCK_LT)*/

        /* Delay a random time when have NOT RX ack. */
        l_wDelayMs = CalcRandDelay(s_chJoinBE, s_stTimeRFSpeed.wJoinWin);
        etimer_set(&s_stETimer, MS_2_CLOCK(l_wDelayMs));
        RIME_DBG( JOIN_DBG,
                          "Join: RX noise=%d, rand=%d, delay=%ums\r\n", 
                          s_chRxNoise, l_wDelayMs / s_stTimeRFSpeed.wJoinWin, l_wDelayMs );

        PT_YIELD(&s_tPT);
        s_chJoinBE = MIN(s_chJoinBE + 1, MAX_BE);
    }/*for*/

    *p_chJoinStatus = -2; /* Failed on RX of Sink ACK. */
    RIME_DBG(JOIN_DBG, "Join: failed!\r\n\r\n");

exit_join:
    PT_END(&s_tPT);
}

/**
  * @note  This function for reducing UPLINK thread complexity.
  */
static int8_t ProcUplinkAck(void)
{
    uint16_t    wTimeOnAir;
    const RF_FRAME_UPLINK_ACK    *p_stAck;
#if (UPLINK_WAKE == CUR_SYST)
    const RF_FRAME_UPLINK_ACK_DIFF_VER    *p_stAckDiff;
#endif

    /* Copy data from Radio Buffer into PacketBuf. */
    s_stParseRFBuf.bySize = GetRadioBuf(s_stParseRFBuf.a_byBuf, RF_FIFO_SIZE, &s_stParseRFBuf.lRxTime);
    if (0 == s_stParseRFBuf.bySize)
    {
        return -1; /* Size is 0 */
    }

    /* Check validity of this packet. */
    p_stAck = (const RF_FRAME_UPLINK_ACK *)s_stParseRFBuf.a_byBuf;
    if (GET_RF_TYPE_RESP(RF_TYPE_UPLINK_DATA) != p_stAck->byFlag)
    {
        return -2; /* Bad flag */
    }

    if (p_stAck->tAddr != s_stNetSettings.tNetAddr)
    {
        return -3; /* Bad address */
    }

    if (!(0 <= p_stAck->lCurRTC && p_stAck->lCurRTC < MAX_MS_OF_DAY))
    {
        return -4; /* Bad RTC of sink */
    }

    wTimeOnAir = s_stTimeRFSpeed.wUplinkRespOnAir;
#if (UPLINK == CUR_SYST)
    if (p_stAck->wCRC16 != util_CRC16(p_stAck, GET_ST_FLD_OFFSET(RF_FRAME_UPLINK_ACK, wCRC16)))
    {
        return -5; /* Bad CRC16 */
    }
#else
    if (sizeof(RF_FRAME_UPLINK_ACK) == s_stParseRFBuf.bySize) /* Same NetVer */
    {
        if (p_stAck->wCRC16 != util_CRC16(p_stAck, GET_ST_FLD_OFFSET(RF_FRAME_UPLINK_ACK, wCRC16)))
        {
            return -5; /* Bad CRC16 */
        }
    }
    else /* Different NetVer */
    {
        wTimeOnAir = s_stTimeRFSpeed.wUplinkRespDiffOnAir;
        p_stAckDiff = (const RF_FRAME_UPLINK_ACK_DIFF_VER *)p_stAck;
        if (p_stAckDiff->wCRC16 != util_CRC16(p_stAckDiff, GET_ST_FLD_OFFSET(RF_FRAME_UPLINK_ACK_DIFF_VER, wCRC16)))
        {
            return -5; /* Bad CRC16 */
        }

        /* Update the settings.*/
        memcpy(&s_stNetSettings.stWNetConf, &p_stAckDiff->stWNetConf, sizeof(WNetConf_t));
        s_stNetSettings.byNetVer = p_stAckDiff->byNetVer;
        CalcNetRunSettings();
        SetNextRTCAlarm(); /* Reset the NEXT RTC alarm time when changed "NetRunSettings".*/
        ss_SaveNetSettings(&s_stNetSettings);
        comm2trm_UpdateNetSettings(&s_stNetSettings);
    }
#endif

    /* Correct our RTC, this ACK packet is valid. */
    CorrectRTC(p_stAck->lCurRTC, s_stParseRFBuf.lRxTime, wTimeOnAir);

    return 0;
}

/**
  * @brief  Thread for uplink the data frame to Sink.
  * @param  none.
  * @retval  PT_ENDED=end of thread; PT_YIELDED=yield of thread.
  */
static int8_t UplinkThread(void)
{
    /* ATTENTION: stack variables can NOT cross any "XX_YIELD()". */
    int8_t    chResult;
    uint8_t    bySize;
    uint16_t    wRepeatTxTime;
    RF_FRAME_UPLINK_DATA    *p_stData;

    static bool    s_bNeedTx;
    static rtimer_clock_t    s_tTimeEnd;
    static struct pt    s_tPT;
#if (RIME_DBG_ON == UPLINK_DBG)
    static int8_t    s_chUplinkTestCnt = 0;
#endif

    PT_BEGIN(&s_tPT);

    RIME_DBG(UPLINK_DBG, "UplinkThread(): start on %ldms.\r\n", rtc_GetTimeMs());

    /* Make the RF uplink packet. */
    packetbuf_clear(); /* ATTENTION: MUST call this before copy data otherwise may make error. */
    p_stData = (RF_FRAME_UPLINK_DATA *)packetbuf_dataptr();

#if (UPLINK == CUR_SYST)
    bySize = comm2trm_GetUplinkData(p_stData->a_byDataBuf, s_stNetSettings.byMaxPayload);
#else
    bySize = comm2trm_GetUplinkData(p_stData->a_byDataBuf, s_stNetSettings.stWNetConf.byUplinkPayload);
#endif
    if (0 == bySize)
    {
        goto exit_uplink; /* Have no data to uplink */
    }

    p_stData->byFlag = RF_TYPE_UPLINK_DATA;
#if (UPLINK_WAKE == CUR_SYST)
    p_stData->byNetVer = s_stNetSettings.byNetVer;
#endif
    p_stData->tAddr = s_stNetSettings.tNetAddr;
    bySize += GET_ST_FLD_OFFSET(RF_FRAME_UPLINK_DATA, a_byDataBuf); /* Add Flag+Addr */
    packetbuf_set_datalen(bySize);

    /* repeat tx time=allowable slot time-(tx uplink packet+rx ack packet), MUST < 32768. */
#if (UPLINK == CUR_SYST)
    wRepeatTxTime = s_stNetSettings.wSlotLen -    \
                              (s_stWNetRunSettings.wUplinkReqTime + s_stTimeRFSpeed.wUplinkResp);
#else
    wRepeatTxTime = s_stWNetRunSettings.wSlotUsed -    \
                              (s_stWNetRunSettings.wUplinkReqTime + s_stTimeRFSpeed.wUplinkResp);
#endif
    ASSERT(wRepeatTxTime < 32768);
    /* EXPLAIN: Tt's right even if "s_tTimeEnd" is negative(as 65535). */
    s_tTimeEnd = wRepeatTxTime + RTIMER_NOW();
 
#if (RIME_DBG_ON == UPLINK_DBG)
    s_chUplinkTestCnt = 0;
#endif
    s_bNeedTx = TRUE;
    do
    {
        /* EXPLAIN: the ISR of RF_Tx_Done would turn on RF RX. */
        ASSERT(RF_IDLE == SX1278GetStatus());
        SX1278Send(packetbuf_dataptr(), packetbuf_datalen());
    #if (RIME_DBG_ON == UPLINK_DBG)
        ++s_chUplinkTestCnt;
    #endif

        /* Yield until: RF_Rx_Done or RF_Rx_Timeout or RF_Rx_Error. */
        PT_YIELD(&s_tPT);

        /* Process the received packet. */
        if (RF_Rx_Done == s_tRFResult)
        {
            if (0 <= (chResult = ProcUplinkAck()))
            {
                s_bNeedTx = FALSE; /* Finished job. */
                s_chCorrectRTCFailed = 0;
            #if (RIME_DBG_ON == UPLINK_DBG)
                RIME_DBG( UPLINK_DBG | RIME_DBG_STATE, 
                                  "UplinkThread(): OK, count=%d, on %ldms.\r\n", 
                                  s_chUplinkTestCnt, rtc_GetTimeMs() );
            #endif
            }
            else
            {
                RIME_DBG( UPLINK_DBG | RIME_DBG_STATE, 
                                  "UplinkThread(): ACK error: %d, on %ldms.\r\n", 
                                  chResult, rtc_GetTimeMs() );
            }
        }
        else
        {
            RIME_DBG( UPLINK_DBG | RIME_DBG_TRACE, 
                              "UplinkThread(): %s.\r\n", 
                              (RF_Rx_Timeout == s_tRFResult) ? "RF_Rx_Timeout" : "RF_Rx_Error" );
        }
    } while (s_bNeedTx && RTIMER_CLOCK_LT(RTIMER_NOW(), s_tTimeEnd));
    
    if (s_bNeedTx)
    {
    #if (RIME_DBG_ON == UPLINK_DBG)
        RIME_DBG( UPLINK_DBG | RIME_DBG_STATE, 
                          "UplinkThread(): failed, count=%d, on %ldms.\r\n", 
                          s_chUplinkTestCnt, rtc_GetTimeMs() );
    #endif
        ++s_chCorrectRTCFailed;
    }

exit_uplink:
    PT_END(&s_tPT);
}


#if (UPLINK_WAKE == CUR_SYST)
/**
  * @note  This function for reducing WAKE thread complexity.
  */
static void ProcWakeData(void)
{
    uint8_t    byHeadSize;
    uint8_t    byDataSize;
    uint8_t    *p_byDataBuf;
    uint16_t    wCRC16;
    RF_FRAME_TYPE_TypeDef    eType;
    RF_FRAME_UC_DATA_ACK    *p_stUCDataAck;
    const RF_FRAME_BC_TIME    *p_stBCTime;
    const RF_FRAME_BC_NET_SETTINGS    *p_stBCNetSettings;

    /* Set the default is: have NO need to send ACK. */
    packetbuf_set_datalen(0);

    s_stParseRFBuf.bySize = GetRadioBuf(s_stParseRFBuf.a_byBuf, RF_FIFO_SIZE, &s_stParseRFBuf.lRxTime);
    if (0 == s_stParseRFBuf.bySize)
    {
        return; /* Size is 0 */
    }

    eType = (RF_FRAME_TYPE_TypeDef)s_stParseRFBuf.a_byBuf[0];
    switch (eType)
    {
        case RF_TYPE_BC_DATA:
        {
            /* Deliver Wake Data to Comm2Trm Process. */
            byHeadSize = GET_ST_FLD_OFFSET(RF_FRAME_BC_DATA, a_byDataBuf);
            byDataSize = s_stParseRFBuf.bySize - byHeadSize;
            p_byDataBuf = &s_stParseRFBuf.a_byBuf[byHeadSize];
            comm2trm_RxWakeData(p_byDataBuf, byDataSize);
        }
            break;
        case RF_TYPE_BC_TIME:
        {
            p_stBCTime = (const RF_FRAME_BC_TIME *)s_stParseRFBuf.a_byBuf;
            wCRC16 = util_CRC16(p_stBCTime, GET_ST_FLD_OFFSET(RF_FRAME_BC_TIME, wCRC16));
            if ( (0 <= p_stBCTime->lCurRTC && p_stBCTime->lCurRTC < MAX_MS_OF_DAY) &&
                 (p_stBCTime->wCRC16 == wCRC16) )
            {
                CorrectRTC(p_stBCTime->lCurRTC, s_stParseRFBuf.lRxTime, s_stTimeRFSpeed.wBCTimeOnAir);
                s_chCorrectRTCFailed = 0;
                RIME_DBG(REPAIR_BROKEN_DBG, "Rx BEACON, CorrectRTCFailed=0.\r\n");
            }
            else
            {
                RIME_DBG(CORRECT_RTC_DBG, "ProcWakeData: CRC16 of BC_TIME is error.\r\n");
            }
        }
            break;
        case RF_TYPE_BC_NET_SETTINGS:
        {
            p_stBCNetSettings = (const RF_FRAME_BC_NET_SETTINGS *)s_stParseRFBuf.a_byBuf;
            if ( p_stBCNetSettings->wCRC16 ==    \
                 util_CRC16(p_stBCNetSettings, GET_ST_FLD_OFFSET(RF_FRAME_BC_NET_SETTINGS, wCRC16)) )
            {
                if (p_stBCNetSettings->byNetVer != s_stNetSettings.byNetVer)
                {
                    /* Update the settings.*/
                    memcpy(&s_stNetSettings.stWNetConf, &p_stBCNetSettings->stWNetConf, sizeof(WNetConf_t));
                    s_stNetSettings.byNetVer = p_stBCNetSettings->byNetVer;
                    CalcNetRunSettings();
                    SetNextRTCAlarm(); /* Reset the NEXT RTC alarm time when changed "NetRunSettings".*/
                    ss_SaveNetSettings(&s_stNetSettings);
                    comm2trm_UpdateNetSettings(&s_stNetSettings);                
                }/*if(byNetVer)*/
            }/*if(wCRC16)*/
        }/*case*/
            break;
        case RF_TYPE_UC_DATA:
        {
            /* Deliver Wake Data to Comm2Trm Process. */
            byHeadSize = GET_ST_FLD_OFFSET(RF_FRAME_UC_DATA, a_byDataBuf);
            byDataSize = s_stParseRFBuf.bySize - byHeadSize;
            p_byDataBuf = &s_stParseRFBuf.a_byBuf[byHeadSize];
            comm2trm_RxWakeData(p_byDataBuf, byDataSize);

            /* Make Wake Ack frame. */
            packetbuf_clear(); /* ATTENTION: MUST call this before copy data otherwise may make error. */
            p_stUCDataAck = (RF_FRAME_UC_DATA_ACK *)packetbuf_dataptr();
            p_stUCDataAck->byFlag = GET_RF_TYPE_RESP(RF_TYPE_UC_DATA);
            p_stUCDataAck->tAddr = s_stNetSettings.tNetAddr;
            byDataSize = comm2trm_GetWakeAck( p_stUCDataAck->a_byDataBuf,
                                                                      s_stNetSettings.stWNetConf.byWakeAckSize );
            byHeadSize = GET_ST_FLD_OFFSET(RF_FRAME_UC_DATA_ACK, a_byDataBuf);
            packetbuf_set_datalen(byHeadSize + byDataSize);
        }
            break;
        default:
            RIME_DBG(RIME_DBG_ON, "Error: Bad type of Wake Data.\r\n");
            break;
    }

    /* Whether need to TX the ACK frame?? */
    if (0 < packetbuf_datalen())
    {
        ASSERT(RF_IDLE == SX1278GetStatus());
        SX1278Send(packetbuf_dataptr(), packetbuf_datalen());
    }

    return;
}

/*---------------------------------------------------------------------------*/
static int8_t WakeThread(void)
{
    static int16_t    s_nWakeCnt = 0;
    static struct pt    s_tPT;

    PT_BEGIN(&s_tPT);

    ASSERT (RF_IDLE == SX1278GetStatus());

    if (s_stWNetRunSettings.nQuotientBeaconWake <= ++s_nWakeCnt)
    {
        s_nWakeCnt = 0;
        ++s_chCorrectRTCFailed;
        RIME_DBG(REPAIR_BROKEN_DBG, "WakeThread: CorrectRTCFailed=%d.\r\n", s_chCorrectRTCFailed);
    }

#if (!REL_VER)
    if (RF_Rx_Done == s_tRFResult)
    {
        ++s_ulRxWakeDataCnt;
        if (s_bEnPrintWakeCnt)
        {
            RIME_DBG(RIME_DBG_ON, "Wake OK Cnt = %lu\r\n", s_ulRxWakeDataCnt);
        }
    }
#endif

    if (RF_Rx_Done == s_tRFResult && RF_RX_DONE_WAKE_DATA == s_tRFRxDone)
    {
        /* Process the Wake Data frame. */
        ProcWakeData();

        /* Whether need to block until the TX ACK frame finished?? */
        if (0 < packetbuf_datalen())
        {
            PT_YIELD_UNTIL(&s_tPT, RF_Tx_Done == s_tRFResult || RF_Tx_Timeout == s_tRFResult);
            RIME_DBG(WAKE_DBG, "WakeThread(): Tx wake ack.\r\n");
        }
    }

    RIME_DBG(WAKE_DBG, "WakeThread(): end on %ldms.\r\n", rtc_GetTimeMs());

    PT_END(&s_tPT);
}
#endif/*#if (UPLINK_WAKE == CUR_SYST)*/


/*---------------------------------------------------------------------------*/
PROCESS(NetworkProcess, "network process");

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(NetworkProcess, ev, data)
{
    /* ATTENTION: stack variables can NOT cross any "XX_YIELD()". */
    int32_t    lMs;
    halIntState_t    intState;

    static int8_t    s_chStatus;
    static bool    s_bNeedUplink;

    PROCESS_BEGIN();

    rtc_EnAlarm();

    while (TRUE)
    {
        while (1)
        {
            /* Repeat Join or Correct RTC until done. */
            while (PT_YIELDED == JoinOrRTCThread(&s_chStatus))
            {
                PROCESS_YIELD(); /* Yield to ContikiOS */
            }

            if (0 <= s_chStatus) /* Join OK!*/
            {
                break; /* Break of while (1) */
            }
            else /* Join failed!*/
            {
                /* ATTENTION: do NOT break the "Get->Calc->Set RTC Alarm"(needs 1ms) by ISR! */
                HAL_ENTER_CRITICAL_SECTION(intState);
                lMs = rtc_GetTimeMs() + (util_GetRand16() % 100 + 100) * 1000; /* Delay [100, 200]s */
                if (MAX_MS_OF_DAY <= lMs)
                {
                    lMs -= MAX_MS_OF_DAY; /* Wrap on 00:00:00.000 */
                }
                if (rtc_SetAlarm(lMs) < 0)
                {
                    RIME_DBG(RIME_DBG_ON | RIME_DBG_LEVEL_SERIOUS, "NetworkProcess(): Set Alarm to join error!\r\n");
                }
                HAL_EXIT_CRITICAL_SECTION(intState);

                pm_AgrHalt(PWR_ID_NETWORK); /* HALT the MCU that have nothing to do. */
                PROCESS_YIELD(); /* Yield until the RTC alarm. */
                pm_OpposeHalt(PWR_ID_NETWORK); /* Do NOT HALT that have job to do. */
            }/*if*/
        }/*while (1)*/

        SetNextRTCAlarm();

        /* Uplink -> SetAlarm -> Uplink ->... */
        s_bNeedUplink = TRUE;
        while (s_bNeedUplink)
        {
            pm_AgrHalt(PWR_ID_NETWORK); /* HALT the MCU that have nothing to do. */
            PROCESS_YIELD();
            pm_OpposeHalt(PWR_ID_NETWORK); /* Do NOT HALT that have job to do. */

            /* Is RTC Alarm, Uplink the data packet. */
        #if (UPLINK == CUR_SYST)
            s_pfnThread = UplinkThread;
        #else
            if (NETWORK_THREAD_UPLINK == s_tNetworkThread)
            {
                s_pfnThread = UplinkThread;
            }
            else
            {
                s_pfnThread = WakeThread;
            }
        #endif
            while (PT_YIELDED == (* s_pfnThread)())
            {
                PROCESS_YIELD(); /* Yield to ContikiOS */
            }

            if (3 <= s_chCorrectRTCFailed)
            {
                /* EXPLAIN: Restart join and correct RTC to restore the uplink failed that caused by
                    RTC error or wireless breaked. */
                s_chCorrectRTCFailed = 0;
                s_bNeedUplink = FALSE;
                RIME_DBG( RIME_DBG_ON | RIME_DBG_LEVEL_WARNING, 
                                  "3 <= Correct RTC failed, Restart join to Sink!\r\n" );
            }
            else
            {
                SetNextRTCAlarm();
            }/*if*/
        }/*while (s_bNeedUplink)*/
    }/*while (TRUE)*/

    PROCESS_END();
}

#if (UPLINK == CUR_SYST)
/*-------------------------------------------------------------------------*/
static void ChgRFSpeedFreq(void)
{
    uint8_t    byIndex;

    SX1278SetFreq(s_stNetSettings.lFreq);

    if (BR_ON_AIR_START <= s_stNetSettings.tBROnAir && s_stNetSettings.tBROnAir <= BR_ON_AIR_END)
    {
        byIndex = s_stNetSettings.tBROnAir - BR_ON_AIR_START;
    }
    else
    {
        byIndex = BR_ON_AIR_20334 - BR_ON_AIR_START; /* BW=500kHz, SF=7, FEC=4/5*/
    }
    SX1278SetBandwidth(s_astBwSfFec[byIndex].eBW);
    SX1278SetSpreadingFactor(s_astBwSfFec[byIndex].eSF);
    SX1278SetCodingRate(s_astBwSfFec[byIndex].eFEC);
    SX1278SetLoRaSettings(); /* ATTENTION: MUST call this after changed settings */

    SX1278SetTxTimeout(SX1278GetTimeOnAir(RF_FIFO_SIZE) + GetAckDelay());

    /* ATTENTION: call "CalcTimeRFSpeed()" MUST before "CalcNetRunSettings()"! */
    CalcTimeRFSpeed();
    CalcNetRunSettings();

    return;
}
#endif

/**
  * @brief  Fetch settings and set RF as well as initialize process.
  * @param  None
  * @retval  None
  */
void network_Init(void)
{
    /* Fetch settings from EEPROM. */
    ss_FetchNetSettings(&s_stNetSettings);

    /* Set RF according to settings. */
    //复位1278，切换成LoRa模式，使用TCXO晶振作为sx1278的时钟源
    SX1278Init(&s_stRFEvents);
    //设置PA_BOOST引脚作为射频信号输出引脚
    SX1278SetPAOutput(PA_OUTPUT_PIN_BOOST); /* PA_BOOST */
    SX1278SetTxPower(s_stNetSettings.chTxPwr);

#if (UPLINK == CUR_SYST)
    SX1278SetPreambleLen(6); /* Preable Length=6+4.25 */
#endif
    SX1278SetLowDatarateOptimize(FALSE);
    SX1278SetFixLen(FALSE);
    SX1278SetCrcOn(TRUE);	
    SX1278SetRxContinuous(TRUE);

    /* EXPLAIN: the call of "SX1278SetLoRaSettings()" in this function. */
#if (UPLINK == CUR_SYST)
    ChgRFSpeedFreq();
#else
    ChgRFFreq();
    ChgRFSpeed();
#endif
	
    /* Initialize Network-Process. */
    process_start(&NetworkProcess, NULL);

    return;
}

/**
  * @brief  Update the Net-Settings of Network-Process.
  * @param  const NetSettings_t *p_stNew    point to the new Net-Settings.
  * @retval  None.
  */
void network_UpdateNetSettings(const NetSettings_t *p_stNew)
{
    ASSERT(p_stNew);

    memcpy(&s_stNetSettings, p_stNew, sizeof(s_stNetSettings));
#if (UPLINK == CUR_SYST)
    ChgRFSpeedFreq();
#else
    ChgRFFreq();
    ChgRFSpeed();
#endif

    return;
}

/**
  * @brief  Inform the Network-Process that alarmed by RTC.
  * @param  None.
  * @retval  None.
  */
void network_AlarmByRTC(void)
{
#if (UPLINK == CUR_SYST)
    process_poll(&NetworkProcess);
#else
    if (NETWORK_THREAD_WAKE == s_tNetworkThread)
    {
    #if EXTI_HALT_UART
        chip_ExitLowPwr(); /* Enable SPI and TCXO for RF. */
        pm_OpposeHalt(PWR_ID_NETWORK); /* Do NOT HALT that have to wake communication. */
    #endif
        ASSERT(RF_IDLE == SX1278GetStatus());
        SX1278SetCAD(s_stTimeRFSpeed.wCADPeriod);
        s_tCADEndTime = RTIMER_NOW() + s_stWNetRunSettings.wPreambleTime;
        RIME_DBG(WAKE_DBG, "\r\nRTC_ISR: start CAD on %ldms  (%u)\r\n", rtc_GetTimeMs(), RTIMER_NOW());
    #if CATCH_NET_BLOCK
        s_chCADCnt = 1;
    #endif
    }
    else
    {
        process_poll(&NetworkProcess);
    }
#endif

    return;
}

/**
  * @brief  Update the RF TX power.
  * @param  int8_t chTxPwr    RF TX power.
  * @retval  None.
  */
void network_UpdateTxPwr(int8_t chTxPwr)
{
    ASSERT( (RF_TX_PWR_MIN <= chTxPwr) &&
                  (chTxPwr <= RF_TX_PWR_MAX) &&
                  (s_stNetSettings.chTxPwr != chTxPwr) );

    s_stNetSettings.chTxPwr = chTxPwr;
    SX1278SetTxPower(chTxPwr);

    return;
}

/**
  * @brief  Get the RSSI of the latest packet received.
  * @param  Node.
  * @retval  int16_t  value of RSSI.
  */
int16_t network_GetPktRSSI(void)
{
    return s_nPacketRssi;
}

/**
  * @brief  Get the SNR of the latest packet received.
  * @param  Node.
  * @retval  int8_t  value of SNR.
  */
int8_t network_GetPktSNR(void)
{
    return s_chPacketSnr;
}

/**
  * @brief  Check whether this node have joined network.
  * @param  Node.
  * @retval  TRUE=have joined, FALSE=have NOT join.
  */
bool network_IsJoinNet(void)
{
    if (NETWORK_THREAD_CSMA == s_tNetworkThread)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}


/*-------------------------------------------------------------------------*/
#if CATCH_NET_BLOCK
void network_GetLC(void *p_vPrintBuf)
{
    #include <stdio.h>

    char    *p_chBuf;
    uint16_t    wLC;
    RadioState_t    tRFState;
    static const char    *a_pchRF[] =
    {
        "RF_IDLE",
        "RF_RX_RUNNING",
        "RF_TX_RUNNING",
        "RF_CAD_RUNNING",
    };

    wLC = NetworkProcess.pt.lc;
    tRFState = SX1278GetStatus();

    p_chBuf = (char *)p_vPrintBuf;
    sprintf( p_chBuf,
               "cur=%ld, alarm=%ld, for %s; block on L%d, RF=%s; CAD cnt=%d, CAD TO cnt=%d.\r\n",
               s_lCurMs, 
               s_lAlarmMs,
               (NETWORK_THREAD_UPLINK == s_tNetworkThread) ? "UPLINK" : "WAKE",
               wLC,
               a_pchRF[tRFState],
               s_chCADCnt,
               s_nCADTOCnt );

    return;
}
#endif


#if (!REL_VER)
/*-------------------------------------------------------------------------*/
void network_UpdateBROnAir(BR_ON_AIR_Type tBROnAir)
{
    if (tBROnAir != s_stNetSettings.tBROnAir)
    {
        s_stNetSettings.tBROnAir = tBROnAir;
        ss_SaveNetSettings(&s_stNetSettings);
    #if (UPLINK == CUR_SYST)
        ChgRFSpeedFreq();
    #else
        ChgRFSpeed();
    #endif
    }

    return;
}

/*-------------------------------------------------------------------------*/
void network_UpdateFreq(uint32_t ulFreq)
{
    if (ulFreq != s_stNetSettings.lFreq)
    {
        s_stNetSettings.lFreq = ulFreq;
        ss_SaveNetSettings(&s_stNetSettings);
    #if (UPLINK == CUR_SYST)
        ChgRFSpeedFreq();
    #else
        ChgRFFreq();
    #endif
    }

    return;
}

/*-------------------------------------------------------------------------*/
void network_TogglePrintWakeCnt(void)
{
    s_bEnPrintWakeCnt = !s_bEnPrintWakeCnt;

    return;
}

/*-------------------------------------------------------------------------*/
void network_Poll(void)
{
    process_poll(&NetworkProcess);

    return;
}
#endif /*#if (!REL_VER)*/


/*-------------------------------------------------------------------------*/
/**
  * @note  Call this when RF speed changed.
  */
static void CalcTimeRFSpeed(void)
{
    if (JOIN_NET_FLAG == s_stNetSettings.wJoinNetFlag)
    {
        s_stTimeRFSpeed.wJoinReq = SX1278GetTimeOnAir(sizeof(RF_FRAME_REQ_SLOT_RTC));
    }
    else
    {
        s_stTimeRFSpeed.wJoinReq = SX1278GetTimeOnAir(sizeof(RF_FRAME_REQ_JOIN));
    }
    s_stTimeRFSpeed.wJoinResp = WaitJoinOrRTC();
    s_stTimeRFSpeed.wJoinWin = s_stTimeRFSpeed.wJoinReq + s_stTimeRFSpeed.wJoinResp;
    if (500 < s_stTimeRFSpeed.wJoinWin)
    {
        s_stTimeRFSpeed.wJoinWin = 500; /* 500ms: 65535/127=512 */
    }
    s_stTimeRFSpeed.wUplinkRespOnAir = SX1278GetTimeOnAir(sizeof(RF_FRAME_UPLINK_ACK));
#if (UPLINK == CUR_SYST)
    s_stTimeRFSpeed.wUplinkResp = GetAckDelay() + s_stTimeRFSpeed.wUplinkRespOnAir;
#else
    s_stTimeRFSpeed.wUplinkRespDiffOnAir = SX1278GetTimeOnAir(sizeof(RF_FRAME_UPLINK_ACK_DIFF_VER));
    s_stTimeRFSpeed.wUplinkResp = GetAckDelay() + s_stTimeRFSpeed.wUplinkRespDiffOnAir;
    s_stTimeRFSpeed.wWakeAddr = GetAckDelay() + SX1278GetTimeOnAir(sizeof(RF_FRAME_WAKE_ADDR));
    s_stTimeRFSpeed.wBCTimeOnAir = SX1278GetTimeOnAir(sizeof(RF_FRAME_BC_TIME));
    s_stTimeRFSpeed.wMinWakeExchange = SEPARATE_TIME +    \
        SX1278GetTimeOnAir(sizeof(RF_FRAME_WAKE_ADDR)) + SX1278GetTimeOnAir(sizeof(RF_FRAME_BC_NET_SETTINGS));
#endif
    s_stTimeRFSpeed.wCADPeriod = SX1278CalcCADPeriod() + 10; /* Add 10ms for prepare time. */

    return;
}

/*-------------------------------------------------------------------------*/
/**
  * @note  Call on: 1="s_stNetSettings" changed, 2=RF speed changed.
  */
static void CalcNetRunSettings(void)
{
#if (UPLINK == CUR_SYST)
    s_stWNetRunSettings.wUplinkReqTime =    \
        SX1278GetTimeOnAir(s_stNetSettings.byMaxPayload + GET_ST_FLD_OFFSET(RF_FRAME_UPLINK_DATA, a_byDataBuf));
#else
    s_stWNetRunSettings.wUplinkReqTime =    \
        SX1278GetTimeOnAir(s_stNetSettings.stWNetConf.byUplinkPayload + GET_ST_FLD_OFFSET(RF_FRAME_UPLINK_DATA, a_byDataBuf));

    /* ATTENTION: MUST change RF speed before calculated run settings. */
    if (0 == s_stNetSettings.stWNetConf.byUplinkPayload)
    {
        s_stWNetRunSettings.wUplinkExchange = 0;
        s_stWNetRunSettings.wSlotUsed = 0;
        s_stWNetRunSettings.wSlotIdle = 0;
        s_stWNetRunSettings.lReservedSlot = 0;
    }
    else
    {
        s_stWNetRunSettings.wUplinkExchange = s_stWNetRunSettings.wUplinkReqTime + s_stTimeRFSpeed.wUplinkResp;
        s_stWNetRunSettings.wSlotUsed = s_stWNetRunSettings.wUplinkExchange * (1 + s_stNetSettings.stWNetConf.chRepeatNum);
        s_stWNetRunSettings.wSlotIdle = s_stNetSettings.stWNetConf.wSlotLen - s_stWNetRunSettings.wSlotUsed;
        s_stWNetRunSettings.lReservedSlot = s_stNetSettings.stWNetConf.lUplinkPeriod -    \
            s_stNetSettings.stWNetConf.wSlotLen * s_stNetSettings.stWNetConf.nNodeNum;
   }

    if (0 == s_stNetSettings.stWNetConf.byWakeDataSize)
    {
        s_stWNetRunSettings.tWakeSort = WAKE_SORT_NO_NEED;
        s_stWNetRunSettings.wWakeExchange = 0;
        s_stWNetRunSettings.wWakeGuard = 0;
        s_stWNetRunSettings.lWakeNoFly = 0;
    }
    else
    {
        s_stWNetRunSettings.wWakeExchange =    \
            GetTime4WakeExchange(s_stNetSettings.stWNetConf.byWakeDataSize, s_stNetSettings.stWNetConf.byWakeAckSize);
        if (s_stWNetRunSettings.wWakeExchange < s_stTimeRFSpeed.wMinWakeExchange)
        {
            s_stWNetRunSettings.wWakeExchange = s_stTimeRFSpeed.wMinWakeExchange;
        }

        if (0 == s_stNetSettings.stWNetConf.byUplinkPayload)
        {
            s_stWNetRunSettings.tWakeSort = WAKE_SORT_FREE;
            s_stWNetRunSettings.wWakeGuard = 0;
            s_stWNetRunSettings.lWakeNoFly = 0;            
        }
        else
        {
            if (s_stWNetRunSettings.wWakeExchange <= s_stWNetRunSettings.wSlotIdle)
            {
                s_stWNetRunSettings.tWakeSort = WAKE_SORT_FREE;
                s_stWNetRunSettings.lWakeNoFly = SEPARATE_TIME + s_stWNetRunSettings.wSlotUsed;
            }
            else
            {
                if (s_stWNetRunSettings.wWakeExchange <= s_stWNetRunSettings.lReservedSlot)
                {
                    s_stWNetRunSettings.tWakeSort = WAKE_SORT_NO_FLY_ZONE;
                    s_stWNetRunSettings.lWakeNoFly = SEPARATE_TIME +    \
                        s_stNetSettings.stWNetConf.wSlotLen * s_stNetSettings.stWNetConf.nNodeNum;
                }
                else
                {
                    s_stWNetRunSettings.tWakeSort = WAKE_SORT_NO_NEED;
                    s_stWNetRunSettings.wWakeGuard = 0;
                    s_stWNetRunSettings.lWakeNoFly = 0;
                }
            }
        }
    }

    /* Calculate "wake guard" . */
    if (0 != s_stWNetRunSettings.lWakeNoFly)
    {
        ASSERT(0 < s_stNetSettings.stWNetConf.lWakeInterval);
        s_stWNetRunSettings.wWakeGuard =    \
            (s_stNetSettings.stWNetConf.lUplinkPeriod - s_stWNetRunSettings.lWakeNoFly) % s_stNetSettings.stWNetConf.lWakeInterval;        		
    }

    if (WAKE_SORT_NO_NEED != s_stWNetRunSettings.tWakeSort)
    {
        #define BEACON_INTERVAL    (int32_t)128000 /* 128s */
        ASSERT(0 < s_stNetSettings.stWNetConf.lWakeInterval);
        s_stWNetRunSettings.nQuotientBeaconWake =    \
            (BEACON_INTERVAL + s_stNetSettings.stWNetConf.lWakeInterval / 2) / s_stNetSettings.stWNetConf.lWakeInterval;
    }
#endif

    return;
}

#if (UPLINK_WAKE == CUR_SYST)
/*-------------------------------------------------------------------------*/
static uint16_t GetTime4WakeExchange(uint8_t byWakeDataSize, uint8_t byWakeAckSize)
{
    uint8_t    byWakeDataFrame;
    uint8_t    byWakeAckFrame;
    uint16_t    wTime;

    byWakeDataFrame = byWakeDataSize + GET_ST_FLD_OFFSET(RF_FRAME_UC_DATA, a_byDataBuf);
    byWakeAckFrame = byWakeAckSize + GET_ST_FLD_OFFSET(RF_FRAME_UC_DATA_ACK, a_byDataBuf);

    wTime = SEPARATE_TIME + GetAckDelay();
    wTime += SX1278GetTimeOnAir(sizeof(RF_FRAME_WAKE_ADDR));
    wTime += SX1278GetTimeOnAir(byWakeDataFrame);
    wTime += SX1278GetTimeOnAir(byWakeAckFrame);

    return wTime;
}

/*-------------------------------------------------------------------------*/
static void ChgRFFreq(void)
{
    SX1278SetFreq(s_stNetSettings.lFreq);

    return;
}

/*-------------------------------------------------------------------------*/
static void ChgRFSpeed(void)
{
    #define MIN_PREAMBLE_TIME    20 /* 20ms */
    #define MIN_PREAMBLE_SIZE    8

    uint8_t    byIndex;

    /* Set "BW+SF+FEC" according to BR on air. */
    if (BR_ON_AIR_START <= s_stNetSettings.tBROnAir && s_stNetSettings.tBROnAir <= BR_ON_AIR_END)
    {
        byIndex = s_stNetSettings.tBROnAir - BR_ON_AIR_START;
    }
    else
    {
        byIndex = BR_ON_AIR_20334 - BR_ON_AIR_START; /* BW=500kHz, SF=7, FEC=4/5*/
    }
    SX1278SetBandwidth(s_astBwSfFec[byIndex].eBW);
    SX1278SetSpreadingFactor(s_astBwSfFec[byIndex].eSF);
    SX1278SetCodingRate(s_astBwSfFec[byIndex].eFEC);

    /* Set the appropriate size of preamble */
    s_stWNetRunSettings.chPreambleSize = SX1278CalcPreambleNum(MIN_PREAMBLE_TIME);
    if (s_stWNetRunSettings.chPreambleSize < MIN_PREAMBLE_SIZE)
    {
        s_stWNetRunSettings.chPreambleSize = MIN_PREAMBLE_SIZE;
    }
    SX1278SetPreambleLen(s_stWNetRunSettings.chPreambleSize);
    s_stWNetRunSettings.wPreambleTime = SX1278GetPreambleTime();

    /* ATTENTION: MUST call this after changed settings */
    SX1278SetLoRaSettings();

    SX1278SetTxTimeout(SX1278GetTimeOnAir(255) + GetAckDelay());

    /* ATTENTION: call "CalcTimeRFSpeed()" MUST before "CalcNetRunSettings()"! */
    CalcTimeRFSpeed();
    CalcNetRunSettings();

    return;
}

/*-------------------------------------------------------------------------*/	
static int32_t CalcWakeTime(int32_t lRTC)
{
    int32_t    lRTCVal, lXInterval, lMulDiv, lWakeTime, lWakeEnd;

    ASSERT(WAKE_SORT_NO_NEED != s_stWNetRunSettings.tWakeSort);

    lRTCVal = lRTC;

    if (s_stNetSettings.stWNetConf.lWakeInterval < s_stNetSettings.stWNetConf.lUplinkPeriod)
    {
        /* Check whether need to add safeguard. */
        if ( lRTCVal / s_stNetSettings.stWNetConf.lUplinkPeriod !=    \
             (lRTCVal + s_stWNetRunSettings.wWakeGuard) / s_stNetSettings.stWNetConf.lUplinkPeriod )
        {
            lRTCVal += s_stWNetRunSettings.wWakeGuard;
        }

        /* Calculate value of "N" */
        ASSERT(0 < s_stNetSettings.stWNetConf.lUplinkPeriod);
        lMulDiv = lRTCVal / s_stNetSettings.stWNetConf.lUplinkPeriod;

        /* Calculate value of "x" */
        lXInterval = (lRTCVal + s_stNetSettings.stWNetConf.lWakeInterval) -    \
                          lMulDiv * s_stNetSettings.stWNetConf.lUplinkPeriod -    \
                          s_stWNetRunSettings.lWakeNoFly;
        lXInterval /= s_stNetSettings.stWNetConf.lWakeInterval;
        if (lXInterval < 0)
        {
            lXInterval = 0;
        }
        ++lXInterval;

        /* wake time = N * wake period + nofly + (x - 1) * wake interval. */
        lWakeTime = lMulDiv * s_stNetSettings.stWNetConf.lUplinkPeriod +    \
                           s_stWNetRunSettings.lWakeNoFly +    \
                           (lXInterval - 1) * s_stNetSettings.stWNetConf.lWakeInterval;
        ASSERT(lRTC < lWakeTime);

        /* Check collision between WAKE<->UPLINK or WAKE<->WAKE.
            EXPLAIN: (WakeEnd - SEPARATE_TIME) for remove redundancy 
                          that both NoFly and WakeExchange add SEPARATE_TIME. */
        lWakeEnd = lWakeTime + s_stWNetRunSettings.wWakeExchange - SEPARATE_TIME - 1;
        if ( (lWakeTime / s_stNetSettings.stWNetConf.lUplinkPeriod) !=    \
             (lWakeEnd / s_stNetSettings.stWNetConf.lUplinkPeriod) )
        {
            lWakeTime = lWakeEnd / s_stNetSettings.stWNetConf.lUplinkPeriod *    \
                               s_stNetSettings.stWNetConf.lUplinkPeriod +    \
                               s_stWNetRunSettings.lWakeNoFly;
        }
    }
    else
    {
        /* Calculate value of "x" */
        lXInterval = lRTCVal + s_stNetSettings.stWNetConf.lWakeInterval - s_stWNetRunSettings.lWakeNoFly;
        lXInterval = lXInterval / s_stNetSettings.stWNetConf.lWakeInterval;
        ++lXInterval;

        /* wake time = nofly + (x - 1) * wake interval. */
        lWakeTime = s_stWNetRunSettings.lWakeNoFly +    \
                           (lXInterval - 1) * s_stNetSettings.stWNetConf.lWakeInterval;
        ASSERT(lRTC < lWakeTime);
    }
	
    /* Check whether wrap on 00:00:00.000. */
    if (MAX_MS_OF_DAY <= lWakeTime)
    {
        lWakeTime -= MAX_MS_OF_DAY;
    }

    return lWakeTime;
}
#endif/*#if (UPLINK_WAKE == CUR_SYST)*/


#if 0
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
void network_Test(void)
{
    uint8_t    bySize;
    uint8_t    byCnt;
    RF_FRAME_BC_DATA    *p_stFrame;

    bySize = 248;

    p_stFrame = (RF_FRAME_BC_DATA *)s_astRadioBuf[0].a_byBuf;
    p_stFrame->byFlag = RF_TYPE_UC_DATA;
    for (byCnt = 0; byCnt < bySize; ++byCnt)
    {
        p_stFrame->a_byDataBuf[byCnt] = byCnt + 1;
    }
    s_astRadioBuf[0].bySize = 1 + bySize;

    ProcWakeData();


#if 0
    /* Uplink Only */
    s_stNetSettings.stWNetConf.byUplinkPayload = 1;
    s_stWNetRunSettings.tWakeSort = WAKE_SORT_NO_NEED;

    s_lCurMs = 50;
    s_lUplinkTime = 100;
    s_lWakeTime = 200;
    SetNextRTCAlarm();


    /* Wake Only */
    s_stNetSettings.stWNetConf.byUplinkPayload = 0;
    s_stWNetRunSettings.tWakeSort = WAKE_SORT_FREE;

    s_lCurMs = 50;
    s_lUplinkTime = 100;
    s_lWakeTime = 200;
    SetNextRTCAlarm();


    /* Uplink && Wake */
    s_stNetSettings.stWNetConf.byUplinkPayload = 1;
    s_stWNetRunSettings.tWakeSort = WAKE_SORT_FREE;

    s_lCurMs = MAX_MS_OF_DAY - 50;
    s_lUplinkTime = 100;
    s_lWakeTime = 200;
    SetNextRTCAlarm();

    s_lCurMs = MAX_MS_OF_DAY - 50;
    s_lUplinkTime = 300;
    s_lWakeTime = 200;
    SetNextRTCAlarm();

    s_lCurMs = 50;
    s_lUplinkTime = 100;
    s_lWakeTime = 200;
    SetNextRTCAlarm();

    s_lCurMs = 50;
    s_lUplinkTime = 300;
    s_lWakeTime = 200;
    SetNextRTCAlarm();
#endif

    return;
}
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
#endif


/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

