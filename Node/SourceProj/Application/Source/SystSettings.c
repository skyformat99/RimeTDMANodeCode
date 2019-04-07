/**
 * \file
 *         SystSettings.c
 * \description
 *         System Settings
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-11-13 10:09
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */


/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include "Dbg.h"
#include "Util.h"
#include "EEPROM.h"
#include "Comm2Trm.h"
#include "SystSettings.h"


/* Private macro -------------------------------------------------------------*/
/* EXPLAIN: Multiple 2 for MAIN and SHADOW block */
#define CALC_OBJ_ADDR(PrevAddr, PrevSize)    ((PrevAddr) + 2 * (PrevSize))

/* Format of object:[LEN][Object][CRC16] */
#define BYTE_CFG_VALID_LEN    2
#define BYTE_CFG_CRC16    2
#define BYTE_CFG_CTRL    (BYTE_CFG_VALID_LEN + BYTE_CFG_CRC16)

/* Initialized flag of EEPROM */
#define E2_ADDR_INIT_FLAG    (0 + E2_ADDR_START)
#define E2_SIZE_INIT_FLAG    2

/*
*Start address of settings at EEPROM 
*地址是0x1002，参考STM8L151C8数据手册P41
*/
#define E2_ADDR_CFG_START    (E2_ADDR_INIT_FLAG + E2_SIZE_INIT_FLAG)  

/* Settings_1: Network Settings. */
#define E2_ADDR_NET_SETTINGS    E2_ADDR_CFG_START  //地址是0x1002
#define E2_SIZE_NET_SETTINGS    (BYTE_CFG_CTRL + sizeof(NetSettings_t))  //4+sizeof（NetSettings_t）

#if 0
/* Settings_2: */
#define E2_ADDR_XXXX    \
    CALC_OBJ_ADDR(E2_ADDR_NET_SETTINGS, E2_SIZE_NET_SETTINGS)
#define E2_SIZE_NETWORK    (BYTE_CFG_CTRL + sizeof(SYST_XXXX))
#endif


/* Private typedef -----------------------------------------------------------*/
/**
* @brief  Add ID of settings object on here!
*/
typedef enum _cfg_id
{
    ID_INVALID_MIN = (uint8_t)0,
    ID_NET_SETTINGS,
    ID_INVALID_MAX,    
} CFG_ID_Type;

/* Configure object in EEPROM */
typedef void (* PFN_CFG)(void);
typedef struct
{
    CFG_ID_Type    eID;
    int8_t    chTaskNum; /* Number of task that depends on this settings */
    uint16_t    wAddrMain; /* Address of MAIN block */
    uint16_t    wMaxSize; /* Maximum size of settings */
    const PFN_CFG    *p_pfnUpdate; /* Point to updated function by this pointer */
} CFG_SAVE;

/* Size and Ptr of default settings */
typedef struct
{
    uint16_t    wSize;
    const void    *p_vData;
} SIZE_PTR;


/* Private variables ---------------------------------------------------------*/
const static CFG_SAVE    s_stCfgSaveNetSettings =
{
    .eID = ID_NET_SETTINGS,             //值为1的枚举常量
    .chTaskNum = 0,
    .wAddrMain = E2_ADDR_NET_SETTINGS,   //地址是0x1002 
    .wMaxSize = E2_SIZE_NET_SETTINGS,   //4+sizeof（NetSettings_t）
    .p_pfnUpdate = NULL,
};

//定义一个指针常量，确保其永远指向s_stCfgSaveNetSettings,而不能指向其它的
const static CFG_SAVE * const    s_apstCfgSave[] =
{
    &s_stCfgSaveNetSettings,
};

/**
* @brief  Add default settings on here!
*/
const static NetSettings_t    s_stNetSettingsDefault =
{
#if (UPLINK == CUR_SYST)
    .byMaxPayload = MAX_LEN_UART_FRAME_DATA,
    .tNetAddr = 0x0000,
    .nNodeNum = 100,
    .wJoinNetFlag = 0xFFFF, /* Have not join. */
    .wSlotLen = 500, /* 500ms */
    .lSlotOffset = (int32_t)1000 * HHMMSS_2SEC(0, 0, 0), /* 00:00:00.000 */
    .lSlotInterval = (int32_t)1000 * HHMMSS_2SEC(12, 0, 0), /* 12:00:00 */
#else
    .wJoinNetFlag = 0xFFFF, /* Have not join. */
    .tNetAddr = 0x0000,
    .stWNetConf =
    {
        .byUplinkPayload = 4,
        .chRepeatNum = 0,
        .byWakeDataSize = 4,
        .byWakeAckSize = 0,
        .nNodeNum = 3,
        .wSlotLen = 1500, /* 1500ms */
        .lUplinkPeriod = 8000, /* 8s */
        .lWakeInterval = 8000, /* 8s */
    },
    .byNetVer = 1,        
#endif
#if (COMPANY_ShuWei == CUR_COMPANY)
    .chTxPwr = RF_TX_PWR_MIN,
#else
    .chTxPwr = RF_TX_PWR_MAX,
#endif
//    .tBROnAir = BR_ON_AIR_443,
    .tBROnAir = BR_ON_AIR_2876,
//    .lFreq = RF_FREQ_DEFAULT, /* 470MHz */
    .lFreq = 439000000, /* 439MHz */
};

const static SIZE_PTR    s_astDefaultSizePtr[] =
{
    {sizeof(s_stNetSettingsDefault), &s_stNetSettingsDefault},
};


/* Private function prototypes -----------------------------------------------*/


/* Private Constants ---------------------------------------------------------*/


/**
  * @brief  Save a settings into EEPROM that format to [Length][Data][CRC16].
  * @param  void *p_vBuf    point to buffer saved settings by this pointer
  * @param  uint16_t wSize    count of the settings
  * @param  CFG_ID eID    configured ID
  * @retval  -1=Error of ID; -2=Invalid size; -3=Allocate memory failed; 0=OK
  */
static int8_t SaveCfg(const void *p_vBuf, uint16_t wSize, CFG_ID_Type eID)
{
    ASSERT(p_vBuf && wSize > 0 && eID > ID_INVALID_MIN && eID < ID_INVALID_MAX);

    int8_t    chCnt;
    uint16_t    wCRC16, wShadowAddr;
    uint8_t    *p_byAlloc;
    const CFG_SAVE    *p_stSave;	

    if (eID > ID_INVALID_MIN && eID < ID_INVALID_MAX)
    {
        p_stSave = s_apstCfgSave[eID - ID_INVALID_MIN - 1]; /* Start from 0 */
    }
    else
    {
        return -1; /* Error of ID */
    }

    if (wSize > (p_stSave->wMaxSize - BYTE_CFG_CTRL))
    {
        return -2; /* Invalid size */
    }

    p_byAlloc = (uint8_t *)malloc(p_stSave->wMaxSize);
    if (!p_byAlloc)
    {
        PRINTF("SaveCfg() allocate memory failed.\r\n", 0, PRINTF_FORMAT_NONE);
        return -3; /* Allocate memory failed */
    }

    /* Make settings to [Length][Data][CRC16] */
    *(uint16_t *)&p_byAlloc[0] = wSize; /* Length of object */
    memcpy(&p_byAlloc[BYTE_CFG_VALID_LEN], p_vBuf, wSize); /* Copy object body */

    wCRC16 = util_CRC16(&p_byAlloc[0], BYTE_CFG_VALID_LEN + wSize); /* Calculate CRC16 */
    *(uint16_t *)&p_byAlloc[BYTE_CFG_VALID_LEN + wSize] = wCRC16; /* Save CRC16 */

    /* Save settings into EEPROM */
    e2_Wr(&p_byAlloc[0], BYTE_CFG_CTRL + wSize, p_stSave->wAddrMain); /* MAIN */
    wShadowAddr = p_stSave->wAddrMain + p_stSave->wMaxSize;
    e2_Wr(&p_byAlloc[0], BYTE_CFG_CTRL + wSize, wShadowAddr);/* SHADOW */
	
    free(p_byAlloc);

    /* Inform the dependent task to update settings */
    for (chCnt = 0; chCnt < p_stSave->chTaskNum; ++chCnt)
    {
        p_stSave->p_pfnUpdate[chCnt]();
    }

    return 0;
}

/**
  * @brief  Fetch a settings from EEPROM.
  * @param  void *p_vBuf    point to buffer saved settings by this pointer.
  * @param  uint16_t wSize    count of the desired fetched data.
  * @param  CFG_ID eID    configured ID
  * @retval  -1=Error of ID; >0=>length of fetched data.
  */
static int16_t FetchCfg(void *p_vBuf, uint16_t wSize, CFG_ID_Type eID)
{
    ASSERT(p_vBuf && wSize > 0 && eID > ID_INVALID_MIN && eID < ID_INVALID_MAX);

    uint16_t    wValidLen;
    const CFG_SAVE    *p_stSave;	

    if (eID > ID_INVALID_MIN && eID < ID_INVALID_MAX)
    {
        p_stSave = s_apstCfgSave[eID - ID_INVALID_MIN - 1]; /* Start from 0 */
    }
    else
    {
        return -1;    /* Error of ID */
    }

    /* Get valid length of configuration data */
    e2_Rd(&wValidLen, BYTE_CFG_VALID_LEN, p_stSave->wAddrMain);

    /* Fetch configuration data from EEPROM */
    wValidLen = MIN(wValidLen, wSize);    /* Prevent overwrite invoked buffer */
    e2_Rd(p_vBuf, wValidLen, BYTE_CFG_VALID_LEN + p_stSave->wAddrMain);

    return (int16_t)wValidLen;
}

/*---------------------------------------------------------------------------------------------*/
/* Format of object:[LEN=2 Byte][Object=LEN Byte][CRC16=2 Byte] */
static bool CheckCRC16(const uint8_t *p_byBuf)
{
    ASSERT(p_byBuf);

    uint16_t    wTemp, wCRC16;

    wTemp = *(uint16_t *)p_byBuf; /* Length of object */
    wTemp += BYTE_CFG_VALID_LEN;
    wCRC16 = util_CRC16(&p_byBuf[0], wTemp);
    wTemp = *(uint16_t *)&p_byBuf[wTemp];

    return (bool)(wCRC16 == wTemp);
}

/**
  * @brief  Initialize system settings.
  * @param  None
  * @retval  -1=error,allocate memory failed; 0=OK.
  */
int8_t ss_Init(void)
{
    int8_t     chCnt;
    uint8_t    *p_byAlloc;
    uint16_t    wRefFlag, wTemp, wShadowAddr;

    //设置flash编程时间
    e2_Init(); /* Initialize EEPROM */

    /* Get the Max size of objects */
    for (chCnt = 0, wTemp = 0; chCnt < SIZE_OF_ARRAY(s_apstCfgSave); ++chCnt)
    {
        if (wTemp < s_apstCfgSave[chCnt]->wMaxSize)
        {
            wTemp = s_apstCfgSave[chCnt]->wMaxSize;
        }
    }

    /* Allocate memory for operate settings. */
    p_byAlloc = (uint8_t *)malloc(wTemp);
    if (!p_byAlloc)
    {
        PRINTF("sc_Init() allocate memory failed.\r\n", 0, PRINTF_FORMAT_NONE);
        return -1; /* Allocate memory failed */
    }

    /* EXPLAIN: Reset FRAM if updated version */ 
    wRefFlag = util_CRC16(CUR_VER, strlen(CUR_VER));

    /* Check whether need to initialize EEPROM */
    /*从地址0x1000开始读取两个字节存放在wTemp中*/
    e2_Rd(&wTemp, E2_SIZE_INIT_FLAG, E2_ADDR_INIT_FLAG);
    if (wRefFlag == wTemp) /* Already initialized */
    {
        /* Check validity of all data block */
        for (chCnt = 0; chCnt < SIZE_OF_ARRAY(s_apstCfgSave); ++chCnt)
        {
            wShadowAddr = s_apstCfgSave[chCnt]->wAddrMain + s_apstCfgSave[chCnt]->wMaxSize;
            e2_Rd(p_byAlloc, s_apstCfgSave[chCnt]->wMaxSize, s_apstCfgSave[chCnt]->wAddrMain);
            if (CheckCRC16(p_byAlloc)) /* MAIN block is OK, check SHADOW block */
            {
                e2_Rd( p_byAlloc, s_apstCfgSave[chCnt]->wMaxSize, wShadowAddr); /* Read from SHADOW */
                if (!CheckCRC16(p_byAlloc)) /* SHADOW block is insanity, copy from MAIN */
                {
                    e2_Rd(p_byAlloc, s_apstCfgSave[chCnt]->wMaxSize, s_apstCfgSave[chCnt]->wAddrMain);
                    e2_Wr(p_byAlloc, s_apstCfgSave[chCnt]->wMaxSize, wShadowAddr); /* Save into SHADOW */
                }
            }
            else /* MAIN block is insanity, restore from SHADOW */
            {
                e2_Rd( p_byAlloc, s_apstCfgSave[chCnt]->wMaxSize, wShadowAddr); /* Read from SHADOW */
                e2_Wr(p_byAlloc, s_apstCfgSave[chCnt]->wMaxSize, s_apstCfgSave[chCnt]->wAddrMain); /* Save into MAIN */
            }	        
        }/*for*/
    }
    else   /*EEPROM没有初始化，则初始化EEPROM*/
    {
        //这里的s_astDefaultSizePtr指针指向默认的网络参数设置,所以这里是把默认的网络参数保存到EEPROM中
        for (chCnt = 0; chCnt < SIZE_OF_ARRAY(s_astDefaultSizePtr); ++chCnt)
        {
            /* Make a body as: Length+Data+CRC16 */
            *(uint16_t *)&p_byAlloc[0] = s_astDefaultSizePtr[chCnt].wSize; /* Length */
            memcpy( &p_byAlloc[BYTE_CFG_VALID_LEN], /* Copy data */
                          s_astDefaultSizePtr[chCnt].p_vData,
                          s_astDefaultSizePtr[chCnt].wSize );
            wTemp = BYTE_CFG_VALID_LEN + s_astDefaultSizePtr[chCnt].wSize; /* Offset=Length+Object */
            *(uint16_t *)&p_byAlloc[wTemp] = util_CRC16(&p_byAlloc[0], wTemp); /* 计算前面的Length+Data的CRC16的值 */
            /*  
             *P_byAlloc的长度是2+2+默认网络参数长度,即s_apstCfgSave[chCnt]->wMaxSize,其中Length和CRC16各2个字节
             *到这里为止，已经成功构建了Length+Data+CRC16的数据帧，
             *其中的Data是默认的网络参数,Length是默认网络参数的长度,单位是字节
             */
            /* Save into MAIN block,把构建好的数据帧写到EEPROM中，从地址0x1002开始写*/
            e2_Wr(p_byAlloc, s_apstCfgSave[chCnt]->wMaxSize, s_apstCfgSave[chCnt]->wAddrMain);

            /* Save into SHADOW block */
            /*参数*/
            wShadowAddr = s_apstCfgSave[chCnt]->wAddrMain + s_apstCfgSave[chCnt]->wMaxSize;
            e2_Wr(p_byAlloc, s_apstCfgSave[chCnt]->wMaxSize, wShadowAddr);
        }/*for*/

        /* Save initialized flag into EEPROM */
        /*把初始化标记（即软件版本号的CRC16值）存放在从地址0x1000开始的两个字节内*/
        e2_Wr(&wRefFlag, E2_SIZE_INIT_FLAG, E2_ADDR_INIT_FLAG);
    }

    free(p_byAlloc); /* Attention: release memory before exit funciton! */

    return 0;
}

/*---------------------------------------------------------------------------------------------*/
int16_t ss_FetchNetSettings(NetSettings_t *p_stSettings)
{
    return FetchCfg(p_stSettings, sizeof(NetSettings_t), ID_NET_SETTINGS);
}

/*---------------------------------------------------------------------------------------------*/
int8_t ss_SaveNetSettings(const NetSettings_t *p_stSettings)
{
    return SaveCfg(p_stSettings, sizeof(NetSettings_t), ID_NET_SETTINGS);
}


/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

