/*
************************************************************************************************
* Filename   	: Monitor.c
* Programmer : JiangJun
* Description	: Task for monitoring system
* Date           : 2014-08-29
************************************************************************************************
*/

#include "main.h"

/*
*********************************************************************************************************
*                                                                        COMPILE SWITCH
*********************************************************************************************************
*/
#if (!REL_VER)

/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include <string.h>
#include <stdlib.h>
#include "process.h"
#include "Chip.h"
#include "Dbg.h"
#include "DebugPort.h"
#include "Monitor.h"
#include "Network.h"
#include "Comm2Trm.h"


/*
*********************************************************************************************************
*                                                                      	MACRO DEFINITION
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*										   DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/
/* Process command received from PC */
typedef void (* PFN_CMD)(void);
typedef struct _pc_cmd
{
    char    *p_chCmdStr;
    PFN_CMD    pfnProcCmd;
} PC_CMD;


/*
************************************************************************************************
*                                                                       FUNCTION PROTOTYPE
************************************************************************************************
*/
static void CmdHelp(void);
static void CmdVer(void);
static void CmdEnablePrint(void);
static void CmdDisablePrint(void);
static void CmdSetBROnAir(void);
static void CmdSetFreq(void);
static void CmdSetRTC(void);
#if (UPLINK == CUR_SYST)
static void CmdQuitNet(void);
#endif
static void CmdTogglePrintWakeCnt(void);
static void CmdPollNetwork(void);


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
/* Command RX from PC */
#define SIZE_BUF_PC_CMD    48
static INT8U    s_abyPCCmdBuf[SIZE_BUF_PC_CMD];

/* Supported command */
const static PC_CMD    s_astPCCmd[] =
{
    {"?", CmdHelp},
    {"version", CmdVer},
    {"open print", CmdEnablePrint},
    {"close print", CmdDisablePrint},
    {"bps=", CmdSetBROnAir},
    {"freq=", CmdSetFreq},
    {"rtc=", CmdSetRTC},
#if (UPLINK == CUR_SYST)
    {"quit net", CmdQuitNet},
#endif
    {"toggle print", CmdTogglePrintWakeCnt},
    {"poll", CmdPollNetwork},
};

#define SEND_OK_2_PC()    dp_Tx("\r\nOK\r\n", 6);

/*---------------------------------------------------------------------------------------------*/
static void CmdHelp(void)
{
    INT8S    chCnt;

    /* Print all string of command */
    for (chCnt = 0; chCnt < SIZE_OF_ARRAY(s_astPCCmd); ++chCnt)
    {
        dp_Tx(s_astPCCmd[chCnt].p_chCmdStr, strlen(s_astPCCmd[chCnt].p_chCmdStr));
        dp_Tx("\r\n", 2);
    }

    return;
}

/*---------------------------------------------------------------------------------------------*/
static void CmdVer(void)
{
    dp_Tx(CUR_VER, strlen(CUR_VER));
    dp_Tx(", ID=", 5);
    dp_Tx(chip_GetID(), strlen(chip_GetID()));
    dp_Tx("\r\n", 2);

    return;
}

/*---------------------------------------------------------------------------------------------*/
static void CmdEnablePrint(void)
{
    SEND_OK_2_PC();
    dbg_EnablePrint();

    return;
}

/*---------------------------------------------------------------------------------------------*/
static void CmdDisablePrint(void)
{
    SEND_OK_2_PC();
    dbg_DisablePrint();

    return;
}

/*---------------------------------------------------------------------------------------------*/
static void CmdSetBROnAir(void)
{
    BR_ON_AIR_Type    tBROnAir;

    /* Get value of OFF_TIME */
    tBROnAir = (BR_ON_AIR_Type)atoi((const char *)&s_abyPCCmdBuf[4]);
    if ((tBROnAir >= BR_ON_AIR_START) && (tBROnAir <= BR_ON_AIR_END))
    {
        SEND_OK_2_PC();
        network_UpdateBROnAir(tBROnAir);
    }
    else
    {
        PRINTF("Bad value, valid=1~10.\r\n", 0, PRINTF_FORMAT_NONE);
    }
	
    return;
}

/*---------------------------------------------------------------------------------------------*/
static void CmdSetFreq(void)
{
    uint32_t    uFreq;

    /* Get value of BW */
    uFreq = atol((const char *)&s_abyPCCmdBuf[5]);
    if (uFreq >= 410000000 && uFreq <= 525000000)
    {
        SEND_OK_2_PC();
        network_UpdateFreq(uFreq);
    }
    else
    {
        PRINTF("Bad Freq, valid=410000000~525000000.\r\n", 0, PRINTF_FORMAT_NONE);
    }	

    return;	
}

/*---------------------------------------------------------------------------------------------*/
#if (UPLINK == CUR_SYST)
static void CmdQuitNet(void)
{
    SEND_OK_2_PC();
    comm2trm_QuitNet();

    return;	
}
#endif

/*---------------------------------------------------------------------------------------------*/
static void CmdSetRTC(void)
{
    int32_t    lRTCMs;

    /* Get value of BW */
    lRTCMs = atol((const char *)&s_abyPCCmdBuf[4]);
    if (lRTCMs < MAX_MS_OF_DAY)
    {
        #include "RTC.h"
        SEND_OK_2_PC();
        rtc_SetTimeSec(lRTCMs / 1000);
        rtc_SetMs(lRTCMs % 1000);
    }
    else
    {
        PRINTF("Bad RTC, valid=0~86399999.\r\n", 0, PRINTF_FORMAT_NONE);
    }	

    return;	
}

/*---------------------------------------------------------------------------------------------*/
static void CmdTogglePrintWakeCnt(void)
{
    SEND_OK_2_PC();
    network_TogglePrintWakeCnt();

    return;
}

/*---------------------------------------------------------------------------------------------*/
static void CmdPollNetwork(void)
{
    SEND_OK_2_PC();
    network_Poll();

    return;
}

/*---------------------------------------------------------------------------------------------*/
static void ProcPCCmd(void)
{
    #define UNKNOWN_PROMPT    ("Unknown command, ? for help.\r\n")

    INT8S    chCnt;

    /* Match the command string one by one */
    for (chCnt = 0; chCnt < SIZE_OF_ARRAY(s_astPCCmd); ++chCnt)
    {
        if (0 == strncmp(s_astPCCmd[chCnt].p_chCmdStr, (char *)s_abyPCCmdBuf, 3))    /* Match a command */
        {
            (* s_astPCCmd[chCnt].pfnProcCmd)();    /* Invork corresponding procedure */
            return;    /* Finished job */			
        }
    }
    dp_Tx(UNKNOWN_PROMPT, strlen(UNKNOWN_PROMPT));	

    return;
}


#include "stm8l15x_rst.h"
static void PrintClockResetReason(void)
{
    INT32U    uClkFreq;

    /* Print system clock */
    uClkFreq = CLK_GetClockFreq();
    PRINTF("Clk=", uClkFreq, PRINTF_FORMAT_DEC);

    /* Print reason of reset */
    if (RST_GetFlagStatus(RST_FLAG_PORF))
    {
        RST_ClearFlag(RST_FLAG_PORF);
        PRINTF(" Hz, reset=POR.", 0, PRINTF_FORMAT_NONE);
    }
    else if (RST_GetFlagStatus(RST_FLAG_SWIMF))	
    {
        RST_ClearFlag(RST_FLAG_SWIMF);
        PRINTF(" Hz, reset=SWIM.", 0, PRINTF_FORMAT_NONE);
    }
    else if (RST_GetFlagStatus(RST_FLAG_ILLOPF))	
    {
        RST_ClearFlag(RST_FLAG_ILLOPF);
        PRINTF(" Hz, reset=Illigal opcode.", 0, PRINTF_FORMAT_NONE);
    }
    else if (RST_GetFlagStatus(RST_FLAG_IWDGF))	
    {
        RST_ClearFlag(RST_FLAG_IWDGF);
        PRINTF(" Hz, reset=Independent watchdog.", 0, PRINTF_FORMAT_NONE);
    }
    else if (RST_GetFlagStatus(RST_FLAG_WWDGF))	
    {
        RST_ClearFlag(RST_FLAG_WWDGF);
        PRINTF(" Hz, reset=Window watchdog.", 0, PRINTF_FORMAT_NONE);
    }
    else if (RST_GetFlagStatus(RST_FLAG_BORF))	
    {
        RST_ClearFlag(RST_FLAG_BORF);
        PRINTF(" Hz, reset=BOR.", 0, PRINTF_FORMAT_NONE);
    }
    PRINTF("\r\n", 0, PRINTF_FORMAT_NONE);

    return;
}


/*---------------------------------------------------------------------------*/
PROCESS(monitor_process, "Monitor process");

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(monitor_process, ev, data)
{
    PROCESS_BEGIN();

    while (1)
    {
        PROCESS_YIELD(); /* Yield process until received a poll or an event */

        if (PROCESS_EVENT_POLL == ev)
        {
            /* Process this received command */
            dp_GetCmd(s_abyPCCmdBuf, SIZE_BUF_PC_CMD);
            ProcPCCmd();
        }
    }

    PROCESS_END();
}

/*---------------------------------------------------------------------------------------------*/
void monitor_ProcPCCmd(void)
{
    /* Inform monitor_process to deal with this received command */
    process_poll(&monitor_process);	

    return;
}


/*---------------------------------------------------------------------------------------------*/
void monitor_Init(void)
{
    process_start(&monitor_process, NULL);

    return;
}
#endif /*#if (!REL_VER)*/


/*--------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/

