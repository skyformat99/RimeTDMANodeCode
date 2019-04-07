/*
************************************************************************************************
* Filename   	: Dbg.c
* Programmer : JiangJun
* Description	: For system debug
* Date           : 2014-08-22
************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "main.h"
#include "Dbg.h"
#include "DebugPort.h"
#include "Rtimer.h"
#include "clock.h"

/*
*********************************************************************************************************
*                                                                        COMPILE SWITCH
*********************************************************************************************************
*/
#if (!REL_VER)
#if (2 == PRINT_WAY)

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


/*
*********************************************************************************************************
*                                                                   GLOBAL VARIABLE & STRUCTURE
*********************************************************************************************************
*/
/* size of buffer that saved debug string */
#define SIZE_DBG_BUF    128
static INT8U    s_abyDbgBuf[SIZE_DBG_BUF];


/*
*********************************************************************************************************
*                                                                   LOCAL VARIABLE & STRUCTURE
*********************************************************************************************************
*/
/* Flag indicate whether enable print messages */
static BOOLEAN    s_bEnablePrint = TRUE;


/*
************************************************************************************************
*                                                                       FUNCTION PROTOTYPE
************************************************************************************************
*/


/*
************************************************************************************************
*			             	                 			Put ASSERT on UART 
* Description : Print information of ASSERT() on UARTx   
* Arguments  : const char *p_chFile    point to string of file name by this pointer
*                    unsinged int uLine    number of line
* Returns      : void
* Notes        : 
************************************************************************************************
*/
void _AssertUART(const char *p_chFile, unsigned int uLine)
{
    INT8U    byLen;

    /* Print this information of ASSERT() */
    dbg_Printf("ASSERT error: ", 0, PRINTF_FORMAT_NONE);

    byLen = strlen(p_chFile);
    if (byLen > (SIZE_DBG_BUF - 12))
    {
        byLen -= (SIZE_DBG_BUF - 12); /* Size of need to move */
        p_chFile += byLen; /* Only can print the piece string */		
    }
    dbg_Printf(p_chFile, uLine, PRINTF_FORMAT_DEC);

    /* Delay a while for UARTx printf() */
    util_Delay(0xFFFF);
	
    abort();    /* Abort system */	
}


/*
************************************************************************************************
*			             	                  			   Printf for debug 
* Description : like the "printf()" but put messages to UART 
* Arguments  : const INT8S *p_chStr    NULL=Not print string; otherwise=printed string
*                    INT32S lData    desired printed data
*                    INT8U eFormat    PRINTF_FORMAT_NONE=Not print data; 
*                                              PRINTF_FORMAT_DEC=Decimal;
*                                              PRINTF_FORMAT_HEX=Hexadecimal;
* Returns      : void 
* Notes        :   
************************************************************************************************
*/
void dbg_Printf(const char *p_chStr, INT32S lData, PRINTF_FORMAT eFormat)
{
    #define PROMPT_ERR    "dbg_Printf() error: string too long.\r\n"

    INT8U    byLen;

    if (!s_bEnablePrint)
    {
        return; /* Exit procedure if DISABLE print */
    }

    if (strlen(p_chStr) > (SIZE_DBG_BUF - 12))
    {
        dp_Tx(PROMPT_ERR, strlen(PROMPT_ERR));
        return;		
    }

    /* Format printed string into buffer */
    byLen = FormatPrintf((char *)s_abyDbgBuf, p_chStr, lData, eFormat);

    /* TX debug string through COMM */
    dp_Tx(s_abyDbgBuf, byLen);

    return;
}


/*
************************************************************************************************
*                                                                Printf variable argument 
* Description : like the "printf()" but put messages to UART 
* Arguments  : const char *p_chFormat, ...    variable arguments 
* Returns      : void 
* Notes        : (1) Call this have no need to wait hardware;   
************************************************************************************************
*/
void dbg_PrintfArg(const char *p_chFormat, ...)
{
    ASSERT(NULL != p_chFormat);

    INT8U    byLen;
    va_list    args;

    if (!s_bEnablePrint)
    {
        return; /* Exit procedure if DISABLE print */
    }

    /* Process a variable number of arguments into array */
    va_start(args, p_chFormat);
    byLen = vsnprintf((char *)s_abyDbgBuf, SIZE_DBG_BUF, p_chFormat, args);
    va_end(args);

    /* TX debug string through COMM */
    dp_Tx(s_abyDbgBuf, byLen);

    return;
}


/*---------------------------------------------------------------------------------------------*/
void dbg_DisablePrint(void)
{
    s_bEnablePrint = FALSE; 
	
    return;
}


/*---------------------------------------------------------------------------------------------*/
void dbg_EnablePrint(void)
{
    s_bEnablePrint = TRUE; 
	
    return;
}


/*---------------------------------------------------------------------------------------------*/
void dbg_Tx(const void *p_vSrcBuf, INT16S nLen)
{
    ASSERT(p_vSrcBuf);

    if (!s_bEnablePrint)
    {
        return; /* Exit procedure if DISABLE print */
    }

    dp_Tx(p_vSrcBuf, nLen);
	
    return;
}

#endif/*#if (2 == PRINT_WAY)*/


/*---------------------------------------------------------------------------------------------*/
static INT16U    s_wPrevUSTmrCnt;

/*---------------------------------------------------------------------------------------------*/
void dbg_MeasureTimeUSStart(void)
{
    s_wPrevUSTmrCnt = RTIMER_NOW(); /* Record start time */

    return;
}

/*---------------------------------------------------------------------------------------------*/
void dbg_MeasureTimeUSEnd(const char *p_strPrint)
{
    INT16U    wEnd;

    wEnd = RTIMER_NOW();
    wEnd -= s_wPrevUSTmrCnt;

    PRINTF(p_strPrint, (1000ul * wEnd), PRINTF_FORMAT_DEC);
    PRINTF(" us.\r\n", 0, PRINTF_FORMAT_NONE);				

    return;
}

/*---------------------------------------------------------------------------------------------*/
static INT16U    s_wPrevMSTmrCnt;

/*---------------------------------------------------------------------------------------------*/
void dbg_MeasureTimeMSStart(void)
{
    s_wPrevMSTmrCnt = RTIMER_NOW(); /* Record start time */

    return;
}

/*---------------------------------------------------------------------------------------------*/
void dbg_MeasureTimeMSEnd(const char *p_strPrint)
{
    INT16U    wEnd;

    wEnd = RTIMER_NOW();
    wEnd -= s_wPrevMSTmrCnt;

    PRINTF(p_strPrint, wEnd, PRINTF_FORMAT_DEC);
    PRINTF(" ms.\r\n", 0, PRINTF_FORMAT_NONE);

    return;
}

/*---------------------------------------------------------------------------------------------*/
#define LINE_DATA_SIZE    MIN(16, (SIZE_DBG_BUF / 3))
static int8_t PrintfLineData(const uint8_t *p_byBuf, int8_t chSize)
{
    int8_t    chCnt;
    uint8_t    byData;

    /* Avoid overflow */
    if (LINE_DATA_SIZE < chSize)
    {
        return -1;
    }

    if (chSize <= 0)
    {
        return -2; /* have nothing to do. */
    }

    for (chCnt = 0; chCnt < chSize; ++chCnt)
    {
        byData = *p_byBuf++;
        s_abyDbgBuf[chCnt * 3 + 0] = DATA_2_ASCII(GET_BYTE_HIGH4(byData));
        s_abyDbgBuf[chCnt * 3 + 1] = DATA_2_ASCII(GET_BYTE_LOW4(byData));
        s_abyDbgBuf[chCnt * 3 + 2] = ' ';
    }
    
    s_abyDbgBuf[chCnt * 3 + 0] = '\r';
    s_abyDbgBuf[chCnt * 3 + 1] = '\n';

    dbg_Tx(s_abyDbgBuf, chCnt * 3 + 2);
	
    return 0;
}

/*---------------------------------------------------------------------------------------------*/
void dbg_PrintfBufData(const void *p_vBuf, uint8_t bySize)
{
    uint8_t    byCnt;
    uint8_t    byRemainCnt;
    uint8_t    byPrintedCnt;
    const uint8_t    *p_byBuf;

    p_byBuf = (const uint8_t *)p_vBuf;
    byRemainCnt = bySize;
    byPrintedCnt = 0;
    while (0 < byRemainCnt)
    {
        byCnt = MIN(byRemainCnt, LINE_DATA_SIZE);
        PrintfLineData(&p_byBuf[byPrintedCnt], byCnt);

        byPrintedCnt += byCnt;
        byRemainCnt -= byCnt;
    }
}

#else /*#if (!REL_VER)*/
void dbg_PrintfArg(const char *p_chFormat, ...)
{
    null();
}
void dbg_PrintfBufData(const void *p_vBuf, uint8_t bySize)
{
    null();
}
#endif /*#if (!REL_VER)*/


/*
************************************************************************************************
*			             	                  			            Format Printf
* Description : Format a string for UART PRINTF   
* Arguments  : INT8S *p_chBuf    point to saved buffer by this pointer
*                    const INT8S *p_chStr    NULL=Not print string; otherwise=printed string
*                    INT32S lData    desired printed data
*                    PRINTF_FORMAT eFormat    format for printf()
* Returns      : INT8U    length of formated string 
* Notes        : 
************************************************************************************************
*/
INT8U FormatPrintf(char *p_chBuf, const char *p_chStr, INT32S lData, PRINTF_FORMAT eFormat)
{
    INT8S    *p_chCur, l_achBuf[12];
    INT8U    byTemp, byMod;	
    INT32U    uData;	

    if (p_chStr)
    {
        strcpy((char *)p_chBuf, p_chStr);
    }
    else
    {
        p_chBuf[0] = '\0';    /* Prevent error that garbage in buffer*/
    }

    if (PRINTF_FORMAT_NONE != eFormat)
    {
        if (PRINTF_FORMAT_DEC == eFormat)    /* Is decimal */
        {
            uData = (lData < 0) ? (-1 * lData) : lData;
            byMod = 10;			
        }
        else    /* Is hexadecimal */
        {
            uData = (INT32U)lData;
            byMod = 16;			
        }/*if(PRINTF_FORMAT_DEC==eFormat)*/
		
        p_chCur = &l_achBuf[sizeof(l_achBuf) - 1];    /* Point to the last unit */
        *p_chCur = '\0';    /* Set terminator */
        do
        {
            byTemp = uData % byMod;
            *--p_chCur = (INT8S)DATA_2_ASCII(byTemp);
        } while (0 != (uData /= byMod));

        if ((PRINTF_FORMAT_DEC == eFormat) && (lData < 0))
        {
            *--p_chCur = '-';    /* Add '-' */
        }

        ASSERT((p_chCur >= &l_achBuf[0]) && (p_chCur <= &l_achBuf[sizeof(l_achBuf) - 1]));
        strcat((char *)p_chBuf, (char *)p_chCur);		
    }

    return (INT8U)strlen((char *)p_chBuf);	
}


/*--------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/

