/*
************************************************************************************************
* Filename   	: Dbg.h
* Programmer : JiangJun
* Description	: For system debug
* Date           : 2014-08-22
************************************************************************************************
*/
#ifndef __DBG_H__
#define __DBG_H__



/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include "main.h"
#include "Util.h"
#include <assert.h>


/*
*********************************************************************************************************
*                                                                      		DEBUG SYSTEM
*********************************************************************************************************
*/
/****************  followed have no need to modify by user  *****************/
#if REL_VER /* Release version */
    #define DBG_VER    0
    #define PRINT_WAY    0 /* 0=none */
#else /* Debug version */
    #define DBG_VER    1
    #define PRINT_WAY    2 /* 2=UART */
#endif

#if (0 == DBG_VER)
    #define  NDEBUG    /* turn off "ASSERT()" */
#endif

/* ATTENTION: PRINTF=>"dbg_Printf()" can NOT been called by ISR */
#undef PRINTF    /* Undefine "PRINTF", prevent define repeatly */
#if (0 == PRINT_WAY)
    #define PRINTF(fmt, args...)
#elif (1 == PRINT_WAY)
    #define PRINTF(fmt, args...)    printf(fmt, ##args)
#elif (2 == PRINT_WAY)
    #define PRINTF(str, data, format)    dbg_Printf(str, data, format)
#endif

#undef PRINTFF
#define PRINTFF(fmt, args...)    // do nothing, only a placeholder

#undef PRINT_STR
#if (0 == PRINT_WAY)
    #define PRINT_STR(str)
#elif (1 == PRINT_WAY)
    #define PRINT_STR(str)    printf("%s", str)
#elif (2 == PRINT_WAY)
    #define PRINT_STR(str)    dbg_Printf(str, 0, PRINTF_FORMAT_NONE)
#endif

#undef UNPRINT_STR
#define UNPRINT_STR(str) /* do nothing, only a placeholder */

/*##########################    ASSERT DEFINITION    ############################*/
#undef ASSERT	/* remove existing definition */
#ifdef NDEBUG
    #define ASSERT(test)  NULL
#else/*NDEBUG*/
    extern void _AssertUART(const char *, unsigned int);
    #if (2 == PRINT_WAY)
        #define ASSERT(test)    \
        if (test)    \
            NULL;    \
        else    \
            _AssertUART(__FILE__, __LINE__)
    #else
        #define ASSERT(test)    assert(test)
    #endif
#endif/*NDEBUG*/


/*
*********************************************************************************************************
*                                                                      	MACRO DEFINITION
*********************************************************************************************************
*/
#define MEM_GARBAGE    0xA3    /* garbage for debug memory */

/* Format for printf */
typedef enum
{
    PRINTF_FORMAT_NONE = (INT8U)0, /* Don't format data */
    PRINTF_FORMAT_DEC = (INT8U)1, /* Format data to decimal */
    PRINTF_FORMAT_HEX = (INT8U)2, /* Format data to hexadecimal */
} PRINTF_FORMAT;


/*
*********************************************************************************************************
*                                            				     DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/
/* User select level as: ALL, WARNING, SERIOUS, SEVERE */
#define RIME_DBG_MIN_LEVEL    RIME_DBG_LEVEL_ALL

/* User select types as: ON, TRACE, STATE, FRESH */
#define RIME_DBG_TYPES_ON    RIME_DBG_ON

/** lower two bits indicate debug level
 * - 0 all
 * - 1 warning
 * - 2 serious
 * - 3 severe
 */
#define RIME_DBG_LEVEL_ALL     0x00
#define RIME_DBG_LEVEL_WARNING 0x01 /* bad checksums, dropped packets, ... */
#define RIME_DBG_LEVEL_SERIOUS 0x02 /* memory allocation failures, ... */
#define RIME_DBG_LEVEL_SEVERE  0x03
#define RIME_DBG_MASK_LEVEL    0x03

/** flag for RIME_DBG to enable that debug message */
#define RIME_DBG_ON            0x80U
/** flag for RIME_DBG to disable that debug message */
#define RIME_DBG_OFF           0x00U

/** flag for RIME_DBG indicating a tracing message (to follow program flow) */
#define RIME_DBG_TRACE         0x40U
/** flag for RIME_DBG indicating a state debug message (to follow module states) */
#define RIME_DBG_STATE         0x20U
/** flag for RIME_DBG indicating newly added code, not thoroughly tested yet */
#define RIME_DBG_FRESH         0x10U
/** flag for RIME_DBG to halt after printing this debug message */
#define RIME_DBG_HALT          0x08U


#if DBG_VER
/** print debug message only if debug message type is enabled...
 *  AND is of correct type AND is at least RIME_DBG_LEVEL
 */
#define RIME_DBG(dbg, fmt, args...)    \
    do    \
    {    \
        if ( ((dbg) & RIME_DBG_ON) &&    \
              ((dbg) & RIME_DBG_TYPES_ON) &&    \
              (RIME_DBG_MIN_LEVEL <= (int16_t)((dbg) & RIME_DBG_MASK_LEVEL)) )    \
        {    \
            dbg_PrintfArg(fmt, ##args);    \
        }    \
        if ((dbg) & RIME_DBG_HALT)    \
        {    \
            while (1) ;    \
        }    \
    } while (0)

/** Light weight debug that do NOT use "vsnprintf()" which needs many ROM/RAM.
  *   call this 3 format as:
  *   (1) RIME_DBG2(BLOCK_DBG, "only string\r\n", 0, PRINTF_FORMAT_NONE);
  *   (2) RIME_DBG2(BLOCK_DBG, "\r\ndec data=", dec, PRINTF_FORMAT_DEC);
  *   (3) RIME_DBG2(BLOCK_DBG, "\r\nhex data=", hex, PRINTF_FORMAT_HEX);
 */
#define RIME_DBG2(dbg, str, data, fmt)    \
    do    \
    {    \
        if ( ((dbg) & RIME_DBG_ON) &&    \
              ((dbg) & RIME_DBG_TYPES_ON) &&    \
              (RIME_DBG_MIN_LEVEL <= (int16_t)((dbg) & RIME_DBG_MASK_LEVEL)) )    \
        {    \
            dbg_Printf(str, data, fmt);    \
        }    \
        if ((dbg) & RIME_DBG_HALT)    \
        {    \
            while (1) ;    \
        }    \
    } while (0)
#else
#define RIME_DBG(dbg, fmt, args...) 
#define RIME_DBG2(dbg, str, data, fmt) 
#endif


/*
*********************************************************************************************************
*                                                                        FUNCTION PROTOTYPE
*********************************************************************************************************
*/
extern void dbg_PrintfArg(const char *p_chFormat, ...);
extern void dbg_Printf(const char *p_chStr, INT32S lData, PRINTF_FORMAT eFormat);
extern INT8U FormatPrintf(char *p_chBuf, const char *p_chStr, INT32S lData, PRINTF_FORMAT eFormat);
extern void dbg_EnablePrint(void);
extern void dbg_DisablePrint(void);
extern void dbg_Tx(const void *p_vSrcBuf, INT16S nLen);
extern void dbg_MeasureTimeUSStart(void);
extern void dbg_MeasureTimeUSEnd(const char *p_strPrint);
extern void dbg_MeasureTimeMSStart(void);
extern void dbg_MeasureTimeMSEnd(const char *p_strPrint);
extern void dbg_DelayMS(INT16U wMS);
extern void dbg_PrintfBufData(const void *p_vBuf, uint8_t bySize);


#endif    /* __DBG_H__ */

