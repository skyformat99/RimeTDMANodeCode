/*
************************************************************************************************
* Filename   	: Util.h
* Programmer : JiangJun
* Description	: For common utility
* Date           : 2014-08-22
************************************************************************************************
*/
#ifndef __UTIL_H__
#define __UTIL_H__


/*
*********************************************************************************************************
*                                                                      		DEBUG SYSTEM
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                              INCLUDE FILES
*********************************************************************************************************
*/
#include "stm8l15x.h"


/*
*********************************************************************************************************
*                                                                      	   MACRO DEFINITION
*********************************************************************************************************
*/
#define PI    3.1415926535
#define SQRT3    1.7320508
#define FOREVER    0


/*
*********************************************************************************************************
*										   DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/
typedef uint8_t    BOOLEAN;
typedef uint8_t    INT8U; /* Unsigned 8 bit quantity */
typedef int8_t    INT8S; /* Signed 8 bit quantity */
typedef uint16_t    INT16U; /* Unsigned 16 bit quantity */
typedef int16_t    INT16S; /* Signed 16 bit quantity */
typedef uint32_t    INT32U; /* Unsigned 32 bit quantity */
typedef int32_t    INT32S; /* Signed 32 bit quantity */

typedef uint8_t    halIntState_t; /*< Used for save interrupt state */


/* NULL pointer */
#ifndef NULL
#define NULL    ((void *)0)
#endif

/* Sort of message for communication between tasks */
typedef enum
{
    MSG_SORT_RX_PING, /* Receive a ping packet */
} MSG_SORT;

/* Header of message for communication between tasks */
typedef struct _msg_header
{
    INT8U    bySort; /* Sort of message */
    INT8U    byLen; /* Length of data in message */	
} MSG_HEADER;


/*
*********************************************************************************************************
*                                                                          COMMON OPERATION
*********************************************************************************************************
*/
/* adjust value for boundary */
#define  ADJUST(x, max, min)    (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x))) 

#ifndef MAX
#define  MAX(x, y)    (((x) > (y)) ? (x) : (y))
#endif

#ifndef MIN
#define  MIN(x, y)    (((x) < (y)) ? (x) : (y))
#endif

/* get low/high byte in a word(2 bytes) */
#define  GET_LOW(word)    ((word) & 0x00FF)
#define  GET_HIGH(word)     (((word) >> 8) & 0x00FF)

/* make up HIGH byte and LOW byte to a word(2 bytes) */
#define  MAKE_WORD(HIGH, LOW)    (((INT8U)(HIGH) << 8) | ((INT8U)(LOW)))

/* get low/high word in a double-word(4bytes) */
#define GET_LOW_DWORD(iDWord)    ((iDWord) & 0x0000FFFFL)
#define GET_HIGH_DWORD(iDWord)    (((iDWord) >> 16) & 0x0000FFFFL)

/* make up HIGH word and LOW word to a double-word(4 bytes) */
#define MAKE_DWORD(HIGH, LOW)    (((INT32U)(HIGH) << 16) | ((INT32U)(LOW)))

/* get next and previous unit in a circular buffer */
#define GET_NEXT_CIRBUF(current, sz_cirbuf)    (((current) + 1) % sz_cirbuf)
#define GET_PREV_CIRBUF(current, sz_cirbuf)    (((current) + sz_cirbuf - 1) % sz_cirbuf)

/* Make 3 and 4 to 0x34 */
#define MAKE_BYTE(H4, L4)    (((INT8U)(H4) << 4) | ((INT8U)(L4)))

/* Get unit number of an array */
#define SIZE_OF_ARRAY(Array) (sizeof(Array) / sizeof(Array[0]))

/* Get high and low 4 bits of a byte, as 0x34 => 0x03 and 0x04 */
#define GET_BYTE_HIGH4(Byte)    (((Byte) >> 4) & 0x0F)
#define GET_BYTE_LOW4(Byte)    ((Byte) & 0x0F)

/* Convert data(0~15) to ASCII, as 1=>'1', a=>'a' */
#define DATA_2_ASCII(Data)    ((Data) < 10 ? ((Data) + '0') : ((Data) + 'A' - 10))

/* Convert BCD to Byte, as 0x12=>12 */
#define BCD_2_BYTE(bcd)    ((((bcd) >> 4) & 0x0F) * 10 + ((bcd) & 0x0F))

/* Convert Byte to BCD, as 12=>0x12 */
#define BYTE_2_BCD(byte)    (((byte) % 10) | (((byte) / 10) << 4))

/* null statement */
#define null()


/*
*********************************************************************************************************
*                                                                        BYTE ORDER
*********************************************************************************************************
*/
/* indicate this CPU is little-endian or big-endian */
#undef CPU_LITTLE_ENDIAN
#undef CPU_BIG_ENDIAN
#define CPU_LITTLE_ENDIAN    /* LPC1768 is little-endian */

/* swap byte orders for 32 bit integer */
#define  __swap32(x)                 \
  ( (INT32U)                            \
    ( (((x) & 0x000000FFL) << 24) |  \
      (((x) & 0x0000FF00L) << 8 ) |  \
      (((x) & 0x00FF0000L) >> 8 ) |  \
      (((x) & 0xFF000000L) >> 24) )  \
  )

/* swap byte orders for 16 bit integer */
#define  __swap16(x)           \
  ( (INT16U)                      \
    ( (((x) & 0x00FF) << 8) |  \
      (((x) & 0xFF00) >> 8) )  \
  )

/* convert byte orders from CPU to big endian */
#ifdef CPU_LITTLE_ENDIAN
#define cpu_to_be32(x)   __swap32(x) 
#define cpu_to_be16(x)   __swap16(x) 
#else
#define cpu_to_be32(x)   ((INT32U)(x))
#define cpu_to_be16(x)   ((INT16U)(x))
#endif

/* convert byte orders from CPU to little endian */
#ifdef CPU_LITTLE_ENDIAN
#define cpu_to_le32(x)   ((INT32U)(x))
#define cpu_to_le16(x)   ((INT16U)(x))
#else
#define cpu_to_le32(x)   __swap32(x) 
#define cpu_to_le16(x)   __swap16(x) 
#endif

/* convert byte orders from big endian to CPU */
#ifdef CPU_LITTLE_ENDIAN
#define be32_to_cpu(x)   __swap32(x) 
#define be16_to_cpu(x)   __swap16(x) 
#else
#define be32_to_cpu(x)   ((INT32U)(x))
#define be16_to_cpu(x)   ((INT16U)(x))
#endif

/* convert byte orders from little endian to CPU */
#ifdef CPU_LITTLE_ENDIAN
#define le32_to_cpu(x)   ((INT32U)(x))
#define le16_to_cpu(x)   ((INT16U)(x))
#else
#define le32_to_cpu(x)   __swap32(x) 
#define le16_to_cpu(x)   __swap16(x) 
#endif


/*
*********************************************************************************************************
*                                                                        FUNCTION PROTOTYPE
*********************************************************************************************************
*/
#define ASM_NOP()    nop()

 /* Interrupt Macros */
#define HAL_ENABLE_INTERRUPTS()    __enable_interrupt()
#define HAL_DISABLE_INTERRUPTS()    __disable_interrupt()

/* Save interrupt mask and disable interrupts */
#define HAL_ENTER_CRITICAL_SECTION(intState)    \
    do    \
    {    \
        intState = __get_interrupt_state();    \
        __disable_interrupt();    \
    } while (0)

 /* Restore interrupts */
#define HAL_EXIT_CRITICAL_SECTION(intState)    \
    do    \
    {    \
        __set_interrupt_state(intState);    \
    } while (0)

/* Get offset of a specified field in a struct */
#define GET_ST_FLD_OFFSET(ST_TYPE, FLD)    (uint32_t)(&((ST_TYPE *)0)->FLD)

extern void util_DelayMs(INT16U wMs); /* MUST enable RTIMER before call this procedure. */
extern void util_Delay(INT16U wCnt);
extern INT16U util_GetRand16(void);
extern INT16U util_CRC16(const void *p_vDataBuf, INT16S nDataLen);
extern INT16S util_GetRoomCBuf(INT16S nHead, INT16S nTail, INT16S nTotNum);
extern INT8U util_CalcCS(const void *p_vBuf, INT16S nSize);
extern INT16U CRC16_2(const void *p_vDataBuf, INT16S nDataLen);

#endif    /* __UTIL_H__ */

