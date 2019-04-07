/*
************************************************************************************************
* Filename   	: Util.c
* Programmer : ZJU_CEE
* Description	: function sets for common utility
* Date           : 2014-01-15
************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                                        COMPILE SWITCH
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include "Dbg.h"
#include "rtimer.h"
#include "sx1278_ports.h"

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
*                                                                   LOCAL VARIABLE & STRUCTURE
*********************************************************************************************************
*/


/*
************************************************************************************************
*                                                                       FUNCTION PROTOTYPE
************************************************************************************************
*/

/*
************************************************************************************************
*                                                                                Delay Ms
* Description : Delay millisecond
* Arguments  : INT16U wMs    count of millisecond.
* Returns      : void
* Notes        : MUST enabled RTIMER before call this procedure.
************************************************************************************************
*/
void util_DelayMs(INT16U wMs)
{
    DelayMs(wMs);

    return;
}


/*
************************************************************************************************
*                                                                                  Delay
* Description : Delay a while
* Arguments  : INT16U wCnt    count of delayed period
* Returns      : void
* Notes        :  this delay 24ms when wCnt=0xFFFF based on 16MHz of STM8L151C8T6.
************************************************************************************************
*/
void util_Delay(INT16U wCnt)
{
    while (0 != wCnt)
    {
        --wCnt;
    }

    return;
}


/*
************************************************************************************************
*                                                                                Get Random 16
* Description : Get a 16 bits random number
* Arguments  : void
* Returns      : void
* Notes        : 
************************************************************************************************
*/
INT16U util_GetRand16(void)
{
    #define RANDOM_POLY    0x1021
    #define RANDOM_TOP_MOST_BIT    0x8000

    INT8S    chCnt;
    INT16U    random_seed;

    /* Get random seed from real clock */
    random_seed = RTIMER_NOW();

    /* Generate random number */
    for( chCnt = 0; chCnt < 16; chCnt++ )
    {
        if( random_seed & RANDOM_TOP_MOST_BIT )
        {
            random_seed = (random_seed << 1) ^ RANDOM_POLY;
        }
        else
        {
            random_seed = (random_seed << 1);
        }
    }

    return random_seed;
}




/*
************************************************************************************************
*                                                                                  CRC16
* Description : Calculate CRC16 for a specified data stream
* Arguments  : const INT8U *p_byDataBuf    point to data stream by this pointer
*                    INT16S nDataLen    length of data stream
* Returns      : INT16U    value of CRC16
* Notes         : Text=32 words, ROData=256 words, CPU=540 cycles
************************************************************************************************
*/
const static INT8U    s_abyCRC16High[256] = 
{
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

const static INT8U    s_abyCRC16Low[256] = 
{
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 
0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 
0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 
0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 
0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 
0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 
0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 
0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 
0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 
0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 
0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 
0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 
0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 
0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/* That need 2ms for calculating 255 bytes based on STM8L151C8T6 16MHz. */
INT16U util_CRC16(const void *p_vDataBuf, INT16S nDataLen)
{
    INT8U    byCRCHigh, byCRCLow, byIndex;
    const INT8U    *p_byDataBuf;

    byCRCHigh = 0xFFu;
    byCRCLow = 0xFFu;
    p_byDataBuf = (const INT8U *)p_vDataBuf;
    while (nDataLen-- > 0)   //这里的两横是自减符号
    {
        byIndex   = byCRCHigh ^ *p_byDataBuf++;
        byCRCHigh = byCRCLow ^ s_abyCRC16High[byIndex];
        byCRCLow = s_abyCRC16Low[byIndex];
    }
    //crc数据发送时，低八位在前，高八位在后
    return ((byCRCLow << 8) | byCRCHigh);
}

/**
  * @brief  Calculate the CRC16 that use the simple table data.
  * @note  Text=57 words, ROData=16 words, CPU=1120 cycles
  */
static const INT16U    wCRCTalbeAbs[] =
{
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401, 
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400,
};

INT16U CRC16_2(const void *p_vDataBuf, INT16S nDataLen)
{
    INT8U    byChar;
    INT16S    nCnt;
    INT16U    wCRC;
    const INT8U    *p_byDataBuf;

    wCRC = 0xFFFF;
    p_byDataBuf = (const INT8U *)p_vDataBuf;
    for (nCnt = 0; nCnt < nDataLen; nCnt++)
    {
        byChar = *p_byDataBuf++;
        wCRC = wCRCTalbeAbs[(byChar ^ wCRC) & 15] ^ (wCRC >> 4);
        wCRC = wCRCTalbeAbs[((byChar >> 4) ^ wCRC) & 15] ^ (wCRC >> 4);
    }

    return wCRC;
}

/*
************************************************************************************************
*                                                              Get Room of Circle Buffer
* Description : get total free space of circular buffer
* Arguments  : INT8S chHead    index of the head(write) pointer
*                    INT8S chTail    index of the tail(read) pointer
*                    INT8S chTotNum    total number of circle buffer
* Returns      : INT8S    number of free space of this circle buffer
* Notes        : The maximum 
************************************************************************************************
*/
INT16S util_GetRoomCBuf(INT16S nHead, INT16S nTail, INT16S nTotNum)
{
    ASSERT( (nTotNum >= 2) && 
                  (nHead >= 0 && nHead < nTotNum) &&
                  (nTail >= 0 && nTail < nTotNum) );

    if (nHead == nTail) 
    {
        return (nTotNum - 1);
    }

    return ( (nTotNum + nTail - nHead) % nTotNum - 1);
}


/*
************************************************************************************************
*                                                                       Calculate Check Sum  
* Description: Calculate check sum of a specified frame of data  
* Arguments : const INT8U *p_byBuf    point to original data by this pointer
*                   INT32S lSize    size of this frame of data
* Returns     : INT8U    value of check sum
* Notes       : 
************************************************************************************************
*/
INT8U util_CalcCS(const void *p_vBuf, INT16S nSize)
{
    ASSERT(p_vBuf && nSize > 0);

    INT8U    bySum;
    const INT8U    *p_byBuf;

    bySum = 0;
    p_byBuf = (const INT8U *)p_vBuf;	
	
    while (nSize-- > 0)
    {
        bySum += *p_byBuf++;
    }

    return bySum;
}

#if 0
/*---------------------------------------------------------------------------------------------*/
static inline INT8S Hex2INT8S(INT8S chHex)
{
    ASSERT( (chHex >= (INT8S)'0' && chHex <= (INT8S)'9') || 
                  (chHex >= (INT8S)'A' && chHex <= (INT8S)'F') );

    INT8S    chVal;
	
    if (chHex <= '9')    // '0'(0x30) ~ '9'(0x39)    
    {
        chVal = chHex - '0';
    }
    else    // 'A'(0x41) ~ 'F'(0x46)
    {
        chVal = chHex - 'A' + 10;
    }

    return chVal;
}
#endif

