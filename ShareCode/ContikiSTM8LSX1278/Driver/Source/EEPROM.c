/*
************************************************************************************************
* Filename   	: EEPROM.c
* Programmer : JiangJun
* Description	: operate EEPROM which on chip of STM8L
* Note           : 2015-02-06
************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include <string.h>
#include "stm8l15x_clk.h"
#include "stm8l15x_flash.h"
#include "Dbg.h"
#include "EEPROM.h"


/*
*********************************************************************************************************
*                                                                        COMPILE SWITCH
*********************************************************************************************************
*/
#define EN_DBG_E2    0 /* 0=Disable, 1=Enable */


/*
*********************************************************************************************************
*                          							MACRO DEFINITION
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            				     DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                   LOCAL VARIABLE & STRUCTURE
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                       FUNCTION PROTOTYPE
*********************************************************************************************************
*/


/*---------------------------------------------------------------------------------------------*/
void e2_Init(void)
{
    /* Define flash programming Time*/
    FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);

    return;
}


/*---------------------------------------------------------------------------------------------*/
void e2_Wr(const void *p_vSrc, INT16S nSize, INT16U wAddr)
{
    ASSERT(p_vSrc && (nSize > 0) && (nSize <= E2_TOT_SIZE));
    ASSERT(wAddr >= E2_ADDR_START && (wAddr + nSize - 1) <= E2_ADDR_END);

    INT16S    nCnt;	
    const INT8U    *p_bySrc;

    /* Unlock flash data e2 memory */
    FLASH_Unlock(FLASH_MemType_Data);

    /* Wait until Data E2 area unlocked flag is set*/
    nCnt = 0x7FFF;		
    while (RESET == FLASH_GetFlagStatus(FLASH_FLAG_DUL) && nCnt > 0)
    {
        --nCnt;
    }

    /* Write data into E2 one by one */
    p_bySrc = (const INT8U *)p_vSrc;
    while (nSize-- > 0)
    {
        FLASH_ProgramByte(wAddr++, *p_bySrc++);

        /* Wait until write or erase operation is finished */
        nCnt = 0x7FFF;		
        while ((RESET == FLASH_GetFlagStatus(FLASH_FLAG_HVOFF)) && (nCnt > 0))
        {
            --nCnt;
        }
        ASM_NOP();		
    }

    /* Lock flash data e2 memory */
    FLASH_Lock(FLASH_MemType_Data);

    /* Wait until Data E2 area locked flag is set*/
    nCnt = 0x7FFF;
    while (SET == FLASH_GetFlagStatus(FLASH_FLAG_DUL) && nCnt > 0)
    {
        --nCnt;
    }

     return;
}


/*---------------------------------------------------------------------------------------------*/
void e2_Rd(void *p_vDst, INT16S nSize, INT16U wAddr)
{
    //检查设定参数保存的地址是否已经超出了EEPROM
    ASSERT(p_vDst && (nSize > 0) && (nSize <= E2_TOT_SIZE));
    ASSERT(wAddr >= E2_ADDR_START && (wAddr + nSize - 1) <= E2_ADDR_END);

    INT16S    nCnt;	
    INT8U    *p_byDst;

    /* Unlock flash data e2 memory */
    FLASH_Unlock(FLASH_MemType_Data);

    /* Wait until Data E2 area unlocked flag is set*/
    nCnt = 0x7FFF;		
    while (RESET == FLASH_GetFlagStatus(FLASH_FLAG_DUL) && nCnt > 0)
    {
        --nCnt;
    }

    //这里应该是读数据而不是写数据
    /* Read data from E2 one by one */
    p_byDst = (INT8U *)p_vDst;
    /*从地址0x1000开始读取两个字节*/
    while (nSize-- > 0)
    {
        *p_byDst++ = FLASH_ReadByte(wAddr++);
    }

    /* Lock flash data e2 memory */
    FLASH_Lock(FLASH_MemType_Data);

    /* Wait until Data E2 area locked flag is set*/
    nCnt = 0x7FFF;		
    while (SET == FLASH_GetFlagStatus(FLASH_FLAG_DUL) && nCnt > 0)
    {
        --nCnt;
    }

    return;
}


/*---------------------------------------------------------------------------------------------*/
void e2_Fill(INT8U byData, INT16S nSize, INT16U wAddr)
{
    ASSERT((nSize > 0) && (nSize <= E2_TOT_SIZE));
    ASSERT(wAddr >= E2_ADDR_START && (wAddr + nSize - 1) <= E2_ADDR_END);

    INT16S    nCnt;	

    /* Unlock flash data e2 memory */
    FLASH_Unlock(FLASH_MemType_Data);

    /* Wait until Data E2 area unlocked flag is set*/
    nCnt = 0x7FFF;		
    while (RESET == FLASH_GetFlagStatus(FLASH_FLAG_DUL) && nCnt > 0)
    {
        --nCnt;
    }

    /* Write data into E2 one by one */
    while (nSize-- > 0)
    {
        FLASH_ProgramByte(wAddr++, byData);

        /* Wait until write or erase operation is finished */
        nCnt = 0x7FFF;		
        while ((RESET == FLASH_GetFlagStatus(FLASH_FLAG_HVOFF)) && (nCnt > 0))
        {
            --nCnt;
        }
        ASM_NOP();		
    }

    /* Lock flash data e2 memory */
    FLASH_Lock(FLASH_MemType_Data);

    /* Wait until Data E2 area locked flag is set*/
    nCnt = 0x7FFF;		
    while (SET == FLASH_GetFlagStatus(FLASH_FLAG_DUL) && nCnt > 0)
    {
        --nCnt;
    }
	
    return;
}



//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#if    0
#define TEST_SIZE    E2_TOT_SIZE
#define START_ADDR    E2_ADDR_START
#define FILL_VAL    0xA5

#define DATA_CNT    64 /* MUST be exponent of 2 */
static INT8U    s_abySrc[DATA_CNT];
static INT8U    s_abyDst[DATA_CNT];

//static INT8U    s_abyTestBuf[] = "Test write-read of FM24L256.\r\n";

void e2_Test(void)
{
    INT8U    byData;
    INT32S    lTotCnt, iCnt, lLoop;
    INT16U    wAddr;	

    e2_Init();

#if 1
    e2_Fill(FILL_VAL, TEST_SIZE, START_ADDR);

    wAddr = E2_ADDR_START;
    for (lTotCnt = TEST_SIZE; lTotCnt > 0;)
    {
        iCnt = MIN(lTotCnt, DATA_CNT);
        e2_Rd(s_abyDst, iCnt, wAddr);

        wAddr += iCnt;
        lTotCnt -= iCnt;

        for (lLoop = 0; lLoop < iCnt; ++lLoop)
        {
            if (FILL_VAL != s_abyDst[lLoop])
            {
                goto fill_error;
            }
        }
    }
fill_ok:
    PRINTF("e2_Fill() ok!\r\n", 0, 0);
    goto test_wr;

fill_error:
    PRINTF("e2_Fill() error!\r\n", 0, 0);
    return;
#endif

#if 1
test_wr:
    wAddr = E2_ADDR_START;	
    while (wAddr < E2_ADDR_END)
    {
        byData = (INT8U)rand();    /* Get random data */

        memset(s_abySrc, byData, sizeof(s_abySrc));
        e2_Wr(s_abySrc, sizeof(s_abySrc), wAddr);
        e2_Rd(s_abyDst, sizeof(s_abyDst), wAddr);

        wAddr += sizeof(s_abySrc);		

        if (0 != memcmp(s_abySrc, s_abyDst, sizeof(s_abySrc)))
        {
            goto wr_error;
        }
    }/*while*/	
wr_ok:
    PRINTF("e2_Wr() and e2_Rd() ok!\r\n", 0, 0);
    return;

wr_error:
    PRINTF("e2_Wr() and e2_Rd() error!\r\n", 0, 0);
    return;
#endif	

    return;
}
#endif
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


