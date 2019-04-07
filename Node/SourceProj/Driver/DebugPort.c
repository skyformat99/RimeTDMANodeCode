/*
************************************************************************************************
* Filename   	: DebugPort.c
* Programmer : JiangJun
* Description	: Debug port by USARTx 
* Date           : 2015-01-14
************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include <string.h>
#include "stm8l15x_clk.h"
#include "stm8l15x_usart.h"
#include "stm8l15x_dma.h"
#include "Dbg.h"
#include "DebugPort.h"
#include "Monitor.h"


/*
*********************************************************************************************************
*                                                                        COMPILE SWITCH
*********************************************************************************************************
*/
#if (2 == PRINT_WAY)

/*
*********************************************************************************************************
*                                                                       MACRO DEFINITION
*********************************************************************************************************
*/
#if defined DP_PORT_IS_USART1
    #define DP_PORT    USART1
    #define DP_TX_DMA_CH    DMA1_Channel1
    #define DP_TX_DMA_IT_TC    DMA1_IT_TC1
#elif defined DP_PORT_IS_USART2
    #define DP_PORT    USART2
    #define DP_TX_DMA_CH    DMA1_Channel0
    #define DP_TX_DMA_IT_TC    DMA1_IT_TC0
#elif defined DP_PORT_IS_USART3
    #define DP_PORT    USART3
    #define DP_TX_DMA_CH    DMA1_Channel1
    #define DP_TX_DMA_IT_TC    DMA1_IT_TC1
#endif

#define DP_PORT_BAUD_RATE    115200u   
#define USART_DR_OFFSET    GET_ST_FLD_OFFSET(USART_TypeDef, DR)

#define CHAR_CR    0x0Du /* carriage return */


/*
*********************************************************************************************************
*                                            				   DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/
#define NUM_DP_CMD    2 /* number of DP command */
#define MAX_SIZE_DP_CMD_BUF    32 /* maximum size of buffer saved DP command */

/* save command received by DP */
typedef struct _dp_cmd
{
    INT8U    byLen;    /* length of valid command string exclude '\0' */
    INT8U    a_byBuf[MAX_SIZE_DP_CMD_BUF];    /* buffer of command */	
} DP_CMD;

/* ring buffer to deal with command */
typedef struct _dp_cmd_set
{
    volatile INT8U    byHead;    /* index of producer command */
    volatile INT8U    byTail;    /* index of consumer command */
    DP_CMD    a_stCmd[NUM_DP_CMD];    /* buffer saved command */
} DP_CMD_SET;

/* Flag indicated that DMA is busy */
static volatile BOOLEAN    s_bIsDMABusy = FALSE;

/**
* @brief  Ring buffer to save the TX data.
*/
#define SIZE_RING_BUF    256
typedef struct _ring_buf
{
    volatile int16_t    nHead;
    volatile int16_t    nTail;
    volatile int16_t    nDMASize;
    uint8_t    a_byBuf[SIZE_RING_BUF];
} RING_BUF;


/*
*********************************************************************************************************
*                                                                LOCAL VARIABLE & STRUCTURE
*********************************************************************************************************
*/
/* save command received by DP */
static DP_CMD_SET    s_stDPCmdSet;

static RING_BUF    s_stRingBuf;


/*
*********************************************************************************************************
*                                                                       FUNCTION PROTOTYPE
*********************************************************************************************************
*/


/*---------------------------------------------------------------------------*/
void DMA_SetSrcAddr( DMA_Channel_TypeDef* DMA_Channelx,
                                          uint16_t DMA_Memory0BaseAddr )
{
    DMA_Channelx->CM0ARH = (uint8_t)(DMA_Memory0BaseAddr >> (uint8_t)8);
    DMA_Channelx->CM0ARL = (uint8_t)(DMA_Memory0BaseAddr);

    return;
}

/**
  * @brief  Check whether the DMA is running.
  * @param  None
  * @retval  TRUE=is running; FALSE=is NOT running.
  */
static bool rfd_DMAIsRun(void)
{
    if (s_bIsDMABusy)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/**
  * @brief  Start the DMA to transmit data.
  * @param  None
  * @retval  None
  */
static void rfd_StartDMA(void)
{
    int16_t    nCnt;

    if (s_stRingBuf.nTail < s_stRingBuf.nHead)
    {
        nCnt = s_stRingBuf.nHead - s_stRingBuf.nTail; /* Tail to Head */
    }
    else
    {
        nCnt = SIZE_RING_BUF - s_stRingBuf.nTail; /* Tail to End of buffer */	
    }
    s_stRingBuf.nDMASize = nCnt;

    /* EXPLAIN: Set address and counter MUST disable DMA. */
    DMA_Cmd(DP_TX_DMA_CH, DISABLE);
    DMA_SetSrcAddr(DP_TX_DMA_CH, (uint16_t)&s_stRingBuf.a_byBuf[s_stRingBuf.nTail]);
    DMA_SetCurrDataCounter(DP_TX_DMA_CH, s_stRingBuf.nDMASize);

    /* ATTENTION: set "s_bIsDMABusy" before enable DMA to prevent race condition. */
    s_bIsDMABusy = TRUE;
    DMA_Cmd(DP_TX_DMA_CH, ENABLE);

    return;
}

/*
************************************************************************************************
*			             	                  			       Put Char of Command
* Description : Put character received from DP into command buffer   
* Arguments  : INT8U byChar    received character through USART port
* Returns      : void 
* Notes        : (1) only can called this by DP ISR; 
*                   (2) it would replace carriage-return to '\0';
************************************************************************************************
*/
static void PutCmdChar(INT8U byChar)
{
    DP_CMD    *p_stCmd;

    p_stCmd = &s_stDPCmdSet.a_stCmd[s_stDPCmdSet.byHead];

    if (CHAR_CR != byChar)
    {
        p_stCmd->a_byBuf[p_stCmd->byLen] = byChar;    /* save this character */
        if (p_stCmd->byLen < (MAX_SIZE_DP_CMD_BUF - 1))    /* have room */
        {
            ++p_stCmd->byLen;
            return;			
        }
        else    /* have no room so deal with this command */
        {
            goto finish_cmd;    /* add "goto" for readable */
        }
    }

    /* RX carriage-return or command buffer have no room */
finish_cmd:
    p_stCmd->a_byBuf[p_stCmd->byLen] = '\0';    /* replace carriage-return to '\0' */

    /* Inform monitor_process to deal with this command */
    monitor_ProcPCCmd();

    if (++s_stDPCmdSet.byHead >= NUM_DP_CMD)
    {
        s_stDPCmdSet.byHead = 0;    /* warp to 1th unit */
    }
    ASSERT(s_stDPCmdSet.byHead != s_stDPCmdSet.byTail);    /* Error! have not process CMD */

    return;
}


/*
************************************************************************************************
*			             	                  			       Get Command
* Description : Get command string that received through DP port   
* Arguments  : void * p_vSaveBuf    point to buffer which saved this command string by this pointer
*                    INT8U byLen    length of desired copy command string 
* Returns      : INT8U    size of copied command string NOT include '\0' 
* Notes        : (1) only can called this by Monitor-Task; 
*                   (2) Always append '\0';
************************************************************************************************
*/
INT8U dp_GetCmd(void *p_vSaveBuf, INT8U byLen)
{
    ASSERT(p_vSaveBuf);

    DP_CMD    *p_stCmd;

    p_stCmd = &s_stDPCmdSet.a_stCmd[s_stDPCmdSet.byTail];

    /* Copy command string */
    byLen = MIN(byLen, (p_stCmd->byLen + 1));    /* add 1 for '\0', insure copy NOT exceed */
    memcpy(p_vSaveBuf, p_stCmd->a_byBuf, byLen);
    ((char *)p_vSaveBuf)[byLen - 1] = '\0';    /* force terminator='\0' if size of buffer less than string length */	

    /* Adjust Command Buffer Set */
    p_stCmd->byLen = 0;    /* ATTENTION: must clear to 0 since release this unit */
    if (++s_stDPCmdSet.byTail >= NUM_DP_CMD)
    {
        s_stDPCmdSet.byTail = 0;    /* warp to 1th unit */
    }

    return byLen;	
}


/*
************************************************************************************************
*			             	                  			       TX by DP
* Description : Transmit data by DP port   
* Arguments  : const void * p_vSrcBuf    point to buffer which saved the desired TX data
*                    INT16S nLen    length of desired TX data 
* Returns      : INT8S    -1=Tx data too long; -2=UART hardware error; 0=TX OK
* Notes        : (1) It may block procedure while on previous transmission;
*                   (2) USART1<=>DMA1_Channel1, USART2<=>DMA1_Channel0, USART3<=>DMA1_Channel1;
************************************************************************************************
*/
INT8S dp_Tx(const void *p_vSrcBuf, INT16S nSize)
{
    int16_t    nCopySize, nRemainSize;
    const uint8_t    *p_bySrcBuf;

    p_bySrcBuf = (const uint8_t *)p_vSrcBuf;	

    /* Check if have room to save string */
    nRemainSize = util_GetRoomCBuf(s_stRingBuf.nHead, s_stRingBuf.nTail, SIZE_RING_BUF);
    if (nRemainSize < nSize)
    {
        return -1;    /* Have no room */
    }

    /* Copy those data into ring buffer */
    nRemainSize = SIZE_RING_BUF - s_stRingBuf.nHead;
    nCopySize = MIN(nSize, nRemainSize);	
    memcpy(&s_stRingBuf.a_byBuf[s_stRingBuf.nHead], p_bySrcBuf, nCopySize);	
    s_stRingBuf.nHead += nCopySize;
    if (SIZE_RING_BUF <= s_stRingBuf.nHead)
    {
        s_stRingBuf.nHead = 0;
    }

    /* Check whether need to wrap to begin of ring buffer. */
    nRemainSize = nSize - nCopySize;	
    if (0 < nRemainSize)
    {
        memcpy(&s_stRingBuf.a_byBuf[0], &p_bySrcBuf[nCopySize], nRemainSize);	
        s_stRingBuf.nHead = nRemainSize;
    }

    /* Check if need to START DMA */
    if (!rfd_DMAIsRun())
    {
        rfd_StartDMA();
    }

    return 0;
}


/*
************************************************************************************************
*			             	                  			     Initialize USARTx Port
* Description : Initialize USARTx port for communication to PC   
* Arguments  : void 
* Returns      : void
* Notes        : (1) USART1<=>DMA1_Channel1, USART2<=>DMA1_Channel0, USART3<=>DMA1_Channel1;
************************************************************************************************
*/
void dp_Init(void)
{
    /* Enable clock to USARTx and DMA1 */
    CLK_PeripheralClockConfig(CLK_Peripheral_DMA1, ENABLE);	

#if defined DP_PORT_IS_USART1
    CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
#elif defined DP_PORT_IS_USART2
    CLK_PeripheralClockConfig(CLK_Peripheral_USART2, ENABLE);
#elif defined DP_PORT_IS_USART3
    CLK_PeripheralClockConfig(CLK_Peripheral_USART3, ENABLE);
#endif

    /* Configure USART Tx and RX as alternate function push-pull(software pull up) */
#if defined DP_PORT_IS_USART1
    GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_2, ENABLE);
    GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_3, ENABLE);
#elif defined DP_PORT_IS_USART2
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_3, ENABLE);
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_4, ENABLE);
#elif defined DP_PORT_IS_USART3
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_6, ENABLE);
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_7, ENABLE);
#endif

    /* Initialize USARTx */
    USART_Init( DP_PORT, 
                       DP_PORT_BAUD_RATE, 
                       USART_WordLength_8b, 
                       USART_StopBits_1, 
                       USART_Parity_No, 
                       (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx) );

    /* Enable the USARTx Receive interrupt: this interrupt is generated when the USARTx
        receive data register is not empty */
    USART_ITConfig(DP_PORT, USART_IT_RXNE, ENABLE);

    /* Deinitialize DMA channels */
    DMA_GlobalDeInit();
    DMA_DeInit(DP_TX_DMA_CH);

    DMA_Init( DP_TX_DMA_CH, 
                    (uint32_t)&s_stRingBuf.a_byBuf[0],
                    (uint32_t)DP_PORT + USART_DR_OFFSET,
                    0xFF,
                    DMA_DIR_MemoryToPeripheral,
                    DMA_Mode_Normal,
                    DMA_MemoryIncMode_Inc,
                    DMA_Priority_High,
                    DMA_MemoryDataSize_Byte );

    /* Enable DMA for TX */
    USART_DMACmd(DP_PORT, USART_DMAReq_TX, ENABLE);

    DMA_Cmd(DP_TX_DMA_CH, DISABLE); /* Disable DMA1_Channel1*/

    /* Enable Transaction Complete Interrupt */
    DMA_ITConfig(DP_TX_DMA_CH, DMA_ITx_TC, ENABLE);

    /* Initialize DMA for TX(channel_0<->USART2_TX, channel_1<->USART1_TX) */
    DMA_GlobalCmd(ENABLE); /* Enable DMA1 */

    return;
}


/*---------------------------------------------------------------------------------------------*/
void dp_RxIRQHandler(void)
{
    INT8U    byData;

    /* Read one byte from the received data register and send it back */
    byData = USART_ReceiveData8(DP_PORT);
    USART_SendData8(DP_PORT, byData);

    /* Put the received data into command buffer */
    PutCmdChar(byData);

    return;
}


/*---------------------------------------------------------------------------------------------*/
void dp_DMATxIRQHandler(void)
{
    /* Clear INT pending flag. */
    s_bIsDMABusy = FALSE; /* Transfer completed */
    DMA_ClearITPendingBit(DP_TX_DMA_IT_TC);

    /* Update the index of Tail. */
    s_stRingBuf.nTail += s_stRingBuf.nDMASize;
    if (SIZE_RING_BUF <= s_stRingBuf.nTail)
    {
        s_stRingBuf.nTail = 0;
    }

    /* Check whether need to start DMA again. */
    if (s_stRingBuf.nHead != s_stRingBuf.nTail)
    {
        rfd_StartDMA();
    }

    return;
}

#endif/*#if (2 == PRINT_WAY)*/

/*--------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/

