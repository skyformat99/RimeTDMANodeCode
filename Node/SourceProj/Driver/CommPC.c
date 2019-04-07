/*
************************************************************************************************
* Filename   	: CommPC.c
* Programmer : JiangJun
* Description	: UARTx port for communication to PC 
* Date           : 2014-08-25
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
#include "stm8l15x_gpio.h"
#include "stm8l15x_exti.h"
#include "main.h"
#include "Dbg.h"
#include "CommPC.h"
#include "Comm2Trm.h"
#include "PwrManage.h"
#include "Tim4UART.h"


/*
*********************************************************************************************************
*                                                                        COMPILE SWITCH
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                                       MACRO DEFINITION
*********************************************************************************************************
*/
#if defined CPC_PORT_IS_USART1
    #define CPC_PORT    USART1
    #define CPC_TX_DMA_CH    DMA1_Channel1
    #define CPC_TX_DMA_IT_TC    DMA1_IT_TC1
#elif defined CPC_PORT_IS_USART2
    #define CPC_PORT    USART2
    #define CPC_TX_DMA_CH    DMA1_Channel0
    #define CPC_TX_DMA_IT_TC    DMA1_IT_TC0
#elif defined CPC_PORT_IS_USART3
    #define CPC_PORT    USART3
    #define CPC_TX_DMA_CH    DMA1_Channel1
    #define CPC_TX_DMA_IT_TC    DMA1_IT_TC1
#endif

#define CPC_PORT_BAUD_RATE    115200u   
#define USART_DR_OFFSET    GET_ST_FLD_OFFSET(USART_TypeDef, DR)


/*
*********************************************************************************************************
*                                            				   DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/
/* Wake up User-System for UART TX. */
#define AUX_PORT    GPIOA
#define AUX_PIN    GPIO_Pin_4 /* AUX=PA4 */

/* Enternal INT to wake up MCU for UART RX. */
#define EXTI_UART_IOPORT    GPIOA
#define EXTI_UART_PIN    GPIO_Pin_5
#define EXTI_UART_EXTI    EXTI_Pin_5


/*
*********************************************************************************************************
*                                                                LOCAL VARIABLE & STRUCTURE
*********************************************************************************************************
*/
/* Buffer for CPC TX */
#define SIZE_CPC_TX_BUF    255
static INT8U    s_abyCPCTxBuf[SIZE_CPC_TX_BUF];

/* Flag indicated that DMA is busy */
static volatile BOOLEAN    s_bIsDMABusy = FALSE;

/* Whether enters the period of UART RX. */
static volatile bool    s_bEnterUARTRx = FALSE;


/*
*********************************************************************************************************
*                                                                      FUNCTION PROTOTYPE
*********************************************************************************************************
*/
/* Set AUX=PA4 to OUTPUT */
#define INIT_AUX()    GPIO_Init(AUX_PORT, AUX_PIN, GPIO_Mode_Out_PP_High_Slow)
#define SET_AUX_LOW()    GPIO_ResetBits(AUX_PORT, AUX_PIN)
#define SET_AUX_HIGH()    GPIO_SetBits(AUX_PORT, AUX_PIN)


/*
************************************************************************************************
*			             	                  			       TX by CPC
* Description : Transmit data by CPC port   
* Arguments  : const void * p_vSrcBuf    point to buffer which saved the desired TX data
*                    INT16S nLen    length of desired TX data 
* Returns      : INT8S    -1=Tx data too long; -2=UART hardware error; 0=TX OK
* Notes        : (1) It may block procedure while on previous transmission;
*                   (2) USART1<=>DMA1_Channel1, USART2<=>DMA1_Channel0, USART3<=>DMA1_Channel1;
************************************************************************************************
*/
INT8S cpc_Tx(const void *p_vSrcBuf, INT16S nLen)
{
    INT16S    nCnt;

    if (nLen > SIZE_CPC_TX_BUF)
    {
        return -1; /* Tx data too long */
    }

    /* Waiting until the DMA is not busy */
    nCnt = 0;	
    while (s_bIsDMABusy)
    {
        /* 0x7FFF=>24.5ms, 115200/9/1000*24.5=313.6B > 255B */
        if (++nCnt >= 0x7FFF)
        {
            return -2; /* UART hardware error: Send used too long time */
        }
    }

    /* Pull down AUX indicated that start TX a UART frame. */
    SET_AUX_LOW();

#if (COMPANY_JiKang == CUR_COMPANY)
    util_DelayMs(10); /* Delay 10ms for wake of MCU of JiKang company. */
#endif

    /* Copy data into buffer */
    memcpy(s_abyCPCTxBuf, p_vSrcBuf, nLen);	

    s_bIsDMABusy = TRUE;

    /* Write this must DISABLE DMA_Channelx */
    DMA_SetCurrDataCounter(CPC_TX_DMA_CH, (INT8U)nLen); 

    DMA_Cmd(CPC_TX_DMA_CH, ENABLE); /* Enable DMA1_Channelx */

    return 0;
}


#if EXTI_HALT_UART
/*---------------------------------------------------------------------------*/
/* Set Socket_Con2<->PA5 to EXTI_INT pin. */
static void SetExti4UART(void)
{
    halIntState_t    intState;

    /* ATTENTION: that MUST disable global interrupts when configure PIN to EXTII, 
        otherwise incurred chaos of external interrupts. */
    HAL_ENTER_CRITICAL_SECTION(intState);

    /* Configure DIO0 as input pull up, external interrupt, rising edge trigger */
    GPIO_Init(EXTI_UART_IOPORT, EXTI_UART_PIN, GPIO_Mode_In_FL_IT);
    EXTI_SetPinSensitivity(EXTI_UART_EXTI, EXTI_Trigger_Rising);

    HAL_EXIT_CRITICAL_SECTION(intState);

    return;
}

/*---------------------------------------------------------------------------*/
void exti4uart_ISR(void)
{
    if (s_bEnterUARTRx)
    {
        return; /* Don't disturb the receiving of UART. */
    }
    else
    {
        s_bEnterUARTRx = TRUE;
    }

    pm_OpposeHalt(PWR_ID_COMM2TRM); /* Don't HALT in the period of UART RX. */
    comm2trm_BeginRxData(); /* MUST set before "cpc_EnRx()". */
    cpc_EnRx(); /* Enable RX of UART. */
    tim4uart_Start(MAX_TIME_UART_RX); /* Enable TIM for timeout of UART RX. */

    return;
}

/*---------------------------------------------------------------------------*/
void exti4uart_End(void)
{
    s_bEnterUARTRx = FALSE;
    pm_AgrHalt(PWR_ID_COMM2TRM); /* Agree HALT that RX and process a frame finished. */

    return;    
}
#endif/*#if EXTI_HALT_UART*/


/*
************************************************************************************************
*			             	                  			     Initialize USARTx Port
* Description : Initialize USARTx port for communication to PC   
* Arguments  : void 
* Returns      : void
* Notes        : (1) USART1<=>DMA1_Channel1, USART2<=>DMA1_Channel0, USART3<=>DMA1_Channel1;
************************************************************************************************
*/
void cpc_Init(void)
{
    /* Enable clock to USARTx and DMA1 */
    CLK_PeripheralClockConfig(CLK_Peripheral_DMA1, ENABLE);	

#if defined CPC_PORT_IS_USART1
    CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
#elif defined CPC_PORT_IS_USART2
    CLK_PeripheralClockConfig(CLK_Peripheral_USART2, ENABLE);
#elif defined CPC_PORT_IS_USART3
    CLK_PeripheralClockConfig(CLK_Peripheral_USART3, ENABLE);
#endif

    /* Configure USART Tx and RX as alternate function push-pull(software pull up) */
#if defined CPC_PORT_IS_USART1
    GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_2, ENABLE);
    GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_3, ENABLE);
#elif defined CPC_PORT_IS_USART2
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_3, ENABLE);
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_4, ENABLE);
#elif defined CPC_PORT_IS_USART3
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_6, ENABLE);
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_7, ENABLE);
#endif

    /* Initialize USARTx as well as enable TX, that NOT enable RX. */
    USART_Init( CPC_PORT, 
                       CPC_PORT_BAUD_RATE, 
                       USART_WordLength_8b, 
                       USART_StopBits_1, 
                       USART_Parity_No, 
                       (USART_Mode_TypeDef)USART_Mode_Tx );

    /* Enable the USARTx Receive interrupt: this interrupt is generated when the USARTx
        receive data register is not empty */
    USART_ITConfig(CPC_PORT, USART_IT_RXNE, ENABLE);

    /* Deinitialize DMA channels */
    DMA_GlobalDeInit();
    DMA_DeInit(CPC_TX_DMA_CH);

    /* Initialize TX DMA */
    DMA_Init( CPC_TX_DMA_CH, 
                    (uint32_t)&s_abyCPCTxBuf[0],
                    (uint32_t)CPC_PORT + USART_DR_OFFSET,
                    SIZE_CPC_TX_BUF,
                    DMA_DIR_MemoryToPeripheral,
                    DMA_Mode_Normal,
                    DMA_MemoryIncMode_Inc,
                    DMA_Priority_High,
                    DMA_MemoryDataSize_Byte );

    /* Turn on UART_TX but not for DMA_TX */
    USART_DMACmd(CPC_PORT, USART_DMAReq_TX, ENABLE);
    DMA_Cmd(CPC_TX_DMA_CH, DISABLE); 

    /* Enable DMA Transaction Complete Interrupt */
    DMA_ITConfig(CPC_TX_DMA_CH, DMA_ITx_TC, ENABLE);

    /* Initialize DMA for TX and RX */
    DMA_GlobalCmd(ENABLE); /* Enable DMA1 */

    /* Followed MUST set behind initialization of UART. */
    INIT_AUX();
#if EXTI_HALT_UART
    tim4uart_Init();
    SetExti4UART(); /* This MUST set in the LAST! */
#else
    cpc_EnRx();
#endif

    return;
}

/*---------------------------------------------------------------------------------------------*/
void cpc_RxIRQHandler(void)
{
    INT8U    byData;

    /* Check whether is an error of overrun */
    if (USART_GetFlagStatus(CPC_PORT, USART_FLAG_OR))
    {
        /* It is cleared by a software sequence: a read to the USART_SR register
            followed by a read to the USART_DR register. */
    }	

    /* EXPLAIN: Interrupt flag of "RXNE" is cleared by a read to the USART_DR register */
    byData = USART_ReceiveData8(CPC_PORT);

    /* Put the received data into received buffer */
    comm2trm_RxUartData(byData);

    return;
}

/*---------------------------------------------------------------------------------------------*/
void cpc_DMATxIRQHandler(void)
{
    s_bIsDMABusy = FALSE; /* Transfer completed */

    DMA_ClearITPendingBit(CPC_TX_DMA_IT_TC);

    DMA_Cmd(CPC_TX_DMA_CH, DISABLE); /* Disable DMA1_Channelx */

#if EXTI_HALT_UART
    /* Delay 1ms(11B on 115200) for transfer the last 2 byte in TDR and Shift-Reg. */
    util_DelayMs(1);
    exti4uart_End(); /* Agree HALT and enable next RX. */
#endif

    /* Pull up AUX indicated that TX a UART frame is done. */
    SET_AUX_HIGH();

    return;
}

/*---------------------------------------------------------------------------------------------*/
void cpc_EnRx(void)
{
    /* Clear OR by a read to the USART_SR followed by a read to the USART_DR. */
    USART_GetFlagStatus(CPC_PORT, USART_FLAG_OR);
    USART_ReceiveData8(CPC_PORT); /* Clear RXNE by a read to the USART_DR register. */

    /* Enable INT of UART RX. */
    USART_ITConfig(CPC_PORT, USART_IT_RXNE, ENABLE);

    /* Enable UART RX. */
    CPC_PORT->CR2 |= USART_CR2_REN;

    return;
}

/*---------------------------------------------------------------------------------------------*/
void cpc_DisRx(void)
{
    /* Disable UART RX. */
    CPC_PORT->CR2 &= ~USART_CR2_REN;

    /* Disable INT of UART RX. */
    USART_ITConfig(CPC_PORT, USART_IT_RXNE, DISABLE);

    return;
}


/*--------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/


