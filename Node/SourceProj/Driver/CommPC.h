/*
************************************************************************************************
* Filename   	: CommPC.h
* Programmer : JiangJun
* Description	: UARTx port for communication to PC 
* Date           : 2014-08-25
************************************************************************************************
*/

#ifndef __COMM_PC_H__
#define __COMM_PC_H__


/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include "main.h"
#include "Util.h"


/*
*********************************************************************************************************
*                                                                          DEBUG SYSTEM
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                       MACROS & CONSTANTS
*********************************************************************************************************
*/
/* EXPLAIN: switch USARTx for COMM as
    USART1    #define CPC_PORT_IS_USART1 in "CommPC.h"
    USART2    #define CPC_PORT_IS_USART2 in "CommPC.h"
    USART3    #define CPC_PORT_IS_USART3 in "CommPC.h" */
#if REL_VER
#define CPC_PORT_IS_USART3
#else
#define CPC_PORT_IS_USART2
#endif

#define CPC_PORT_BAUD_RATE    115200u   

/* The time of received 255 bytes data, add 5ms for prepare. */
#define MAX_TIME_UART_RX    (255ul * (8 + 2) * 1000 / CPC_PORT_BAUD_RATE + 5)


/*
*********************************************************************************************************
*                                            				     DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                        FUNCTION PROTOTYPE
*********************************************************************************************************
*/
extern void cpc_Init(void);
extern INT8S cpc_Tx(const void *p_vSrcBuf, INT16S nLen);
extern void cpc_DMATxIRQHandler(void);
extern void cpc_RxIRQHandler(void);
extern void cpc_EnRx(void);
extern void cpc_DisRx(void);
extern void exti4uart_End(void);


#endif    /* __COMM_PC_H__ */

