/*
************************************************************************************************
* Filename   	: SPI.c
* Programmer : JiangJun
* Description	: Process SPI port of STM8L151C8 
* Date           : 2014-09-01
************************************************************************************************
*/



/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include <string.h>
#include "stm8l15x_clk.h"
#include "stm8l15x_spi.h"
#include "Dbg.h"
#include "SPI.h"


/*
*********************************************************************************************************
*                                                                        COMPILE SWITCH
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                                                      	MACRO DEFINITION
*********************************************************************************************************
*/
/* Set NSS=PB4, SCK=PB5, MOSI=PB6 to OUTPUT; MISO=PB7 to INPUT */
#define SPI_GPIO_PORT    GPIOB
#define SPI_GPIO_SCK    GPIO_Pin_5 /* SCK=PB5 */
#define SPI_GPIO_MOSI    GPIO_Pin_6 /* MOSI=PB6 */
#define SPI_GPIO_MISO    GPIO_Pin_7 /* MISO=PB7 */


/*
*********************************************************************************************************
*                                            				     DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/


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


/*
*********************************************************************************************************
*                                                                        FUNCTION PROTOTYPE
*********************************************************************************************************
*/

	

/*
*********************************************************************************************************
*                                                                              Initialize SPI
* Description: Initialize SPI port of STM8L151C8
* Arguments : None.
* Returns     : None
* Note(s)     : 
*********************************************************************************************************
*/
void spi_Init(void)
{
    /* Set NSS=PB4 to OUTPUT */
    GPIO_Init(NSS_GPIO_PORT, NSS_GPIO_PIN, GPIO_Mode_Out_PP_High_Fast);

    /* Enable SPI clock */
    CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);

    /* MUST set the MOSI and SCK at high level otherwise the SPI cann't work. */
    GPIO_ExternalPullUpConfig(SPI_GPIO_PORT, SPI_GPIO_SCK | SPI_GPIO_MOSI, ENABLE);

    /* Configure SPI1, Max SCK frequency of SX1278=10MHz */
    SPI_DeInit(SPI1);

    SPI_Init( SPI1, 
                  SPI_FirstBit_MSB, /* MSB is transmitted first */
                  SPI_BaudRatePrescaler_2, /* Baudrate is Fsysclk/2=8MHz */
                  SPI_Mode_Master, /* Master Mode */
                  SPI_CPOL_Low, /* SCK=0 when idle */
                  SPI_CPHA_1Edge, /* The 1st clock transition is the 1st data capture edge */
                  SPI_Direction_2Lines_FullDuplex, /* 2-line undirection data mode */
                  SPI_NSS_Soft, /* Software slave management enabled */
                  0x07 );

    /* Enable SPI */
    SPI_Cmd(SPI1, ENABLE);
	
    return;
}


/*
*********************************************************************************************************
*                                                                              Turn Off SPI
* Description: Turn off SPI port of STM8L151C8 for saved energy.
* Arguments : None.
* Returns     : None
* Note(s)     : 
*********************************************************************************************************
*/
void spi_TurnOff(void)
{
    /* Disable SPI */
    SPI_Cmd(SPI1, DISABLE);

    /* Disable SPI clock */
    CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, DISABLE);

    return;	
}


/*
*********************************************************************************************************
*                                                                              Turn On SPI
* Description: Turn on SPI port of STM8L151C8 for saved energy.
* Arguments : None.
* Returns     : None
* Note(s)     : 
*********************************************************************************************************
*/
void spi_TurnOn(void)
{
    /* Enable SPI clock */
    CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);

    /* MUST set the MOSI and SCK at high level otherwise the SPI cann't work. */
    GPIO_ExternalPullUpConfig(SPI_GPIO_PORT, SPI_GPIO_SCK | SPI_GPIO_MOSI, ENABLE);

    /* Enable SPI */
    SPI_Cmd(SPI1, ENABLE);

    return;	
}


/*
*********************************************************************************************************
*                                                                              SPI Input Output
* Description: Input & Output 1 byte by SPI port 
* Arguments : INT8U byOut    byte of output
* Returns     : INT8U    byte of input
* Note(s)     : (1) Design this by GPIO and SW because of ERROR that 2 pins: MOSI and MISO are mixed.
*********************************************************************************************************
*/
INT8U spi_InOut(INT8U byOut)
{
    int8_t    chCnt;

    /* Looping while transmit buffer is not empty */
    chCnt = 10; /* Prevent dead loop, the normal count is 1 */
    while (!SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) && (chCnt > 0))
    {
        --chCnt;
    }

    SPI_SendData(SPI1, byOut);

    /* Looping while receive buffer is empty */
    chCnt = 10; /* Prevent dead loop, the normal count is 1 */
    while (!SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) && (chCnt > 0))
    {
        --chCnt;
    }
    ASSERT(chCnt > 0); /* Catch the error of SPI port */	

    return SPI_ReceiveData(SPI1);
}


/*--------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/


