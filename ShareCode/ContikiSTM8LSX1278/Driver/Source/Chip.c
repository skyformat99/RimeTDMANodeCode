/*
************************************************************************************************
* Filename   	: Chip.c
* Programmer : JiangJun
* Description	: Process clock/pins of STM8L151C8 
* Date           : 2014-08-26
************************************************************************************************
*/



/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include <string.h>
#include "Dbg.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_flash.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_pwr.h"
#include "Chip.h"
#include "energest.h"
#include "spi.h"
#include "sx1278_ports.h"


/*
*********************************************************************************************************
*                                                                        COMPILE SWITCH
*********************************************************************************************************
*/
#define LED_CONF_ON    1 /* 0=Disable LED for save energy; 1=Enable LED. */


/*
*********************************************************************************************************
*                                                                      	MACRO DEFINITION
*********************************************************************************************************
*/
#if (iWL880A == CUR_PRODUCT)
#define LED_IOPORT    GPIOD
#define LED_PIN    GPIO_Pin_7

#define LED2_IOPORT    GPIOC
#define LED2_PIN    GPIO_Pin_7
#else
#define LED_IOPORT    GPIOD
#define LED_PIN    GPIO_Pin_0

#define LED2_IOPORT    GPIOA
#define LED2_PIN    GPIO_Pin_3
#endif

/* Start address of Unique ID in FLASH */
#define UNIQUE_ID_END_ADDR    0x4932	

#define OPT3_ADDR    0x4808
#define IWDG_HALT    0x02


/*
*********************************************************************************************************
*                                            				     DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/
/**
* @brief  Configure of MCU GPIO. 
*/
typedef struct
{
    uint8_t    byNum; /* 1~48 */
    GPIO_TypeDef    *p_byPort; /* A~F */
    uint8_t    byPin; /* 0~7 */
    bool    bNeedSWMode; /* TRUE=Switch mode when enter low power, FALSE=otherwise */
    GPIO_Mode_TypeDef    eModeMCURun; /* GPIO mode on MCU run */
    GPIO_Mode_TypeDef    eModeLowPower; /* GPIO mode on low power */	
} CONFIG_MCU_GPIO;

/**
* @brief  MCU Unique ID.详情可见STM8L151C8数据手册67
*/
typedef struct
{
    uint8_t    a_byLot[7];
    uint8_t    byWafer;
    uint16_t    wYCoordinate;
    uint16_t    wXCoordinate;
} UNIQUE_ID;


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
/* Device unique ID */
static DEV_ID    s_tDevID;
static UNIQUE_ID    s_stUniqueID;
static char    s_achSN[sizeof(UNIQUE_ID) * 2 + 1]; /* 1 byte to 2 char, add 1 for '\0' */

/**
* @brief  Flag indicate whether enter Low-Power mode. 
*/
static volatile bool    s_bIsLowPwr = FALSE;

/**
* @brief  Configure of MCU GPIO. 
*/
#define PIN_LP_VCC    GPIO_Mode_Out_PP_High_Slow /* 3.3V */
#define PIN_LP_NC    GPIO_Mode_In_PU_No_IT /* NC */
#define PIN_LP_MCU_IN    GPIO_Mode_In_FL_No_IT /* MCU input */
#define PIN_LP_MCU_OUT    GPIO_Mode_In_FL_No_IT /* MCU output */

const static CONFIG_MCU_GPIO    s_astConfigGPIO[] =
{
#if (iWL880A == CUR_PRODUCT)
    {1, GPIOA, GPIO_Pin_0, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {2, GPIOA, GPIO_Pin_1, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_Out_PP_Low_Slow},
    {3, GPIOA, GPIO_Pin_2, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {4, GPIOA, GPIO_Pin_3, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {5, GPIOA, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_High_Fast, GPIO_Mode_Out_PP_Low_Slow},
    {6, GPIOA, GPIO_Pin_5, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {7, GPIOA, GPIO_Pin_6, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {8, GPIOA, GPIO_Pin_7, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},

    {14, GPIOE, GPIO_Pin_0, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {15, GPIOE, GPIO_Pin_1, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {16, GPIOE, GPIO_Pin_2, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {17, GPIOE, GPIO_Pin_3, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {18, GPIOE, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {19, GPIOE, GPIO_Pin_5, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},

    {22, GPIOD, GPIO_Pin_2, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
    {23, GPIOD, GPIO_Pin_3, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},

    {24, GPIOB, GPIO_Pin_0, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
    {25, GPIOB, GPIO_Pin_1, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
    {26, GPIOB, GPIO_Pin_2, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
    {27, GPIOB, GPIO_Pin_3, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {28, GPIOB, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_High_Fast, GPIO_Mode_In_FL_No_IT},
    {29, GPIOB, GPIO_Pin_5, FALSE, GPIO_Mode_Out_PP_High_Fast, GPIO_Mode_In_FL_No_IT},
    {30, GPIOB, GPIO_Pin_6, FALSE, GPIO_Mode_Out_PP_High_Fast, GPIO_Mode_In_FL_No_IT},
    {31, GPIOB, GPIO_Pin_7, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},

    {32, GPIOF, GPIO_Pin_0, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},

    {33, GPIOD, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {34, GPIOD, GPIO_Pin_5, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {35, GPIOD, GPIO_Pin_6, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},

    {37, GPIOC, GPIO_Pin_0, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {38, GPIOC, GPIO_Pin_1, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},

    {41, GPIOC, GPIO_Pin_2, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {42, GPIOC, GPIO_Pin_3, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {43, GPIOC, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {44, GPIOC, GPIO_Pin_5, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {45, GPIOC, GPIO_Pin_6, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
#else
    /* connected to 3.3V -------------------------------------------------*/
    {1, GPIOA, GPIO_Pin_0, FALSE, PIN_LP_VCC, PIN_LP_VCC},
    {4, GPIOA, GPIO_Pin_3, FALSE, PIN_LP_VCC, PIN_LP_VCC},
    {20, GPIOD, GPIO_Pin_0, FALSE, PIN_LP_VCC, PIN_LP_VCC},

    /* have not connected anything -----------------------------------------*/
    {3, GPIOA, GPIO_Pin_2, FALSE, PIN_LP_NC, PIN_LP_NC},

    {8, GPIOA, GPIO_Pin_7, FALSE, PIN_LP_NC, PIN_LP_NC},

    {15, GPIOE, GPIO_Pin_1, FALSE, PIN_LP_NC, PIN_LP_NC},

    {17, GPIOE, GPIO_Pin_3, FALSE, PIN_LP_NC, PIN_LP_NC},
    {18, GPIOE, GPIO_Pin_4, FALSE, PIN_LP_NC, PIN_LP_NC},
    {19, GPIOE, GPIO_Pin_5, FALSE, PIN_LP_NC, PIN_LP_NC},

    {21, GPIOD, GPIO_Pin_1, FALSE, PIN_LP_NC, PIN_LP_NC},
    {22, GPIOD, GPIO_Pin_2, FALSE, PIN_LP_NC, PIN_LP_NC},
    {23, GPIOD, GPIO_Pin_3, FALSE, PIN_LP_NC, PIN_LP_NC},

    {27, GPIOB, GPIO_Pin_3, FALSE, PIN_LP_NC, PIN_LP_NC},

    {38, GPIOC, GPIO_Pin_1, FALSE, PIN_LP_NC, PIN_LP_NC},

    {41, GPIOC, GPIO_Pin_2, FALSE, PIN_LP_NC, PIN_LP_NC},
    {42, GPIOC, GPIO_Pin_3, FALSE, PIN_LP_NC, PIN_LP_NC},
    {43, GPIOC, GPIO_Pin_4, FALSE, PIN_LP_NC, PIN_LP_NC},
    {44, GPIOC, GPIO_Pin_5, FALSE, PIN_LP_NC, PIN_LP_NC},
    {45, GPIOC, GPIO_Pin_6, FALSE, PIN_LP_NC, PIN_LP_NC},
    {46, GPIOC, GPIO_Pin_7, FALSE, PIN_LP_NC, PIN_LP_NC},

    /* connected to RF -------------------------------------------------*/
    /* MCU input */
    {14, GPIOE, GPIO_Pin_0, FALSE, PIN_LP_MCU_IN, PIN_LP_MCU_IN},

    {24, GPIOB, GPIO_Pin_0, FALSE, PIN_LP_MCU_IN, PIN_LP_MCU_IN},
    {25, GPIOB, GPIO_Pin_1, FALSE, PIN_LP_MCU_IN, PIN_LP_MCU_IN},
    {26, GPIOB, GPIO_Pin_2, FALSE, PIN_LP_MCU_IN, PIN_LP_MCU_IN},

    {33, GPIOD, GPIO_Pin_4, FALSE, PIN_LP_MCU_IN, PIN_LP_MCU_IN},
    {34, GPIOD, GPIO_Pin_5, FALSE, PIN_LP_MCU_IN, PIN_LP_MCU_IN},
    {35, GPIOD, GPIO_Pin_6, FALSE, PIN_LP_MCU_IN, PIN_LP_MCU_IN},

    /* MCU output */
    {16, GPIOE, GPIO_Pin_2, FALSE, PIN_LP_MCU_OUT, PIN_LP_MCU_OUT},

    {32, GPIOF, GPIO_Pin_0, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},

    {36, GPIOD, GPIO_Pin_7, FALSE, PIN_LP_MCU_OUT, PIN_LP_MCU_OUT},
    {37, GPIOC, GPIO_Pin_0, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},

    /* SPI pins */
    {28, GPIOB, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {29, GPIOB, GPIO_Pin_5, FALSE, PIN_LP_MCU_OUT, PIN_LP_MCU_OUT},
    {30, GPIOB, GPIO_Pin_6, FALSE, PIN_LP_MCU_OUT, PIN_LP_MCU_OUT},
    {31, GPIOB, GPIO_Pin_7, FALSE, PIN_LP_MCU_IN, PIN_LP_MCU_IN},

    /* connected to UserSystem ------------------------------------------*/
    /* connected to 3.3V */
    {2, GPIOA, GPIO_Pin_1, FALSE, PIN_LP_VCC, PIN_LP_VCC}, /* MCU_RST */
    {47, GPIOE, GPIO_Pin_6, FALSE, PIN_LP_VCC, PIN_LP_VCC}, /* UART_TX */

    /* MCU output, SOCKET_CON1 */
    {5, GPIOA, GPIO_Pin_4, FALSE, PIN_LP_MCU_OUT, PIN_LP_MCU_OUT},

    /* MCU input, SOCKET_CON2 */
    {6, GPIOA, GPIO_Pin_5, FALSE, PIN_LP_MCU_IN, PIN_LP_MCU_IN},

    /* have not used, SOCKET_CON3 */
    {7, GPIOA, GPIO_Pin_6, FALSE, PIN_LP_NC, PIN_LP_NC},

    /* function I/O, UART_RX */
    {48, GPIOE, GPIO_Pin_7, FALSE, PIN_LP_MCU_IN, PIN_LP_MCU_IN},
#endif
};

/*
*********************************************************************************************************
*                                                                        FUNCTION PROTOTYPE
*********************************************************************************************************
*/


#if LED_CONF_ON
/*---------------------------------------------------------------------------------------------*/
void chip_LEDOff(void)
{
#if (iWL880A == CUR_PRODUCT)
    GPIO_ResetBits(LED_IOPORT, LED_PIN);
#else
    GPIO_SetBits(LED_IOPORT, LED_PIN);
#endif

    return;
}

/*---------------------------------------------------------------------------------------------*/
void chip_LEDOn(void)
{
#if (iWL880A == CUR_PRODUCT)
    GPIO_SetBits(LED_IOPORT, LED_PIN);
#else
    GPIO_ResetBits(LED_IOPORT, LED_PIN);
#endif

    return;
}

/*---------------------------------------------------------------------------------------------*/
void chip_LEDToggle(void)
{
    GPIO_ToggleBits(LED_IOPORT, LED_PIN);

    return;
}

/*---------------------------------------------------------------------------------------------*/
void chip_LED2Off(void)
{
#if (iWL880A == CUR_PRODUCT)
    GPIO_ResetBits(LED2_IOPORT, LED2_PIN);
#else
    GPIO_SetBits(LED2_IOPORT, LED2_PIN);
#endif

    return;
}

/*---------------------------------------------------------------------------------------------*/
void chip_LED2On(void)
{
#if (iWL880A == CUR_PRODUCT)
    GPIO_SetBits(LED2_IOPORT, LED2_PIN);
#else
    GPIO_ResetBits(LED2_IOPORT, LED2_PIN);
#endif

    return;
}

/*---------------------------------------------------------------------------------------------*/
void chip_LED2Toggle(void)
{
    GPIO_ToggleBits(LED2_IOPORT, LED2_PIN);

    return;
}
#else
void chip_LEDOff(void) {}
void chip_LEDOn(void) {}
void chip_LEDToggle(void) {}
void chip_LED2Off(void) {}
void chip_LED2On(void) {}
void chip_LED2Toggle(void) {}
#endif /*#if LED_CONF_ON*/

/*---------------------------------------------------------------------------------------------*/
DEV_ID chip_GetDevID(void)
{
    return s_tDevID;
}

/*---------------------------------------------------------------------------------------------*/
char* chip_GetSN(void)
{
    return &s_achSN[0];
}

/*---------------------------------------------------------------------------------------------*/
char* chip_GetID(void)
{
    return &s_achSN[GET_ST_FLD_OFFSET(UNIQUE_ID, wYCoordinate) * 2];
}

/**
  * @brief  Wait (1000ms=2*500ms) for LSE stabilization .
  * @param  None.
  * @retval None.
  */
static void WaitLSEStable(void)
{
    int8_t    chCnt;

    /* ATTENTION: MUST enable TIM4 CLK before access its registers. */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);

    TIM4_DeInit();

    /* TIM4 configuration:
         - TIM4CLK is set to 16MHz, the TIM4 Prescaler is equal to 32768 so the TIM4 counter
         clock used is 16MHz / 32768 = 488Hz
         - With 488Hz we can generate time base:
        max time base is 524ms if TIM4_PERIOD = 255 --> (255 + 1) / 488 = 524ms
        min time base is 4ms if TIM4_PERIOD = 1   --> (1 + 1) / 125000 = 4ms */
    TIM4_TimeBaseInit(TIM4_Prescaler_32768, 243);

    /* Clear update flag */
    TIM4_ClearFlag(TIM4_FLAG_Update);

    /* Enable TIM4 */
    TIM4_Cmd(ENABLE);

    /* Wait 1000ms=2*500ms */
    for (chCnt = 0; chCnt < 2; ++chCnt)
    {
        /* Wait until the flag of update is setted. */
        while (RESET == TIM4_GetFlagStatus(TIM4_FLAG_Update))
        {
            null();
        }
        TIM4_ClearFlag(TIM4_FLAG_Update);
    }

    /* Disable TIM4 */
    TIM4_Cmd(DISABLE);

    /* Disable clock of TIM4 for saving energy. */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);
}


/*
*********************************************************************************************************
*                                                                             Initialize Chip
* Description: Initialize Chip of STM8L151C8
* Arguments : None.
* Returns     : None
* Note(s)     : 
*********************************************************************************************************
*/
void chip_Init(void)
{
    INT16S    nCnt;
    INT8U    byOPT3;	
    INT8U    *p_byData;
    INT8U    byData;

    /* Set high speed internal clock prescaler=1(16MHz) */
    CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  
    /* Set all pins to mode of low power. */
    for (nCnt = 0; nCnt < SIZE_OF_ARRAY(s_astConfigGPIO); ++nCnt)
    {
        GPIO_Init( s_astConfigGPIO[nCnt].p_byPort,
                         s_astConfigGPIO[nCnt].byPin,
                         s_astConfigGPIO[nCnt].eModeLowPower );
    }

    /* Configure pins for LED: PD7(pin_36)<->LED1, PC7(pin_46)<->LED2 */
#if (iWL880A == CUR_PRODUCT)	
    GPIO_Init(LED_IOPORT, LED_PIN, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(LED2_IOPORT, LED2_PIN, GPIO_Mode_Out_PP_Low_Slow);
#else
    GPIO_Init(LED_IOPORT, LED_PIN, GPIO_Mode_Out_PP_High_Slow);
    GPIO_Init(LED2_IOPORT, LED2_PIN, GPIO_Mode_Out_PP_High_Slow);
#endif

    /* Twinkle the LED1 and LED2 to indicate booting */
    chip_LEDOn();
    chip_LED2On();

    /* Enable LSE(External Low Speed oscillator) clock for RTC */
    CLK_LSEConfig(CLK_LSE_ON); /* Enable LSE */
    nCnt = 0x7FFF; /* Wait for LSE clock to be ready */
    while ((RESET == CLK_GetFlagStatus(CLK_FLAG_LSERDY)) && (nCnt > 0))
    {
        --nCnt;
    }
    WaitLSEStable();

    /* Enable LSI(Internal Low Speed oscillator) clock for IWDG */
    CLK_LSICmd(ENABLE);
    nCnt = 0x7FFF; /* Wait for LSI clock to be ready */
    while ((RESET == CLK_GetFlagStatus(CLK_FLAG_LSIRDY)) && (nCnt > 0))
    {
        --nCnt;
    }

    chip_LEDOff();
    chip_LED2Off();

    /* Get unique ID. */
    p_byData = (uint8_t *)&s_stUniqueID;
    for (nCnt = 0; nCnt < sizeof(UNIQUE_ID); ++nCnt)
    {
        //这里的ID地址应该是0x4931?STM8L151C8数据手册P67
        byData = FLASH_ReadByte(UNIQUE_ID_END_ADDR - nCnt);
        *p_byData++ = byData;
        s_achSN[2 * nCnt + 0] = DATA_2_ASCII(GET_BYTE_HIGH4(byData));
        s_achSN[2 * nCnt + 1] = DATA_2_ASCII(GET_BYTE_LOW4(byData));
    }
    s_achSN[sizeof(s_achSN) - 1] = '\0';
    s_tDevID = MAKE_DWORD(s_stUniqueID.wYCoordinate, s_stUniqueID.wXCoordinate);

//设置独立看门狗在STM8L151处于休眠或者半休眠的时候停止运行
    /* Stop the IWDG in Halt/Active-halt mode.
     OPT3<1>=IWDG_HALT:
        0: Independent watchdog continues running in Halt/Activ-halt mode;
        1: Independent watchdog stopped in Halt/Active-halt mode. 
        关于OPT3字段的含义，参考STM8L151C8数据手册P65*/
    byOPT3 = FLASH_ReadByte(OPT3_ADDR);
    //等于0，证明IWDG在STM8L151处于休眠或者半休眠状态时仍然继续运行
    if (0 == (IWDG_HALT & byOPT3))
    {
        //下面的操作时取消IWDG在休眠状态下仍然运行，以节能
        /* Write MASS in the FLASH_DUKR to unlock the option bytes otherwise program would failed. */
        FLASH_Unlock(FLASH_MemType_Data);
        byOPT3 |= IWDG_HALT;
        FLASH_ProgramOptionByte(OPT3_ADDR, byOPT3);
        FLASH_Lock(FLASH_MemType_Data);
    }

    /* (1) Internal reference voltage stopped in Active-halt mode for saved energy.
        (2) Fast wakeup time is forced without waiting the start of the internal reference voltage.
        (3) The analog(BOR/PVD/ADC/LCD/Compactor) need wait the internal voltage reference
             is ready by check the Vrefintf flag in the PWR_CSR register for reliable used. */
    PWR_UltraLowPowerCmd(ENABLE);
    PWR_FastWakeUpCmd(ENABLE);
	
    return;
}

/**
  * @brief  Set pins of MCU_OUT/IN to lower power before enter HALT mode.
  * @note  This consume 443us(10000tims=4429ms) for switch mode of 4 GPIO.
  * @param  None.
  * @retval None.
  */
void chip_EnterLowPwr(void)
{
    int8_t    chCnt;

    if (!s_bIsLowPwr)
    {
        s_bIsLowPwr = TRUE;

        /* Turn off the TCXO of RF. */
        SX1278EnterLowPower();

        /* Switch to low power mode for those pins that needs low power. */
        for (chCnt = 0; chCnt < SIZE_OF_ARRAY(s_astConfigGPIO); ++chCnt)
        {
            if (s_astConfigGPIO[chCnt].bNeedSWMode)
            {
                GPIO_Init( s_astConfigGPIO[chCnt].p_byPort,
                                 s_astConfigGPIO[chCnt].byPin,
                                 s_astConfigGPIO[chCnt].eModeLowPower );
            }
        }/*for*/
    }

    return;
}

/**
  * @brief  Set pins of MCU_OUT/IN from lower power after exit HALT mode.
  * @note  This consume 441us(10000tims=4408ms) for switch mode of 4 GPIO.
  * @param  None.
  * @retval None.
  */
void chip_ExitLowPwr(void)
{
    int8_t    chCnt;

    if (s_bIsLowPwr)
    {
        s_bIsLowPwr = FALSE;

        /* Switch to running mode for those pins that enter low power. */
        for (chCnt = 0; chCnt < SIZE_OF_ARRAY(s_astConfigGPIO); ++chCnt)
        {
            if (s_astConfigGPIO[chCnt].bNeedSWMode)
            {
                GPIO_Init( s_astConfigGPIO[chCnt].p_byPort,
                                 s_astConfigGPIO[chCnt].byPin,
                                 s_astConfigGPIO[chCnt].eModeMCURun );
            }
        }/*for*/

        /* Turn on the TCXO of RF. */
        SX1278ExitLowPower();
    }

    return;
}


/*--------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/


