/**
  ******************************************************************************
  * @file    Project/STM8L15x_StdPeriph_Template/stm8l15x_it.c
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    28-June-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all peripherals interrupt service routine.
  ******************************************************************************
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x_it.h"
#include "stm8l15x_rtc.h"
#include "CommPC.h"
#include "DebugPort.h"
#include "sx1278_src.h"
#include "main.h"
#include "Chip.h"
#include "Network.h"


/**
 * Handler for system tick
 * This function wound been invoked by ISR of tick timer
 */
extern void SysTick_handler(void);

/**
 * Handle interrupt of real time timer.
 * This function was invoked when timer of real time is expired.
 */
extern void rtimer_irq_handler(void);

extern void SX1278Dio0IrqHandler(void);
extern void tim4uart_ISR(void);
#if EXTI_HALT_UART
extern void exti4uart_ISR(void);
#endif


/** @addtogroup STM8L15x_StdPeriph_Template
  * @{
  */
	
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

#ifdef _COSMIC_
/**
  * @brief Dummy interrupt routine
  * @par Parameters:
  * None
  * @retval 
  * None
*/
INTERRUPT_HANDLER(NonHandledInterrupt,0)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#endif

/**
  * @brief TRAP interrupt routine
  * @par Parameters:
  * None
  * @retval 
  * None
*/
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief FLASH Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(FLASH_IRQHandler,1)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief DMA1 channel0 and channel1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(DMA1_CHANNEL0_1_IRQHandler,2)
{
#if (defined CPC_PORT_IS_USART1 || defined CPC_PORT_IS_USART3)
    if (DMA_GetITStatus(DMA1_IT_TC1))
#else
    if (DMA_GetITStatus(DMA1_IT_TC0))
#endif
    {
        cpc_DMATxIRQHandler();
    }

#if (2 == PRINT_WAY)
#if (defined DP_PORT_IS_USART1 || defined DP_PORT_IS_USART3)
    if (DMA_GetITStatus(DMA1_IT_TC1))
#else
    if (DMA_GetITStatus(DMA1_IT_TC0))
#endif
    {
        dp_DMATxIRQHandler();
    }
#endif/*#if (2 == PRINT_WAY)*/

    return;
}
/**
  * @brief DMA1 channel2 and channel3 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(DMA1_CHANNEL2_3_IRQHandler,3)
{
}
/**
  * @brief RTC / CSS_LSE Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(RTC_CSSLSE_IRQHandler,4)
{
    RTC_ClearITPendingBit(RTC_IT_ALRA);
    network_AlarmByRTC();
}
/**
  * @brief External IT PORTE/F and PVD Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTIE_F_PVD_IRQHandler,5)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PORTB / PORTG Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTIB_G_IRQHandler,6)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PORTD /PORTH Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTID_H_IRQHandler,7)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN0 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI0_IRQHandler,8)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI1_IRQHandler,9)
{
#if (iWL880A == CUR_PRODUCT)
    EXTI_ClearITPendingBit(EXTI_IT_Pin1);
    SX1278Dio0IrqHandler();
#endif	
}

/**
  * @brief External IT PIN2 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI2_IRQHandler,10)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN3 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI3_IRQHandler,11)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN4 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI4_IRQHandler,12)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN5 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI5_IRQHandler,13)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
#if EXTI_HALT_UART
    EXTI_ClearITPendingBit(EXTI_IT_Pin5);
    exti4uart_ISR();
#endif
}

/**
  * @brief External IT PIN6 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI6_IRQHandler,14)
{
#if ((iWL881A == CUR_PRODUCT) || (iWL882A == CUR_PRODUCT) || (iWL883A == CUR_PRODUCT))
    EXTI_ClearITPendingBit(EXTI_IT_Pin6);
    SX1278Dio0IrqHandler();
#endif	
}

/**
  * @brief External IT PIN7 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI7_IRQHandler,15)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief LCD /AES Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(LCD_AES_IRQHandler,16)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief CLK switch/CSS/TIM1 break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SWITCH_CSS_BREAK_DAC_IRQHandler,17)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief ADC1/Comparator Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(ADC1_COMP_IRQHandler,18)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief TIM2 Update/Overflow/Trigger/Break /USART2 TX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler,19)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief Timer2 Capture/Compare / USART2 RX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM2_CC_USART2_RX_IRQHandler,20)
{
#if (2 == PRINT_WAY)
#ifdef DP_PORT_IS_USART2
    /* EXPLAIN: Interrupt flag of "RXNE" is cleared by a read to the USART_DR register */
    dp_RxIRQHandler();
#endif
#endif/*#if (2 == PRINT_WAY)*/

#ifdef CPC_PORT_IS_USART2
    /* EXPLAIN: Interrupt flag of "RXNE" is cleared by a read to the USART_DR register */
    cpc_RxIRQHandler();
#endif
}


/**
  * @brief Timer3 Update/Overflow/Trigger/Break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQHandler,21)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief Timer3 Capture/Compare /USART3 RX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM3_CC_USART3_RX_IRQHandler,22)
{
#if (2 == PRINT_WAY)
#ifdef DP_PORT_IS_USART3
    /* EXPLAIN: Interrupt flag of "RXNE" is cleared by a read to the USART_DR register */
    dp_RxIRQHandler();
#endif
#endif/*#if (2 == PRINT_WAY)*/

#ifdef CPC_PORT_IS_USART3
    cpc_RxIRQHandler();
#endif
}
/**
  * @brief TIM1 Update/Overflow/Trigger/Commutation Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler,23)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief TIM1 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_CC_IRQHandler,24)
{
    rtimer_irq_handler();
}

/**
  * @brief TIM4 Update/Overflow/Trigger Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM4_UPD_OVF_TRG_IRQHandler,25)
{
    tim4uart_ISR();
}
/**
  * @brief SPI1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SPI1_IRQHandler,26)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */		
}

/**
  * @brief USART1 TX / TIM5 Update/Overflow/Trigger/Break Interrupt  routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler,27)
{
    SysTick_handler();
}

/**
  * @brief USART1 RX / Timer5 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler,28)
{
#if (2 == PRINT_WAY)
#ifdef DP_PORT_IS_USART1
    /* EXPLAIN: Interrupt flag of "RXNE" is cleared by a read to the USART_DR register */
    dp_RxIRQHandler();
#endif
#endif/*#if (2 == PRINT_WAY)*/

#ifdef CPC_PORT_IS_USART1
    /* EXPLAIN: Interrupt flag of "RXNE" is cleared by a read to the USART_DR register */
    cpc_RxIRQHandler();
#endif
}

/**
  * @brief I2C1 / SPI2 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(I2C1_SPI2_IRQHandler,29)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
