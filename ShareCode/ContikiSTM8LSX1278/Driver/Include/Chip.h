/*
************************************************************************************************
* Filename   	: Chip.h
* Programmer : JiangJun
* Description	: Process clock/pins of STM8L151C8 
* Date           : 2014-08-26
************************************************************************************************
*/

#ifndef __CHIP_H__
#define __CHIP_H__


/*
*********************************************************************************************************
*                                                                        INCLUDE FILES
*********************************************************************************************************
*/
#include <stdint.h>


/*
*********************************************************************************************************
*                                                                         DEBUG SYSTEM
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                       MACROS & CONSTANTS
*********************************************************************************************************
*/
/* EXPLAIN: set those to here that called by "CommPC.c" and "DebugPort.c" */
#define AUX_PORT    GPIOA
#define AUX_PIN    GPIO_Pin_4 /* AUX=PA4 */

/* Set CON1=AUX=PA4 to OUTPUT */
#define INIT_AUX()    GPIO_Init(AUX_PORT, AUX_PIN, GPIO_Mode_Out_PP_High_Fast)
#define SET_AUX_LOW()    GPIO_ResetBits(AUX_PORT, AUX_PIN)
#define SET_AUX_HIGH()    GPIO_SetBits(AUX_PORT, AUX_PIN)


/*
*********************************************************************************************************
*                                            				     DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/
/* Device unique ID */
typedef uint32_t    DEV_ID;


/*
*********************************************************************************************************
*                                            				         GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                        FUNCTION PROTOTYPE
*********************************************************************************************************
*/
extern void chip_Init(void);
extern DEV_ID chip_GetDevID(void);
extern void chip_LEDOn(void);
extern void chip_LEDOff(void);
extern void chip_LEDToggle(void);
extern void chip_LED2Off(void);
extern void chip_LED2On(void);
extern void chip_LED2Toggle(void);
extern void chip_EnterLowPwr(void);
extern void chip_ExitLowPwr(void);
extern char* chip_GetSN(void);
extern char* chip_GetID(void);


#endif    /* __CHIP_H__ */

