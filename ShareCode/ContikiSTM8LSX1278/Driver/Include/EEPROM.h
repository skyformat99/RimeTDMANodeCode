/*
************************************************************************************************
* Filename   	: EEPROM.h
* Programmer : JiangJun
* Description	: operate EEPROM which on chip of STM8L
* Note           : 2015-02-06
************************************************************************************************
*/

#ifndef __EEPROM_H__
#define __EEPROM_H__


/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
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
#define E2_TOT_SIZE    (2 * 1024) /* 2KB */
#define E2_ADDR_START    0x1000
#define E2_ADDR_END    (E2_ADDR_START + E2_TOT_SIZE - 1)


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
extern void e2_Init(void);
extern void e2_Wr(const void *p_vSrc, INT16S nSize, INT16U wAddr);
extern void e2_Rd(void *p_vDst, INT16S nSize, INT16U wAddr);
extern void e2_Fill(INT8U byData, INT16S nSize, INT16U wAddr);


#endif    /* __EEPROM_H__ */


