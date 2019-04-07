/*
************************************************************************************************
* Filename   	: main.h
* Programmer : JiangJun
* Description	: control system version
* Date           : 2014-08-22
************************************************************************************************
*/
#ifndef __MAIN_H__
#define __MAIN_H__


/*
*********************************************************************************************************
*                                                                         DEBUG SYSTEM
*********************************************************************************************************
*/
/*  Product      RF Switch     Fxosc              Fstep=Fxosc/(2^19)
    iWL880A     PE4259         32MHz XTAL        61.03515625
    iWL881A     TXE & RXE    26MHz TCXO        49.59106445
    iWL882A     PE4259        26MHz TCXO        49.59106445
    iWL883A     PE4259        32MHz TCXO        61.03515625
*/
#define iWL880A    1
#define iWL881A    2
#define iWL882A    3
#define iWL883A    4
#define CUR_PRODUCT    iWL883A

/*
    Mode    Function     Debug-Halt  Support-Shell  Low-Power  ASSERT    UART-Port
    debug   for test     enable       enable        disable    enable    connect to PC
    release for product  disable      disable       enable     disable   connect to user system
*/
#define REL_VER    1 /* 0=debug; 1=release */

/* Catch the context of Network-Process been blocked. */
#define CATCH_NET_BLOCK    0 /* 0=release; 1=catch version. */

/* UPLINK=Only support uplink; UPLINK_WAKE=Support uplink and wake */
#define UPLINK    1
#define UPLINK_WAKE    2
#define CUR_SYST    UPLINK_WAKE

/* current version */
#if (UPLINK == CUR_SYST)
#define CUR_VER    "RNDU470T V1.0.22 16-08-18"
#else
#define CUR_VER    "RNDU470TWF V2.0.20 17-08-25"
#endif

/* 
    Whether the UART needs EXTI to exit HALT: 
    0=do NOT, sleep is WFI; 
    1=needs, sleep=HALT. 
*/
#define EXTI_HALT_UART    0

/* Add special operation for some company. */
#define COMPANY_PUBLIC    1
#define COMPANY_ShuWei    2
#define COMPANY_JiKang    3
#define CUR_COMPANY    COMPANY_PUBLIC


/*
*********************************************************************************************************
*                                                                      	MACRO DEFINITION
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/


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


#endif    /* __MAIN_H__ */

