/*
************************************************************************************************
* Filename   	: RTC.h
* Programmer : ZJU_CEE
* Description	: deal with all operation of RTC 
* Date           : 2014-05-07
************************************************************************************************
*/

#ifndef __RTC_H__
#define __RTC_H__


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
/* Convert HH:MM:SS to seconds, as: 16:01:52 to 57712s */
#define HHMMSS_2SEC(HH, MM, SS)    ((HH) * 3600ul + (MM) * 60 + (SS))

/* Convert seconds to HH:MM:SS, as: 57712s to 16:01:52 */
#define SEC_2_HH(sec)    ((sec) / 3600)
#define SEC_2_MM(sec)    ((sec) % 3600 / 60) 
#define SEC_2_SS(sec)    ((sec) % 60)

/* The maximum seconds and milliseconds of one day. */
#define MAX_SEC_OF_DAY    ((int32_t)60 * 60 * 24)
#define MAX_MS_OF_DAY    ((int32_t)1000 * MAX_SEC_OF_DAY)

/* Calculate RTC period between start time and end time. */
#define CALC_RTC_PERIOD(start, end)    \
    (((start) <= (end)) ? ((end) - (start)) : ((end) + MAX_MS_OF_DAY - (start)))


/*
*********************************************************************************************************
*                                            				     DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/
typedef struct _rtc_time
{
    INT8U    bySec; /* Seconde:[0, 59] */
    INT8U    byMin; /* Minute:[0, 59] */
    INT8U    byHour; /* Hours:[0, 23] */	
} RTC_TIME;


/*
*********************************************************************************************************
*                                                                  GLOBAL VARIABLE & STRUCTURE
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                        FUNCTION PROTOTYPE
*********************************************************************************************************
*/
void rtc_Init(void);
int8_t rtc_EnWakeUp(uint8_t bySec);
void rtc_DisWakeUp(void);
int8_t rtc_SetWakeUp(uint8_t bySec);
int8_t rtc_EnAlarm(void);
void rtc_DisAlarm(void);
int8_t rtc_SetAlarm(int32_t lAlarmMs);
int8_t rtc_SetTimeSec(uint32_t uSec);
int8_t rtc_SetMs(int16_t nMs);
int32_t rtc_GetTimeMs(void);


#endif    /* __RTC_H__ */

