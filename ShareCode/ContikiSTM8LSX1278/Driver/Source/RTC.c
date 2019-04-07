/*
************************************************************************************************
* Filename   	: RTC.c
* Programmer : JiangJun
* Description	: Process RTC of STM8L151C8 
* Date           : 2014-09-16
************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <limits.h>
#include "Dbg.h"
#include "RTC.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_rtc.h"


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
	

/*
*********************************************************************************************************
*                                            				     DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/
#define MIN_WU_TIME    1
#define MAX_WU_TIME    31 /* 31 = 65535 / (32768 / 16) */

/* Convert seconds to wake up counter. */
#define SEC_2_WU_CNT(sec)    (32768ul / 16 * (sec)) /* the MAX=31s */

/* Insured: (AsynchPrediv + 1)(SynchPrediv + 1)=32768 */
#define ASYNCH_PREDIV    31
#define SYNCH_PREDIV    1023

#define CONVERT_SS_2_MS(SSReg)    \
    ((SYNCH_PREDIV - (int16_t)SSReg) * (int32_t)1000 / (SYNCH_PREDIV + 1))


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
static int32_t GetTimeMsExt(uint16_t *p_wSSReg, int32_t *p_lSecInReg);


/*
************************************************************************************************
*			             	                  			            Initialize RTC
* Description : Initialize RTC module as well as start running.  
* Arguments  : void 
* Returns      : void
* Notes        : 
************************************************************************************************
*/
void rtc_Init(void)
{
    RTC_InitTypeDef    l_stInit;
    RTC_TimeTypeDef   l_stTime;
    RTC_DateTypeDef   l_stDate;

    /* Select LSE(32.768 KHz) as RTC clock source */
    CLK_RTCClockConfig(CLK_RTCCLKSource_LSE, CLK_RTCCLKDiv_1);
    CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);

    /* Deinitializes the RTC registers to their default reset values. */
    RTC_DeInit();

    /* Set RTC to a specific value */
    //设置RTC的计数频率为1Hz（32768/32/1024=1s)
    l_stInit.RTC_HourFormat = RTC_HourFormat_24;
    l_stInit.RTC_AsynchPrediv = ASYNCH_PREDIV; 
    l_stInit.RTC_SynchPrediv = SYNCH_PREDIV;
    RTC_Init(&l_stInit);

    /* Enable bypass shadow register: read calendar from RTC register directly. */
    RTC_BypassShadowCmd(ENABLE);

    RTC_DateStructInit(&l_stDate);
    l_stDate.RTC_WeekDay = RTC_Weekday_Friday;
    l_stDate.RTC_Date = 25;
    l_stDate.RTC_Month = RTC_Month_October;
    l_stDate.RTC_Year = 15;
    RTC_SetDate(RTC_Format_BIN, &l_stDate);

    RTC_TimeStructInit(&l_stTime);
    l_stTime.RTC_Hours   = 0;
    l_stTime.RTC_Minutes = 0;
    l_stTime.RTC_Seconds = 0;
    RTC_SetTime(RTC_Format_BIN, &l_stTime);

    return;
}

/*
************************************************************************************************
*			             	                  			    Enable WakeUp of RTC
* Description : Enable Wake Up of RTC.   
* Arguments  : uint8_t bySec    interval of wake up, the unit is second.
* Returns      : int8_t    0=OK, -1=bad interval.
* Notes        : 
************************************************************************************************
*/
int8_t rtc_EnWakeUp(uint8_t bySec)
{
    ASSERT(MIN_WU_TIME <= bySec && bySec <= MAX_WU_TIME);

    if (!(MIN_WU_TIME <= bySec && bySec <= MAX_WU_TIME))
    {
        return -1;
    }

    /* Wake up CPU per "bySec" seconds by RTC. */
    RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
    RTC_SetWakeUpCounter(SEC_2_WU_CNT(bySec));

    /* Enable INT of RTC wake up. */
    RTC_ClearITPendingBit(RTC_IT_WUT);
    RTC_ITConfig(RTC_IT_WUT, ENABLE);
    RTC_WakeUpCmd(ENABLE);

    return 0;
}

/*
************************************************************************************************
*			             	                  			    Disable WakeUp of RTC
* Description : Disable Wake Up of RTC.   
* Arguments  : None 
* Returns      : None
* Notes        : 
************************************************************************************************
*/
void rtc_DisWakeUp(void)
{
    RTC_WakeUpCmd(DISABLE);

    RTC_ClearITPendingBit(RTC_IT_WUT);
    RTC_ITConfig(RTC_IT_WUT, DISABLE);

    return;
}

/*
************************************************************************************************
*			             	                  			          Set Wake Up Time
* Description : Set wake up time to RTC
* Arguments  : uint8_t bySec    interval of wake up, the unit is second.
* Returns      : void
* Notes        : 
************************************************************************************************
*/
int8_t rtc_SetWakeUp(uint8_t bySec)
{
    ASSERT(MIN_WU_TIME <= bySec && bySec <= MAX_WU_TIME);

    if (!(MIN_WU_TIME <= bySec && bySec <= MAX_WU_TIME))
    {
        return -1;
    }

    RTC_WakeUpCmd(DISABLE); /* Before set wake up counter MUST disable this unit. */
    RTC_SetWakeUpCounter(SEC_2_WU_CNT(bySec));
    RTC_WakeUpCmd(ENABLE);

    return 0;
}

/*
************************************************************************************************
*			             	                  			    Enable Alarm of RTC
* Description : Enable Alarm of RTC.
* Arguments  : uint8_t bySec    interval of wake up, the unit is second.
* Returns      : int8_t    0=OK, -1=bad interval.
* Notes        : 
************************************************************************************************
*/
int8_t rtc_EnAlarm(void)
{
    /* Enable INT of RTC alarm. */
    RTC_ClearITPendingBit(RTC_IT_ALRA);
    RTC_ITConfig(RTC_IT_ALRA, ENABLE);

    return 0;
}

/*
************************************************************************************************
*			             	                  			    Disable Alarm of RTC
* Description : Disable Alarm of RTC.   
* Arguments  : None 
* Returns      : None
* Notes        : 
************************************************************************************************
*/
void rtc_DisAlarm(void)
{
    RTC_AlarmCmd(DISABLE);

    RTC_ClearITPendingBit(RTC_IT_ALRA);
    RTC_ITConfig(RTC_IT_ALRA, DISABLE);

    return;
}

/*
************************************************************************************************
*			             	                  			          Set Alarm Time
* Description : Set Alarm time to RTC
* Arguments  : int32_t lAlarmMs    milliseconds of alarm time start from 00:00:00.000.
* Returns      : 0=OK, -1=Bad quantity, -2=Error of AlarmCmd(DISABLE),
*                    -3=Error of AlarmCmd(ENABLE), -4=Error of RTC_AlarmSubSecondConfig().
* Notes        : MUST call this by atomically(Disable and Enable INT) otherwise incurred alarm error.
************************************************************************************************
*/
int8_t rtc_SetAlarm(int32_t lAlarmMs)
{
    #define MUL_PREDIV_TH    (LONG_MAX / (SYNCH_PREDIV + 1))

    int8_t    chResult;
    int32_t    lSubPer, lCurMs, lSecInReg, lAlarmSec, lDiff, lAddSec;
    int16_t    nSubReg, nAlarmSub;
    RTC_AlarmTypeDef    l_stAlarm;

    ASSERT(lAlarmMs < MAX_MS_OF_DAY);

    if (MAX_MS_OF_DAY <= lAlarmMs)
    {
        return -1; /* Bad quantity */
    }

    /* Get millisecond of RTC. */
    lCurMs = GetTimeMsExt((uint16_t *)&nSubReg, &lSecInReg);

    /* Get difference between RTC-Time and Alarm-Time. */
    lSubPer = lAlarmMs - lCurMs;
    if (lSubPer < 0)
    {
        lSubPer += MAX_MS_OF_DAY; /* Wrap on 00:00:00.000 */
    }

    /* Covert millisecond to subsecond count. */
    if (lSubPer < MUL_PREDIV_TH) /* Prevent OVERFLOW! */
    {
        lSubPer = lSubPer * (1 + SYNCH_PREDIV) / 1000;
    }
    else
    {
        lSubPer = (int32_t)((float)lSubPer * (float)(1 + SYNCH_PREDIV) / 1000.0);
    }

    if (lSubPer <= nSubReg) /* Short Alarm, ONLY change AlarmSub. */
    {
        lAlarmSec = lSecInReg;
        nAlarmSub = nSubReg - lSubPer;
    }
    else /* Long Alarm, change AlarmSec and AlarmSub. */
    {
        lDiff = lSubPer - nSubReg;
        lAddSec = (lDiff + SYNCH_PREDIV) / (1 + SYNCH_PREDIV); 
        nAlarmSub = (1 + SYNCH_PREDIV) * lAddSec - lDiff;
        lAlarmSec = lSecInReg + lAddSec;
        if (MAX_SEC_OF_DAY <= lAlarmSec)
        {
            lAlarmSec -= MAX_SEC_OF_DAY; /* Wrap on 00:00:00 */
        }
    }
	
    RTC_AlarmStructInit(&l_stAlarm);
    l_stAlarm.RTC_AlarmTime.RTC_Hours = SEC_2_HH(lAlarmSec);
    l_stAlarm.RTC_AlarmTime.RTC_Minutes = SEC_2_MM(lAlarmSec);
    l_stAlarm.RTC_AlarmTime.RTC_Seconds = SEC_2_SS(lAlarmSec);
    l_stAlarm.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;

    /* Before configuring the Alarm sttings MUST disable this unit. */
    chResult = 0;
    if (ERROR == RTC_AlarmCmd(DISABLE))
    {
        chResult = -2; /* Error of AlarmCmd(DISABLE) */
    }
    RTC_SetAlarm(RTC_Format_BIN, &l_stAlarm);
    if (ERROR == RTC_AlarmCmd(ENABLE))
    {
        chResult = -3; /* Error of AlarmCmd(ENABLE) */
    }
	
    if (ERROR == RTC_AlarmSubSecondConfig(nAlarmSub,RTC_AlarmSubSecondMask_None))
    {
        chResult = -4; /* Error of RTC_AlarmSubSecondConfig() */
    }

    return chResult;
}

/*
************************************************************************************************
*			             	                  			 Set Time for RTC by Second
* Description : Set time for RTC by second  
* Arguments  : uint32_t uSec    seconds of RTC 
* Returns      : 0=OK, -1=Bad quantity, -2=Set Time failed.
* Notes        : 
************************************************************************************************
*/
int8_t rtc_SetTimeSec(uint32_t uSec)
{
    RTC_TimeTypeDef   l_stTime;

    ASSERT(uSec < MAX_SEC_OF_DAY);

    /* Check validity of parameter. */
    if (MAX_SEC_OF_DAY <= uSec)
    {
        return -1; /* Bad Quantity */
    }

    /* Convert second to RTC_Time. */
    RTC_TimeStructInit(&l_stTime);
    l_stTime.RTC_Hours   = SEC_2_HH(uSec);
    l_stTime.RTC_Minutes = SEC_2_MM(uSec);
    l_stTime.RTC_Seconds = SEC_2_SS(uSec);
	
    if (SUCCESS == RTC_SetTime(RTC_Format_BIN, &l_stTime))
    {
        return 0; /* OK */
    }
    else
    {
        return -2; /* Set Time failed */
    }
}

/*
************************************************************************************************
*                                Add or Subtract x Ms for RTC
* Description : Add or Subtract x millisecond for RTC
* Arguments  : int16_t nMs    +100: positive shift 100ms; -100: negative shift 100ms.
* Returns      : 0=OK, -1=Bad quantity, -2=Set shift failed, -3=Shift timeout.
* Notes        : Assume the RTC=03:25:32.500,
*                   to 03:25:32.600: performs a positive shift of 100ms, call "rtc_SetMs(100)";
*                   to 03:25:32.400: performs a negative shift of 100ms, call "rtc_SetMs(-100)".
************************************************************************************************
*/
int8_t rtc_SetMs(int16_t nMs)
{
    int16_t    nWaitCnt;
    uint16_t    wShiftSubFS;

    ASSERT(abs(nMs) < 1000);

     /* Check validity of parameter. */
    if ((0 == nMs) || (1000 <= abs(nMs)))
    {
        return -1;
    }
	
    if (0 < nMs) /*Add x ms for current RTC time*/
    {
        /* Convert millisecond to the value,which will be writing into SUBFS[14:0] in RTC_SHIFTRx register */
        wShiftSubFS = (uint32_t)(1000 - nMs) * (SYNCH_PREDIV + 1) / 1000;
        if (ERROR == RTC_SynchroShiftConfig(RTC_ShiftAdd1S_Set, wShiftSubFS))
        {
            return -2; /* Set Shift failed */
        }
    }
    else /*Subtract x ms for current RTC time*/
    {
        /* Convert millisecond to the value,which will be writing into SUBFS[14:0] in RTC_SHIFTRx register */
        wShiftSubFS =  (uint32_t)(-1 * nMs) * (SYNCH_PREDIV + 1) / 1000;
        if (ERROR == RTC_SynchroShiftConfig(RTC_ShiftAdd1S_Reset, wShiftSubFS))
        {
            return -2; /* Set Shift failed */
        }  
    }

    /* Waiting until the shift operation is finished. */
    nWaitCnt = 0x7FFF;
    while ((SET == RTC_GetFlagStatus(RTC_FLAG_SHPF)) && (0 < nWaitCnt))
    {
        --nWaitCnt;
    }

    if (0 < nWaitCnt)
    {
        return 0; /* OK */
    }
    else
    {
        return -3; /* Shift operation have not completed */
    }
}

/*
************************************************************************************************
*			             	                  			      Get Time from RTC
* Description : Get time from RTC   
* Arguments  : uint16_t *p_wSSReg =NULL
*                                                 =point to variable that saved [SSRH:SSRL] by this pointer.
*                    int32_t *p_lSecInReg = NULL
*                                                   = point to variable that saved seconds in register of [SEC:MIN:HOUR].
* Returns      : int32_t    quntity of current milli-second.
* Notes        :  the "SecInReg" maybe NOT equal second of the true RTC!
************************************************************************************************
*/
static int32_t GetTimeMsExt(uint16_t *p_wSSReg, int32_t *p_lSecInReg)
{
    uint8_t    bySec, byMin, byHour;
    int16_t    nMs;
    uint16_t    wSSReg;
    int32_t    lSec, lMs;

    uint8_t    bySSRH, bySSRL, byTR1, byTR2, byTR3;
    uint8_t    bySSRHAgain, bySSRLAgain, byTR1Again, byTR2Again, byTR3Again;

    /* EXPLAIN: read the registers twice and confirm insured correct and coherent. */
    do
    {
        /* Read registers firstly. */
        bySSRH = RTC->SSRH;
        bySSRL = RTC->SSRL;
        byTR1 = RTC->TR1;
        byTR2 = RTC->TR2;
        byTR3 = RTC->TR3;

        /* Read registers secondly. */
        bySSRHAgain = RTC->SSRH;
        bySSRLAgain = RTC->SSRL;
        byTR1Again = RTC->TR1;
        byTR2Again = RTC->TR2;
        byTR3Again = RTC->TR3;
    } while (!( bySSRH == bySSRHAgain && bySSRL == bySSRLAgain &&
                    byTR1 == byTR1Again && byTR2 == byTR2Again && byTR3 == byTR3Again ));

    /* Get seconds */
    bySec = BCD_2_BYTE(byTR1);
    byMin = BCD_2_BYTE(byTR2);
    byTR3 &= ~RTC_TR3_PM;
    byHour = BCD_2_BYTE(byTR3);
    lSec = HHMMSS_2SEC(byHour, byMin, bySec);
    if (p_lSecInReg)
    {
        *p_lSecInReg = lSec;
    }

    /* Get milli-seconds */
    wSSReg = MAKE_WORD(bySSRH, bySSRL);
    nMs = CONVERT_SS_2_MS(wSSReg);

    if (p_wSSReg)
    {
        *p_wSSReg = wSSReg;
    }

    lMs = 1000 * lSec + nMs;

    return lMs;
}

/*
************************************************************************************************
*			             	                  			      Get Millisecond from RTC
* Description : get millisecond from RTC.
* Arguments  : none
* Returns      : int32_t    quntity of current millisecond.
* Notes        : error of calculation is +-3ms.
************************************************************************************************
*/
int32_t rtc_GetTimeMs(void)
{
    return GetTimeMsExt(NULL, NULL);
}


/*--------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/

