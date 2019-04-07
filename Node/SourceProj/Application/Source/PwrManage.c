/**
 * \file
 *         PwrManage.h
 * \description
 *         Power manage of MCU
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-11-30 14:44
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "clock.h"
#include "Dbg.h"
#include "PwrManage.h"
#include "Chip.h"
#include "Wdg.h"


/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/**
* @brief  Whether the process agree to HALT the MCU.
*/
static PWR_AGR_HALT    s_tPwrAgrHalt;

/**
* @brief  Whether enters the HALT mode.
*/
static volatile bool    s_bEnterHalt = FALSE;


/* Private function prototypes -----------------------------------------------*/

/* Private Constants ---------------------------------------------------------*/


/**
  * @brief  One process agrees enter into HALT mode.
  * @param  PWR_ID_Type eID: ID of this power managed object.
  * @retval  None.
  */
void pm_AgrHalt(PWR_ID_Type eID)
{
    ASSERT(PWR_ID_INVALID_MIN < eID && eID < PWR_ID_INVALID_MAX);

    if (PWR_ID_INVALID_MIN < eID && eID < PWR_ID_INVALID_MAX)
    {
        s_tPwrAgrHalt &= ~((PWR_AGR_HALT)1 << (eID - PWR_ID_INVALID_MIN - 1)); /* Bit start from 0 */
    }
}

/**
  * @brief  One process opposes enter into HALT mode.
  * @param  PWR_ID_Type eID: ID of this power managed object.
  * @retval  None.
  */
void pm_OpposeHalt(PWR_ID_Type eID)
{
    ASSERT(PWR_ID_INVALID_MIN < eID && eID < PWR_ID_INVALID_MAX);

    if (PWR_ID_INVALID_MIN < eID && eID < PWR_ID_INVALID_MAX)
    {
        s_tPwrAgrHalt |= ((PWR_AGR_HALT)1 << (eID - PWR_ID_INVALID_MIN - 1)); /* Bit start from 0 */
    }
}

/**
  * @brief  Enter low power mode: HALT or WFI.
  * @param  None.
  * @retval  None.
  */
void pm_EnterLowPwr(void)
{
    if (0 == s_tPwrAgrHalt) /* All process agree to HALT */
    {
        s_bEnterHalt = TRUE;
        chip_EnterLowPwr();
        clock_stop(); /* Stop clock && clear its INT otherwise the MCU can NOT enter HALT. */
        wdg_Feed(); /* Feed the IWDG before HALT to avoid unexpected IWDG reset. */
        halt();
    }
    else
    {
        wfi();
    }

    return;
}

/**
  * @brief  Exit low power mode.
  * @param  None.
  * @retval  None.
  */
void pm_ExitLowPwr(void)
{
    if (s_bEnterHalt) /* Wake up from mode of HALT */
    {
        s_bEnterHalt = FALSE;
        clock_start(); /* Start clock for etimer_process. */
        chip_ExitLowPwr();
    }

    return;
}

/**
  * @brief  Initialize power manage.
  * @param  None
  * @retval  None
  */
void pm_Init(void)
{
    ASSERT(PWR_MANAGE_NUM == (PWR_ID_INVALID_MAX - 1));

    /* Set the default value: all process do NOT agreed to HALT the MCU. */
    s_tPwrAgrHalt = ((PWR_AGR_HALT)1 << PWR_MANAGE_NUM) - 1;

    return;
}


#if 0
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void pm_Test(void)
{
    pm_Init();

    s_tPwrAgrHalt = 0;

    pm_OpposeHalt(2);
    pm_AgrHalt(2);
    pm_EnterLowPwr();
	
    return;
}
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#endif

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

