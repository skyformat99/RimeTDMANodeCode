/**
 * \file
 *         PwrManage.h
 * \description
 *         Power manage of MCU
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-11-30 14:21
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */
 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWR_MANAGE_H__
#define __PWR_MANAGE_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "Util.h"

/* Exported variables ------------------------------------------------------- */

/* Exported types ------------------------------------------------------------*/
/**
* @brief  Add ID of power manage on here!
*/
typedef enum _pwr_id
{
    PWR_ID_INVALID_MIN = (uint8_t)0,
    PWR_ID_NETWORK,
    PWR_ID_COMM2TRM,
    PWR_ID_INVALID_MAX
} PWR_ID_Type;

/**
* @brief  Set number of power manage on here!
*/
#define PWR_MANAGE_NUM    2

/**
* @brief  Whether the process agree to HALT the MCU.
*/
#if (PWR_MANAGE_NUM <= 8)
    typedef uint8_t    PWR_AGR_HALT;
#elif (PWR_MANAGE_NUM <= 16)
    typedef uint16_t    PWR_AGR_HALT;
#elif (PWR_MANAGE_NUM <= 32)
    typedef uint32_t    PWR_AGR_HALT;
#else
    #error  "PwrManage.h: proce_id can't great than 32!"
#endif


/* Private types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Imported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void pm_Init(void);
void pm_AgrHalt(PWR_ID_Type eID);
void pm_OpposeHalt(PWR_ID_Type eID);
void pm_EnterLowPwr(void);
void pm_ExitLowPwr(void);


#endif

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

