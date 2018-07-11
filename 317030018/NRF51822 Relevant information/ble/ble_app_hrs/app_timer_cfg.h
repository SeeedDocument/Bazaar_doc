/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision$
 */

 /** @cond To make doxygen skip this file */
 
/** @file
 *
 * @defgroup ble_sdk_app_hrs_eval_use_timers Application timers
 * @{
 * @ingroup ble_sdk_app_hrs_eval
 * @brief Definitions of timers to be used by the applications.
 *
 * @details This file contains application specific definitions.
 */

#ifndef APP_TIMER_CFG_H__
#define APP_TIMER_CFG_H__

/**@brief Preferred counter frequency of the application timer. Note that the actual frequency will
 *        normally be slightly different from this value. */
#define APP_TIMER_FREQ  1000

/**@brief Set of defined timers. */
typedef enum 
{
    APP_TIMER_BATTERY_LEVEL_MEAS,       /**< Battery level measurement (notification) timer. */
    APP_TIMER_HEART_RATE_MEAS,          /**< Heart Rate measurement (notification) timer. */
    APP_TIMER_CONN_PARAMS,              /**< Connection Parameters module timer. */
    APP_TIMER_LAST_ELEMENT              /**< This must always be the last entry in timer_id_t. */
} app_timer_id_t;

#endif // APP_TIMER_CFG_H__

/** @} */
/** @endcond */
