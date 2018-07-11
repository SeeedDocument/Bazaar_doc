/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic
 * Semiconductor ASA.Terms and conditions of usage are described in detail
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $$
 */

/** @file
 *
 * @defgroup ble_sdk_app_hrs_eval_main main.c
 * @{
 * @ingroup ble_sdk_app_hrs_eval
 * @brief Main file for Heart Rate Service Sample Application for nRF51822 evaluation board (PCA10001).
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services) for the nRF51822 evaluation board.
 * This application uses the @ref ble_sdk_lib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "ble_eval_board_pins.h" 
#include "ble_sensorsim.h"
#include "ble_stack_handler.h"
#include "app_timer.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"


#define DEVICE_NAME                          "HRS_EVAL"                                /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                    "NordicSemiconductor"                     /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                     40                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           180                                       /**< The advertising timeout in units of seconds. */

#define BATTERY_LEVEL_MEAS_INTERVAL          2000                                      /**< Battery level measurement interval (milliseconds). */
#define MIN_BATTERY_LEVEL                    81                                        /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL                    100                                       /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT              1                                         /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define HEART_RATE_MEAS_INTERVAL             1000                                      /*!< Heart rate measurement interval (milliseconds). */
#define MIN_HEART_RATE                       60                                        /*!< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                       100                                       /*!< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_CHANGE                    2                                         /*!< Value by which the heart rate is incremented/decremented during button press. */

#define HR_INC_BUTTON_PIN_NO                 EVAL_BOARD_BUTTON_0                       /**< Button used to increment heart rate. */
#define HR_DEC_BUTTON_PIN_NO                 EVAL_BOARD_BUTTON_1                       /**< Button used to decrement heart rate. */

#define MIN_CONN_INTERVAL                    ((1 * 800) / 2)                           /**< Minimum acceptable connection interval (0.5 seconds) */
#define MAX_CONN_INTERVAL                    (1 * 800)                                 /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                        0                                         /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     (4 * 100)                                 /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY       5000                                      /**< Time from initiating event (connect or start of notification) to first time ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        5000                                      /**< Time between each call to ble_gap_conn_param_update after the first (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                         /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT                    60                                        /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                       1                                         /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                         /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                      /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                         /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                         /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                        /**< Maximum encryption key size. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS        1200                                      /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION         3                                         /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS       270                                       /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */
#define TOTAL_NUMBER_OF_IO_PINS              32                                        /**< Total number of Input Output Pins in the chip. */

#define PPI_CHAN0_TO_TOGGLE_LED              0                                         /*!< The PPI Channel that connects CC0 compare event to the GPIOTE Task that toggles the Advertising LED. */
#define PPI_CHAN1_TO_TOGGLE_LED              1                                         /*!< The PPI Channel that connects CC1 compare event to the GPIOTE Task that toggles the Advertising LED. */

#define GPIOTE_CHAN_FOR_LED_TASK             0                                         /*!< The GPIOTE Channel used to perform write operation on the Advertising LED pin. */

#define TIMER_PRESCALER                      9                                         /*!< Prescaler setting for timer. */
#define CAPTURE_COMPARE_0_VALUE              0x1E84                                    /*!< Capture compare value that corresponds to 250 ms. */
#define CAPTURE_COMPARE_1_VALUE              (CAPTURE_COMPARE_0_VALUE * 2)             /*!< Capture compare value that corresponds to 500 ms. */

static ble_gap_sec_params_t                  s_sec_params;                             /*!< Security requirements for this application. */
static ble_gap_adv_params_t                  s_adv_params;                             /*!< Parameters to be passed to the stack when starting advertising. */
static ble_bas_t                             s_bas;                                    /*!< Structure used to identify the battery service. */
static ble_hrs_t                             s_hrs;                                    /*!< Structure used to identify the heart rate service. */
static volatile uint16_t                     s_cur_heart_rate;                         /**< Current heart rate value. */


/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 * @return     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)


/**@brief Assert macro callback function.
 *
 * @details This function will be called if the ASSERT macro fails.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * file_name)
{
    // Copying parameters to static variables because parameters are not accessible in debugger
    static volatile uint8_t  s_file_name[128];
    static volatile uint16_t s_line_num;

    strcpy((char *)s_file_name, (const char *)file_name);
    s_line_num = line_num;
    UNUSED_VARIABLE(s_file_name);
    UNUSED_VARIABLE(s_line_num);
    
    for (;;)
    {
        // Loop forever. On assert, the system can only recover on reset
    }
}


/**@brief Update the given Battery Level characteristic in Battery Service.
 * @param[in] battery_level The battery level in percentage
 */
static void battery_level_update(const uint8_t battery_level)
{
    uint32_t err_code;

    err_code = ble_bas_battery_level_update(&s_bas, battery_level);
    if (
        (err_code != NRF_SUCCESS)
        &&
        (err_code != NRF_ERROR_INVALID_STATE)
        &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        ASSERT(false);
    }
}


/**@brief Function to make the ADC start a conversion
 */
static void adc_start(void)
{
    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;
}


/**@brief Battery measurement timer timeout handler.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in]   timer_id   Id of timer that has expired.
 */
static void battery_level_meas_timeout_handler(app_timer_id_t timer_id)
{
    ASSERT(timer_id == APP_TIMER_BATTERY_LEVEL_MEAS);
    adc_start();
}


/**@brief Heart rate measurement timer timeout handler.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in]   timer_id   Id of timer that has expired.
 */
static void heart_rate_meas_timeout_handler(app_timer_id_t timer_id)
{
    uint32_t err_code;

    ASSERT(timer_id == APP_TIMER_HEART_RATE_MEAS);

    err_code = ble_hrs_heart_rate_measurement_send(&s_hrs, s_cur_heart_rate);
    if (
        (err_code != NRF_SUCCESS)
        &&
        (err_code != NRF_ERROR_INVALID_STATE)
        &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        ASSERT(false);
    }
}


/* @brief ADC Initialisation function.
 *
 * @details This function will initialise the ADC.
 */
static void adc_init()
{
    uint32_t err_code;
    uint32_t per_rdy;

    // Power on ADC module
    err_code = nrf_power_perpower_set(POWER_PERPOWER_ADC_Msk);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }

    // Wait for the power up of the ADC
    do
    {
        err_code = nrf_power_perrdy_get(&per_rdy);
        if (err_code != NRF_SUCCESS)
        {
            ASSERT(false);
        }
    } while ((per_rdy & POWER_PERPOWER_ADC_Msk) == 0);
    
    // Configure ADC
    NRF_ADC->INTENSET   = ADC_INTENSET_END_Msk;
    NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        << ADC_CONFIG_RES_Pos)     |
                          (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |
                          (ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
                          (ADC_CONFIG_PSEL_Disabled                   << ADC_CONFIG_PSEL_Pos)    |
                          (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->EVENTS_END = 0;                        
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;
    
    // Enable ADC interrupt
    err_code = nrf_nvic_ClearPendingIRQ(ADC_IRQn);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }

    err_code = nrf_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
    
    err_code = nrf_nvic_EnableIRQ(ADC_IRQn);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/* @brief Timer 1 initialisation function.
 *
 * @details This function will initialise Timer 1 peripheral. This timer is used only to
 *          generate capture compare events that toggle the advertising LED state.
 */
static void timer1_init(void)
{
    uint32_t err_code;
    uint32_t per_rdy;

    // Power on the TIMER1 peripheral
    err_code = nrf_power_perpower_set(POWER_PERPOWER_TIMER1_Msk);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }

    // Wait for the power up of TIMER1
    do
    {
        err_code = nrf_power_perrdy_get(&per_rdy);
        if (err_code != NRF_SUCCESS)
        {
            ASSERT(false);
        }
    } while ((per_rdy & POWER_PERPOWER_TIMER1_Msk) == 0);

    // Configure timer
    NRF_TIMER1->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER1->PRESCALER = TIMER_PRESCALER;

    // Clear the timer
    NRF_TIMER1->TASKS_CLEAR = 1;

    // Load the values to TIMER1 CC registers. The following values are calculated for generating
    // a 2 Hz waveform that will serve as input to the LED.
    NRF_TIMER1->CC[0] = CAPTURE_COMPARE_0_VALUE;
    NRF_TIMER1->CC[1] = CAPTURE_COMPARE_1_VALUE;

    // Make the Capture Compare 1 event to clear the timer. This will restart the timer.
    NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos;

    // There is no need to setup NRF_TIMER1->INTENSET register because the application do not need
    // to wake up the CPU on Timer interrupts.
}


/* @brief PPI initialisation function.
 *
 * @details This function will initialise Programmable Peripheral Interconnect peripheral. It will
 *          configure the PPI channels as follows -
 *              PPI Channel 0 - Connecting CC0 Compare event to GPIOTE Task to toggle the LED state
 *              PPI Channel 1 - Connecting CC1 Compare event to GPIOTE Task to toggle the LED state
 *          This configuration will feed a PWM input to the LED thereby making it flash in an
 *          interval that is dependent on the TIMER configuration.
 */
static void ppi_init(void)
{
    uint32_t err_code;

    // Configure PPI channel 0 to toggle ADVERTISING_LED_PIN_NO on every TIMER1 COMPARE[0] match
    err_code = nrf_ppi_channel_assign(PPI_CHAN0_TO_TOGGLE_LED,
                                     &(NRF_TIMER1->EVENTS_COMPARE[0]),
                                     &(NRF_GPIOTE->TASKS_OUT[0]));
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }

    // Configure PPI channel 1 to toggle ADVERTISING_LED_PIN_NO on every TIMER1 COMPARE[1] match
    err_code = nrf_ppi_channel_assign(PPI_CHAN1_TO_TOGGLE_LED,
                                      &(NRF_TIMER1->EVENTS_COMPARE[1]),
                                      &(NRF_GPIOTE->TASKS_OUT[0]));
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }

    // Enable the PPI channels 0, 1 and 2 thereby connecting the TIMER compare events to the GPIOTE
    // task that toggles the LED.
    err_code = nrf_ppi_channel_enable_set(PPI_CHEN_CH0_Msk | PPI_CHEN_CH1_Msk);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/**@brief Initialize GPIOTE module
 * @details This will power up the GPIOTE module and initialise it to receive button presses as PORT        
            event.
 */
static void gpiote_init(void)
{
    uint32_t err_code;
    uint32_t per_rdy;

    // Power on GPIOTE module
    err_code = nrf_power_perpower_set(POWER_PERPOWER_GPIOTE_Msk);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }

    do
    {
        err_code = nrf_power_perrdy_get(&per_rdy);
        if (err_code != NRF_SUCCESS)
        {
            ASSERT(false);
        }
    } while ((per_rdy & POWER_PERPOWER_GPIOTE_Msk) == 0);

    // Initialize GPIOTE module
    NRF_GPIOTE->INTENCLR     = 0xffffffffUL;
    NRF_GPIOTE->INTENSET     = GPIOTE_INTENSET_PORT_Msk; // To receive button presses as PORT event

    // Enable GPIOTE interrupt
    err_code = nrf_nvic_ClearPendingIRQ(GPIOTE_IRQn);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }

    err_code = nrf_nvic_SetPriority(GPIOTE_IRQn, NRF_APP_PRIORITY_LOW);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }

    err_code = nrf_nvic_EnableIRQ(GPIOTE_IRQn);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/**@brief Initialize buttons.
 */
static void buttons_init(void)
{
    // Configure HR_INC_BUTTON_PIN_NO (Button 0) as a wake up button and also configure for 'pull
    // up' because the eval board does not have external pull up resistors connected to the buttons
    NRF_GPIO->PIN_CNF[HR_INC_BUTTON_PIN_NO] =
                                          (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)    |
                                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)  |
                                          (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos)   |
                                          (GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos)  |
                                          (GPIO_PIN_CNF_SENSE_Low     << GPIO_PIN_CNF_SENSE_Pos);

    // Configure HR_DEC_BUTTON_PIN_NO (Button 1) for 'pull up' because the eval board does not have
    // a external pull up resistors connected to the buttons
    NRF_GPIO->PIN_CNF[HR_DEC_BUTTON_PIN_NO] =
                                          (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)    |
                                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)  |
                                          (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos)   |
                                          (GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos)  |
                                          (GPIO_PIN_CNF_SENSE_Low     << GPIO_PIN_CNF_SENSE_Pos);
}


/**@brief Timer initialization.
 *
* @details Initializes the timer module. This creates and starts application timers.
*/
static void timers_init(void)
{
    uint32_t err_code;
    
    // Initialize timer module
    err_code = app_timer_init(NULL);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/**@brief GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = ble_gap_device_name_set(&sec_mode, DEVICE_NAME);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }

    err_code = ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
    
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = ble_gap_ppcp_set(&gap_conn_params);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_DISC_MODE_GENERAL | BLE_GAP_DISC_BR_EDR_NOT_SUPPORTED;
    
    ble_uuid_t adv_uuids[] = 
    {
        {BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE}, 
        {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE}, 
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    };

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, NULL);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }

    // Initialise advertising parameters (used when starting advertising)
    memset(&s_adv_params, 0, sizeof(s_adv_params));
    
    s_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    s_adv_params.p_peer_addr = NULL;                           // Undirected advertisement
    s_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    s_adv_params.interval    = APP_ADV_INTERVAL;
    s_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}


/**@brief Initialize services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_hrs_init_t hrs_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    uint8_t        body_sensor_location;
    
    // Initialize Heart Rate Service
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;
    
    memset(&hrs_init, 0, sizeof(hrs_init));
    
    hrs_init.is_sensor_contact_supported = false;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&s_hrs, &hrs_init);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
    
    // Initialize Battery Service
    memset(&bas_init, 0, sizeof(bas_init));
    
    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;
    
    err_code = ble_bas_init(&s_bas, &bas_init);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }

    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));
    
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/**@brief Initialize security parameters.
 */
static void sec_params_init(void)
{
    s_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    s_sec_params.bond         = SEC_PARAM_BOND;
    s_sec_params.mitm         = SEC_PARAM_MITM;
    s_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    s_sec_params.oob          = SEC_PARAM_OOB;  
    s_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    s_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}

/**@brief Start application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;
    
    // Start application timers
    err_code = app_timer_start(APP_TIMER_BATTERY_LEVEL_MEAS, 
                               battery_level_meas_timeout_handler, 
                               BATTERY_LEVEL_MEAS_INTERVAL, 
                               APP_TIMER_MODE_REPEATED, 
                               APP_TIMER_LOWEST_PRI);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
    
    err_code = app_timer_start(APP_TIMER_HEART_RATE_MEAS, 
                               heart_rate_meas_timeout_handler, 
                               HEART_RATE_MEAS_INTERVAL, 
                               APP_TIMER_MODE_REPEATED, 
                               APP_TIMER_LOWEST_PRI);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/**@brief   Function to start flashing the LED.
 * @details This will start the TIMER1 and enable the GPIOTE task that toggles the LED.
 *          The PPI and GPIOTE configurations done by this app will make this action result in the
 *          flashing of the LED.
 */
static void led_start_flashing(void)
{
    // Configure the GPIOTE Task to toggle the LED state.
    nrf_gpiote_task_config(GPIOTE_CHAN_FOR_LED_TASK,
                           ADVERTISING_LED_PIN_NO,
                           NRF_GPIOTE_POLARITY_TOGGLE,
                           NRF_GPIOTE_INITIAL_VALUE_LOW);

    NRF_TIMER1->TASKS_START = 1;
}


/**@brief   Function to stop flashing the LED.
 * @details This will stop the TIMER1 and disable the GPIOTE task that toggles the LED.
 *          The PPI and GPIOTE configurations done by this app will
 *          make this action result in the turning off the LED.
 */
static void led_stop_flashing(void)
{
    // Disable the GPIOTE_CHAN_FOR_LED_TASK. This is because when an task has been configured
    // to operate on a pin, the pin can only be written from GPIOTE module. Attempting to write a
    // pin (using nrf_gpio_pin_clear() below for example) as a normal GPIO pin will have no effect.
    NRF_GPIOTE->CONFIG[GPIOTE_CHAN_FOR_LED_TASK] =
                                            (GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos);

    NRF_TIMER1->TASKS_STOP = 1;

    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
}


/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = ble_gap_adv_start(&s_adv_params);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
        
    led_start_flashing();
}


/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    ASSERT(false);
}


/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = s_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/**@brief Function to put the chip in System OFF Mode
 */
static void system_off_mode_enter(void)
{
    uint32_t err_code = nrf_power_system_off();
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code = NRF_SUCCESS;
    static uint16_t                  s_conn_handle = BLE_CONN_HANDLE_INVALID;
    static ble_gap_evt_auth_status_t s_auth_status;
    ble_gap_enc_info_t *             p_enc_info;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            led_stop_flashing();

            s_conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;

            application_timers_start(); // Start timers used to generate battery and HR measurements.

            // Initialize the current heart rate to the average of max and min values. So that 
            // everytime a new connection is made, the heart rate starts from the same value.
            s_cur_heart_rate = (MAX_HEART_RATE + MIN_HEART_RATE) / 2;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            // Go to system-off mode, should not return from this function, wakeup will trigger
            // a reset.
            system_off_mode_enter();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = ble_gap_sec_params_reply(s_conn_handle, 
                                                BLE_GAP_SEC_STATUS_SUCCESS, 
                                                &s_sec_params);
            break;
            
        case BLE_GAP_EVT_AUTH_STATUS:
            s_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = ble_gatts_sys_attr_set(s_conn_handle, NULL, 0);
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &s_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = ble_gap_sec_info_reply(s_conn_handle, p_enc_info, NULL);
            }
            else
            {
                // No keys found for this device
                err_code = ble_gap_sec_info_reply(s_conn_handle, NULL, NULL);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_TYPE_ADVERTISEMENT)
            { 
                led_stop_flashing();
                
                // Go to system-off mode (this function will not return; wakeup will cause a reset)
                system_off_mode_enter();
            }
            break;
            
        default:
            break;
    }
    
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/** @brief Converts the input voltage (in milli volts) into percentage of 3.0 Volts.
 *
 *  @details The calculation is based on a linearized version of the battery's discharge
 *           curve. 3.0V returns 100% battery level. The limit for power failure is 2.1V and
 *           is considered to be the lower boundary.
 *
 *           The discharge curve for CR2032 is non-linear. In this model it is split into
 *           4 linear sections:
 *           - Section 1: 3.0V - 2.9V = 100% - 42% (58% drop on 100 mV)
 *           - Section 2: 2.9V - 2.74V = 42% - 18% (24% drop on 160 mV)
 *           - Section 3: 2.74V - 2.44V = 18% - 6% (12% drop on 300 mV)
 *           - Section 4: 2.44V - 2.1V = 6% - 0% (6% drop on 340 mV)
 *
 *           These numbers are by no means accurate. Temperature and
 *           load in the actual application is not accounted for!
 *
 *  @param[in] mvolts The voltage in mV
 *
 *  @return    Battery level in percent.
*/
static uint8_t battery_level_in_percent(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 3000)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
    else if (mvolts > 2740)
    {
        battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
    else if (mvolts > 2440)
    {
        battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
    else if (mvolts > 2100)
    {
        battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}


/**@brief ADC interrupt handler.
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void ADC_IRQHandler(void)
{
    if (NRF_ADC->EVENTS_END != 0)
    {
        uint8_t     adc_result;
        uint16_t    batt_lvl_in_milli_volts;
        uint8_t     percentage_batt_lvl;

        NRF_ADC->EVENTS_END     = 0;
        adc_result              = NRF_ADC->RESULT;
        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        percentage_batt_lvl     = battery_level_in_percent(batt_lvl_in_milli_volts);

        battery_level_update(percentage_batt_lvl);
    }
}


/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_hrs_on_ble_evt(&s_hrs, p_ble_evt);
    ble_bas_on_ble_evt(&s_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief BLE stack initialization.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 *
 */
static void ble_stack_init(void)
{
    uint32_t err_code = ble_stack_handler_init(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, ble_evt_dispatch);
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/**@brief GPIOTE interrupt handler.
 */
void GPIOTE_IRQHandler(void)
{
    if (
        (NRF_GPIOTE->EVENTS_PORT != 0)
        &&
        ((NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_PORT_Msk) != 0)
    )
    {
        NRF_GPIOTE->EVENTS_PORT = 0;

        // Handle button for incrementing heart rate
        if (nrf_gpio_pin_read(HR_INC_BUTTON_PIN_NO) == 0)
        {
            s_cur_heart_rate += HEART_RATE_CHANGE;
            if (s_cur_heart_rate > MAX_HEART_RATE)
            {
                s_cur_heart_rate = MIN_HEART_RATE; // Loop back
            }
        }
        
        // Handle button for decrementing heart rate
        if (nrf_gpio_pin_read(HR_DEC_BUTTON_PIN_NO) == 0)
        {
            s_cur_heart_rate -= HEART_RATE_CHANGE;
            if (s_cur_heart_rate < MIN_HEART_RATE)
            {
                s_cur_heart_rate = MAX_HEART_RATE; // Loop back
            }
        }
    }
}


/**@brief PINs initialisation
 * @details This function will make all the 32 pins of the chip to disconnect input buffer.
            This is used to prevent floating pins which result in unwanted current consumption.
            This function should be called BEFORE configuring buttons and LEDs.
 */
static void pins_init(void)
{
    uint8_t i;
    
    NRF_GPIO->DIRCLR = 0xFFFFFFFF;
    
    for (i = 0; i < TOTAL_NUMBER_OF_IO_PINS; i++)
    {
        NRF_GPIO->PIN_CNF[i] = GPIO_PIN_CNF_INPUT_Msk;
    }
}


/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = nrf_wait_for_app_event();
    if (err_code != NRF_SUCCESS)
    {
        ASSERT(false);
    }
}


/**@brief Function to initalise the peripherals needed for generating PWM input to flash an LED.
 * @details This function initialises the GPIOTE, TIMER1 and PPI channels to generate PWM input
 *          to the advertising LED.
 */
static void pwm_config_init(void)
{
    gpiote_init();
    ppi_init();
    timer1_init();
}


/**@brief Application main function.
 */
int main(void)
{
    bool is_startup_on_button_press = true;

    pins_init();
    buttons_init();

    // Check button states.
    if (
        (nrf_gpio_pin_read(HR_INC_BUTTON_PIN_NO) != 0)  // If the button 0 is NOT pressed
        &&
        (nrf_gpio_pin_read(HR_DEC_BUTTON_PIN_NO) != 0)  // If the button 1 is NOT pressed
       )
    {
        is_startup_on_button_press = false;
    }

    ble_stack_init(); // This is needed here because the nrf_power_system_off() that is called
                      // in the code below needs the SoftDevice to be enabled.

    if (!is_startup_on_button_press)
    {
       // The startup was not because of button presses. This is the first start.
       // Go into System-Off mode.
        system_off_mode_enter();
    }

    timers_init();
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
    pwm_config_init();
    adc_init();
    sec_params_init();

    advertising_start();
    
    // Enter main loop
    for (;;)
    {
        power_manage();
    }
}

/** 
 * @}
 */
