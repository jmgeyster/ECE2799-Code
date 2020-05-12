// Main.c 

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_tmp.c"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "nrf_delay.h"

#include "nrf_drv_config.h"
#include "app_util_platform.h"
#include "ble_tmp.h"
#include "TWI.h"
#include "TMP116.h"
#include "TMP116.c"
#include "nrf_gpio.h"


// -------------------------------------------------------------------------------------------------------------------------------------------------------------

#define MAX_PENDING_TRANSACTIONS    5

// Buffer for data read from sensors.
#define BUFFER_SIZE  2
static uint8_t m_buffer[BUFFER_SIZE];

#define NORM_WARNING				0x11
#define HEAT_WARNING 				0x78
#define COLD_WARNING 				0x56
#define SHUTOFF_WARNING 		0x34


typedef struct
{
    int16_t temp;
	
} sample_t;
static sample_t m_sample = { 0 };


static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);


static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = 4,
       .sda                = 5,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
    APP_ERROR_CHECK(err_code);
}

void read_all_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        printf("read_all_cb - error: %d\r\n", (int)result);
        return;
    }

    sample_t * p_sample = &m_sample;

    uint8_t temp_hi = m_buffer[0];
    uint8_t temp_lo = m_buffer[1];


    p_sample->temp = TMP116_GET_TEMPERATURE_VALUE(temp_hi, temp_lo);

    }

static void read_all(void)
{
    static app_twi_transfer_t const transfers[] =
    {
        TMP116_READ_TEMP(&m_buffer[0])
        
    };
    static app_twi_transaction_t const transaction =
    {
        .callback            = read_all_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}
		
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                          /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "No_Heatstroke"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "ECE2799 Team 9"                      			/**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       120                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */


#define TEMPERATURE_MEAS_INTERVAL      	 APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER) /**< Temperature measurement interval (ticks). */
#define BLINKING_MEAS_INTERVAL      	 	 APP_TIMER_TICKS(800, APP_TIMER_PRESCALER) /**< Blink interval (ticks). */



#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(400, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(650, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */


static ble_tmp_t												 m_tmp;																			/**< Structure used to identify the temperature service. */		
uint8_t  warning = 0x30;
int32_t temperature = 80;
bool Blink_Flag;
bool ShutOff_Flag;


APP_TIMER_DEF(m_temperature_timer_id);																							/**< Temperaturature timer. */
APP_TIMER_DEF(m_blinking_timer_id);																									/**< Blinking timer. */

static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_ALERT_NOTIFICATION_SERVICE,            BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */




/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for performing temperature measurement and updating the Temperature characteristic
 *        in Temperature Service.
 */
static void temperature_update(void)
{
	uint32_t err_code;
	if(ShutOff_Flag == false)
		{
			sd_temp_get(&temperature);
			
			if(temperature < 0)							// Check if Temperature is < than 32F (0C also 0 in tmp116 format)
				{
					warning = COLD_WARNING;
				}
			else if(temperature >= 152)		// Check if Temperature is > 100F (~38C which is 4864 in the tmp116 format)
				{
					warning = HEAT_WARNING;
				}
			else		// Check if Temperature is in safe range (0C - 100F)
				{
					warning = NORM_WARNING;
				}
			
			err_code = ble_tmp_temperature_update(&m_tmp, warning);
			if ((err_code != NRF_SUCCESS) &&
					(err_code != NRF_ERROR_INVALID_STATE) &&
					(err_code != BLE_ERROR_NO_TX_PACKETS) &&
					(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
				{
					APP_ERROR_HANDLER(err_code);
				}
		}
}



/**@brief Function for handling the Temperature measurement timer timeout.
 *
 * @details This function will be called each time the temperature measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void temperature_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    temperature_update();
}

static void blinking_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

//		nrf_gpio_pin_toggle(8);				//	Toggle state of LED every time handler is called (Create Blink)
		nrf_gpio_pin_toggle(21);
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
	
		err_code = app_timer_create(&m_temperature_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                temperature_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&m_blinking_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                blinking_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Temperature service.
 */
static void services_init(void)
{
    uint32_t       err_code;

		ble_tmp_init_t tmp_init;


		// Initialize Temperature Service.
    memset(&tmp_init, 0, sizeof(tmp_init));

    // Here the sec level for the Temperature Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&tmp_init.temperature_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&tmp_init.temperature_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&tmp_init.temperature_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&tmp_init.temperature_report_read_perm);

    tmp_init.evt_handler          = NULL;
    tmp_init.support_notification = true;
    tmp_init.p_report_ref         = NULL;
    tmp_init.initial_temp_level   = 100;

    err_code = ble_tmp_init(&m_tmp, &tmp_init);
    APP_ERROR_CHECK(err_code);
			
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;
	
		err_code = app_timer_start(m_temperature_timer_id, TEMPERATURE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

}

static void blink_timer_start(void)
{
    uint32_t err_code;
	
		err_code = app_timer_start(m_blinking_timer_id, BLINKING_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
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
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    uint32_t err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
						blink_timer_start();
            break;
        case BLE_ADV_EVT_IDLE:
//						nrf_gpio_pin_clear(8);					// Turn off the LED
//						nrf_gpio_cfg_sense_input(30, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
						nrf_gpio_pin_set(21);
						nrf_gpio_cfg_sense_input(17, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
						err_code = app_timer_stop(m_blinking_timer_id);				// Shut off blinking timer
						APP_ERROR_CHECK(err_code);					
//					nrf_gpio_pin_set(8);																	// Turn on LED
						nrf_gpio_pin_clear(21);						
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
		ble_tmp_on_ble_evt(&m_tmp, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_4000MS_CALIBRATION, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}





/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds = true;
	
		Blink_Flag = true;
		ShutOff_Flag = false;
		
		nrf_gpio_cfg_output(21);           							
		nrf_gpio_cfg_input(17, NRF_GPIO_PIN_PULLUP);		
//		nrf_gpio_cfg_output(8);           							// Configure pin 8 as output for On/Off LED
//		nrf_gpio_cfg_input(30, NRF_GPIO_PIN_NOPULL);		// Congigure pin 30 as input for On/Off PB
//		nrf_gpio_cfg_input(7, NRF_GPIO_PIN_NOPULL);			// Configure pin 7 as input for Pairing PB
//		nrf_gpio_cfg_input(2, NRF_GPIO_PIN_NOPULL);			// Configure pin 2 as input for ALERT/DATA_RDY
	
	
    // Initialize.
    app_trace_init();
    timers_init();
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
//		twi_config();
	
//		APP_ERROR_CHECK(app_twi_perform(&m_app_twi, tmp116_init_transfers, 			// Perform configuration of TMP116
//        TMP116_INIT_TRANSFER_COUNT, NULL));
		
		// Start execution.
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

		nrf_delay_ms(500);

    // Enter main loop.
    for (;;)
    {
			if (nrf_gpio_pin_read(17) == 0)
				{
					ShutOff_Flag = true;
					err_code = ble_tmp_temperature_update(&m_tmp, SHUTOFF_WARNING);
					if ((err_code != NRF_SUCCESS) &&
							(err_code != NRF_ERROR_INVALID_STATE) &&
							(err_code != BLE_ERROR_NO_TX_PACKETS) &&
							(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
						{
							APP_ERROR_HANDLER(err_code);
						}
					nrf_delay_ms(5000);
						
//					nrf_gpio_pin_clear(8);																													// Turn off the LED
//					nrf_gpio_cfg_sense_input(30, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);			// Configure wake-up Button
					nrf_gpio_pin_set(21);
					nrf_gpio_cfg_sense_input(17, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
					sleep_mode_enter();
				}
    }
}
