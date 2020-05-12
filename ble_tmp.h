
#ifndef BLE_TMP_H__
#define BLE_TMP_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/**@brief Temperature Service event type. */
typedef enum
{
    BLE_TMP_EVT_NOTIFICATION_ENABLED,                             /**< Battery value notification enabled event. */
    BLE_TMP_EVT_NOTIFICATION_DISABLED                             /**< Battery value notification disabled event. */
} ble_tmp_evt_type_t;

/**@brief Temperature Service event. */
typedef struct
{
    ble_tmp_evt_type_t evt_type;                                  /**< Type of event. */
} ble_tmp_evt_t;

// Forward declaration of the ble_tmp_t type. 
typedef struct ble_tmp_s ble_tmp_t;

/**@brief Temperature Service event handler type. */
typedef void (*ble_tmp_evt_handler_t) (ble_tmp_t * p_tmp, ble_tmp_evt_t * p_evt);

/**@brief Temperature Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_tmp_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Temperature Service. */
    bool                          support_notification;           /**< TRUE if notification of Battery Level measurement is supported. */
    ble_srv_report_ref_t *        p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Battery Level characteristic */
    uint8_t                       initial_temp_level;             /**< Initial temperature level */
    ble_srv_cccd_security_mode_t  temperature_char_attr_md;		    /**< Initial security level for temperature characteristics attribute */
    ble_gap_conn_sec_mode_t       temperature_report_read_perm; 	/**< Initial security level for temperature report read attribute */
} ble_tmp_init_t;

/**@brief Temperature Service structure. This contains various status information for the service. */
struct ble_tmp_s
{
    ble_tmp_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Temperature Service. */
    uint16_t                      service_handle;                 /**< Handle of Temperature Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      temperature_handles;	          /**< Handles related to the Battery Level characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint8_t                       temperature_last;            		/**< Last Battery Level measurement passed to the Temperature Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of Battery Level is supported. */
};

/**@brief Function for initializing the Temperature Service.
 *
 * @param[out]  p_tmp       Temperature Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_tmp_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_tmp_init(ble_tmp_t * p_tmp, const ble_tmp_init_t * p_tmp_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Temperature Service.
 *
 * @note For the requirements in the tmp specification to be fulfilled,
 *       ble_tmp_temperature_update() must be called upon reconnection if the
 *       temperature level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_tmp      Temperature Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_tmp_on_ble_evt(ble_tmp_t * p_tmp, ble_evt_t * p_ble_evt);

/**@brief Function for updating the temperature level.
 *
 * @details The application calls this function after having performed a temperature measurement. If
 *          notification has been enabled, the temperature level characteristic is sent to the client.
 *
 * @note For the requirements in the tmp specification to be fulfilled,
 *       this function must be called upon reconnection if the temperature level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_tmp          Temperature Service structure.
 * @param[in]   temperature  New temperature measurement value .
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_tmp_temperature_update(ble_tmp_t * p_tmp, uint8_t warning);

#endif // BLE_TMP_H__

/** @} */
