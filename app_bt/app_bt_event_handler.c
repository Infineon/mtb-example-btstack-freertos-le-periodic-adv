/*******************************************************************************
 * File Name: app_bt_event_handler.c
 *
 * Description:
 * This file contains the starting point of Bluetooth LE Periodic Adv application.
 * The wiced_bt_stack_init() registers for Bluetooth events in this main function.
 * The Bluetooth Management callback manages the Bluetooth events and the
 * application developer can customize the functionality and behavior depending on
 * the Bluetooth events. The Bluetooth Management callback acts like a
 * Finite State Machine (FSM) for the SoC.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2023 Cypress Semiconductor Corporation (an Infineon company)
 *******************************************************************************/

/*******************************************************************************
 *                               Includes
 *******************************************************************************/
#include "stdio.h"
#include "app_bt_event_handler.h"

#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"

#include "cyhal_wdt.h"

#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

#include "app_bt_periodic_adv.h"

/*******************************************************************************
 *                                Macros
 *******************************************************************************/


/*******************************************************************************
 *                               Global Variables
 *******************************************************************************/

/*******************************************************************************
 *                           Function Prototypes
 *******************************************************************************/

/*******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/**
 * Function Name:
 * hci_trace_cback
 *
 * Function Description:
 * @brief This Function route the btstack trace to HCI UART
 *
 * @param void
 *
 * @return void
 */
#ifdef ENABLE_BT_SPY_LOG
void hci_trace_cback(wiced_bt_hci_trace_type_t type,
                     uint16_t length, uint8_t* p_data)
{
    cybt_debug_uart_send_hci_trace(type, length, p_data);
}
#endif

/**
 * Function Name:
 * app_bt_event_management_callback
 *
 * Function Description:
 * @brief This is a Bluetooth stack event handler function to receive management events
 *  from the Bluetooth LE stack and process as per the application.
 *
 * @param event:  Bluetooth LE event code of one byte length
 * @param p_event_data:  Pointer to Bluetooth LE management event
 *                                        structures
 *
 * @return wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 */
wiced_result_t
app_bt_event_management_callback(wiced_bt_management_evt_t event,
                           wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_dev_status_t       bt_dev_status       = WICED_SUCCESS;

    printf("\nBluetooth Management Event: %d,\t%s\r\n", event, app_bt_util_get_btm_event_name(event));
    printf("\r\n");

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Perform application-specific initialization */
#ifdef ENABLE_BTSS_TRACES
            printf("\nenable btss traces\t");
            extern void cybt_enable_btss_traces(void);
            cybt_enable_btss_traces();
#endif

#ifdef ENABLE_BT_SPY_LOG
            wiced_bt_dev_register_hci_trace(hci_trace_cback);
#endif
            /* Periodic Advertisement Init */
            app_bt_periodic_adv_init();

            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            /* Advertisement State Changed */

            break;

        case BTM_SECURITY_REQUEST_EVT:
            /* Need to compare with BT-SDK remote here for this event */
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:

            break;

        case BTM_PIN_REQUEST_EVT:

            break;

        case BTM_PASSKEY_REQUEST_EVT:

            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:

            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:

            break;

        case BTM_PAIRING_COMPLETE_EVT:

            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Save the identity keys to the NV storage */
            printf("Local Identity Key Update\n");
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* Retrieve the identity keys from the NV storage */

            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            /* Encryption Status Change */

            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:

            break;

        case BTM_SECURITY_FAILED_EVT:
            /* Handle pairing Failure */
            printf("Pairing Failed\r\n");
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
            printf("BTM_BLE_PHY_UPDATE_EVT,\r\n "
                    "PHY Tx value is: %d, \r\n"
                    "PHY Rx value is: %d \r\n",
                    p_event_data->ble_phy_update_event.tx_phy,
                    p_event_data->ble_phy_update_event.rx_phy);
            break;

        case BTM_DISABLED_EVT:
            /* Bluetooth Controller and Host Stack Disabled */
            printf("BTM_DISABLED_EVT\r\n");
            break;

        case BTM_BLE_DATA_LENGTH_UPDATE_EVENT:
            printf("BTM_BLE_DATA_LENGTH_UPDATE_EVENT, \r\n"
                    "Max tx octets is :%d ,\r\n"
                    "Max rx octets is :%d \r\n",
                    p_event_data->ble_data_length_update_event.max_tx_octets,
                    p_event_data->ble_data_length_update_event.max_rx_octets);
            break;

        default:
            printf("\nUnhandled Bluetooth Management Event: %d\r\n", event);
            break;
    }

    return (bt_dev_status);
}
