/*******************************************************************************
 * File Name: app_bt_periodic_adv.c
 *
 * Description: This file consists of the bt periodic advertiser function definitions
 *
 *******************************************************************************
 * Copyright 2023 Cypress Semiconductor Corporation (an Infineon company)
 *******************************************************************************/

/******************************************************************************
 *                                INCLUDES
 ******************************************************************************/
/* FreeRTOS header files */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "timers.h"

#include "stdio.h"
#include "app_bt_periodic_adv.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"

/******************************************************************************
 *                                DEFINES
 ******************************************************************************/
#define SIMULATE_SENSOR_TIMEOUT_IN_MS       (5000)

/******************************************************************************
 *                                GLOBAL VARIABLES
 ******************************************************************************/
uint8_t simulate_sensor_data[240] = {0};
#if (APP_BT_PERIODIC_CHANGE_ADV_DATA == 1)
static TimerHandle_t simulate_sensor_timer = NULL;
#endif

wiced_ble_ext_adv_params_t ext_adv_params_t ={
    .event_properties = APP_BT_PERIODIC_ADV_EVENT_PROP,
    .primary_adv_int_min = APP_BT_PERIODIC_ADV_PRI_ADV_MIN,
    .primary_adv_int_max = APP_BT_PERIODIC_ADV_PRI_ADV_MAX,
    .primary_adv_channel_map = APP_BT_PERIODIC_ADV_PRI_CH_MAP,
    .own_addr_type = BLE_ADDR_PUBLIC,
    .peer_addr_type = BLE_ADDR_PUBLIC,
    .peer_addr = {0},
    .adv_filter_policy= BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN,
    .adv_tx_power = APP_BT_PERIODIC_ADV_TX_POWER,
    .primary_adv_phy = WICED_BLE_EXT_ADV_PHY_1M,
    .secondary_adv_max_skip = APP_BT_PERIODIC_SEC_ADV_MAX_SKIP,
    .secondary_adv_phy = WICED_BLE_EXT_ADV_PHY_2M,
    .adv_sid = APP_BT_PERIODIC_ADV_SET_ID,
    .scan_request_not = WICED_BLE_EXT_ADV_SCAN_REQ_NOTIFY_DISABLE,
    .primary_phy_opts = WICED_BLE_EXT_ADV_PHY_OPTIONS_NO_PREFERENCE,
    .secondary_phy_opts = WICED_BLE_EXT_ADV_PHY_OPTIONS_NO_PREFERENCE,
};

wiced_ble_padv_params_t padv_params_t ={
    .adv_int_min = APP_BT_PERIODIC_ADV_INT_MIN,
    .adv_int_max = APP_BT_PERIODIC_ADV_INT_MAX,
    .adv_properties = 0,
    .rsp_slot_delay = 0,
    .rsp_slot_num = 0,
    .rsp_slot_spacing = 0,
    .subevent_interval =0,
    .subevent_num = 0,
};
/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/\
#if (APP_BT_PERIODIC_CHANGE_ADV_DATA == 1)
/**
 * Function Name:
 * app_bt_periodic_adv_generate_simulate_data_cb
 *
 * Function Description:
 * @brief This Function is the callback of 5s periodic timer expired to update
 * the advertiser data
 *
 * @param cb_params: rtos timer callback in-parameters
 *
 * @return void
 */
void app_bt_periodic_adv_generate_simulate_data_cb(TimerHandle_t cb_params)
{
    uint16_t i = 0;
    wiced_bt_dev_status_t result = WICED_BT_SUCCESS;
    static uint8_t fake_data;

    fake_data ++;

    for(i = 0; i < sizeof(simulate_sensor_data); i ++)
    {
        simulate_sensor_data[i] = fake_data;
    }

    result = wiced_ble_padv_set_adv_data(APP_BT_PERIODIC_ADV_HANDLE, sizeof(simulate_sensor_data), simulate_sensor_data);
    if(result != WICED_BT_SUCCESS)
    {
        printf("%s set periodic adv data result:0x%04x\r\n", __FUNCTION__, result);
    }
    else
    {
        printf("%s set periodic adv data fake_data:%d\r\n", __FUNCTION__, fake_data);
    }
}
#endif

/**
 * Function Name:
 * app_bt_periodic_adv_init
 *
 * Function Description:
 * @brief Initialize ble periodic adv, set the necessary parameters
 *
 * @param void
 *
 * @return void
 */
void app_bt_periodic_adv_init(void)
{
    uint8_t i = 0;
    wiced_bt_dev_status_t result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t  bt_dev_bda = {0x20, 0x82, 0x9B, 0x01, 0x22, 0x33};
    wiced_bt_device_address_t  local_addr;
    wiced_ble_ext_adv_duration_config_t  ext_cfg = {APP_BT_PERIODIC_ADV_HANDLE, 0, 0};

    for(i = 0; i < sizeof(simulate_sensor_data); i ++)
    {
        simulate_sensor_data[i] = i;
    }

    result = wiced_bt_set_local_bdaddr(bt_dev_bda, BLE_ADDR_PUBLIC);
    if(result != WICED_BT_SUCCESS)
    {
        printf("%s set local address result:0x%04x\r\n", __FUNCTION__, result);
        return;
    }

    wiced_bt_dev_read_local_addr(local_addr);
    printf("%s local address info:0x%02x:%02x:%02x:%02x:%02x:%02x\r\n", __FUNCTION__, local_addr[0], local_addr[1],
                    local_addr[2], local_addr[3], local_addr[4], local_addr[5]);

    result = wiced_ble_ext_adv_set_params(APP_BT_PERIODIC_ADV_HANDLE,&ext_adv_params_t);

    if(result != WICED_BT_SUCCESS)
    {
        printf("%s set extern adv params result:0x%04x\r\n", __FUNCTION__, result);
        return;
    }
    
    result = wiced_ble_padv_set_adv_params(APP_BT_PERIODIC_ADV_HANDLE,&padv_params_t);

    if(result == WICED_BT_SUCCESS)
    {
        result = wiced_ble_padv_set_adv_data(APP_BT_PERIODIC_ADV_HANDLE, sizeof(simulate_sensor_data), simulate_sensor_data);
        if(result == WICED_BT_SUCCESS)
        {
            result = wiced_ble_padv_enable_adv(APP_BT_PERIODIC_ADV_HANDLE, 1);
            if(result != WICED_BT_SUCCESS)
            {
                printf("%s enable periodic adv result:0x%04x\r\n", __FUNCTION__, result);
            }
        }
        else
        {
            printf("%s set periodic adv data result:0x%04x\r\n", __FUNCTION__, result);
        }
    }
    else
    {
        printf("%s set periodic adv params result:0x%04x\r\n", __FUNCTION__, result);
    }

    // Set extended ADV enable
    wiced_ble_ext_adv_enable(WICED_TRUE, 1, &ext_cfg);

#if (APP_BT_PERIODIC_CHANGE_ADV_DATA == 1)
    simulate_sensor_timer = xTimerCreate("Simulate Sensor Timer",
                            SIMULATE_SENSOR_TIMEOUT_IN_MS,
                            pdTRUE,
                            NULL ,
                            app_bt_periodic_adv_generate_simulate_data_cb);

    if (simulate_sensor_timer != NULL && pdPASS != xTimerStart(simulate_sensor_timer, 10u))
    {
        printf("Failed to start simulate_sensor_timer!\r\n");
        CY_ASSERT(0);
    }
#endif
}

/* [] END OF FILE */
