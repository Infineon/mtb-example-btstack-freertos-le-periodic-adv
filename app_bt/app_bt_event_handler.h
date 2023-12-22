/*******************************************************************************
 * File Name: app_bt_event_handler.h
 *
 * Description:
 * This file contains the interfaces for application procedure to handle the
 * Bluetooth events in Bluetooth Management callback. The Bluetooth Management
 * callback acts like a Finite State Machine (FSM) for the SoC.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2023 Cypress Semiconductor Corporation (an Infineon company)
 *******************************************************************************/
#ifndef __APP_BT_TASK_H__
#define __APP_BT_TASK_H__

/*******************************************************************************
 *                               Includes
 *******************************************************************************/

#include "cycfg_gatt_db.h"
#include "cycfg_gap.h"
#include "cycfg_bt_settings.h"
#include "cyabs_rtos.h"

#include "app_bt_utils.h"

#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_memory.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_l2c.h"

#include <task.h>
#include "timers.h"

/* Bitflags for LE secure pairing keys IO capabilities event */
#define PAIRING_CAPS_KEYS_FLAG      \
        (BTM_LE_KEY_PENC | BTM_LE_KEY_PID)
/* Key Size for LE secure pairing key IO capabilities event */
#define PAIRING_CAPS_KEY_SIZE       (16u)

/*******************************************************************************
 *                           Global Variables
 *******************************************************************************/
/* Status variable for connection ID */
extern uint16_t app_bt_conn_id;

/*******************************************************************************
 *                           Function Prototypes
 *******************************************************************************/
/* Callback function for Bluetooth stack management type events */
wiced_bt_dev_status_t
app_bt_event_management_callback(wiced_bt_management_evt_t event,
                           wiced_bt_management_evt_data_t *p_event_data);

#endif // __APP_BT_TASK_H__