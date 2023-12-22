/*******************************************************************************
 * File Name: app_bt_cfg.c
 *
 * Description: This file contains Bluetooth® configuration settings for platform
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2023 Cypress Semiconductor Corporation (an Infineon company)
 *******************************************************************************/

#include "app_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"

/* BT platform configuration */
/* Configure UART for Bluetooth® HCI control */

const cybt_platform_config_t app_bt_platform_cfg_settings =
{
    .hci_config =
    {
        .hci_transport = CYBT_HCI_IPC,
    },

    .controller_config =
    {
        .bt_power_pin      = NC,
#if defined(CY_CFG_PWR_SYS_IDLE_MODE) && \
            ((CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_SLEEP) || \
            (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP))
        .sleep_mode = { .sleep_mode_enabled = CYBSP_BT_PLATFORM_CFG_SLEEP_MODE_LP_ENABLED },
#else
        .sleep_mode = { .sleep_mode_enabled = 0 },
#endif
    },

    .task_mem_pool_size    = 2048
};

/* [] END OF FILE */
