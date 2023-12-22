/*******************************************************************************
 * File Name: app_bt_periodic_adv.h
 *
 * Description: This file consists of the bt periodic advertise function declarations
 *
 ********************************************************************************
 * Copyright 2023 Cypress Semiconductor Corporation (an Infineon company)
 *******************************************************************************/

#ifndef __APP_BT_PERIODIC_ADV_H__
#define __APP_BT_PERIODIC_ADV_H__

/*******************************************************************************
 *                                INCLUDES
 ******************************************************************************/
#include "wiced_bt_ble.h"

/*******************************************************************************
 *                                Constants
 ******************************************************************************/
#define APP_BT_PERIODIC_ADV_HANDLE              (0x01)
#define APP_BT_PERIODIC_ADV_EVENT_PROP          (0x00)
#define APP_BT_PERIODIC_ADV_PRI_ADV_MIN         (800)
#define APP_BT_PERIODIC_ADV_PRI_ADV_MAX         (800)
#define APP_BT_PERIODIC_ADV_PRI_CH_MAP          (BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39)
#define APP_BT_PERIODIC_ADV_TX_POWER            (0x7F)
#define APP_BT_PERIODIC_SEC_ADV_MAX_SKIP        (0x00)
#define APP_BT_PERIODIC_ADV_SET_ID              (0x01)
#define APP_BT_PERIODIC_ADV_INT_MIN             (400)
#define APP_BT_PERIODIC_ADV_INT_MAX             (400)

#define APP_BT_PERIODIC_CHANGE_ADV_DATA         (1)

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
void app_bt_periodic_adv_init(void);

#endif      /* __APP_BT_PERIODIC_ADV_H__ */

/* [] END OF FILE */
