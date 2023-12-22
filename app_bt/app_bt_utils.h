/*******************************************************************************
 * File Name: app_bt_utils.h
 *
 * Description: This file consists of the utility function declarations that will
 *              help debugging and developing the applications easier with much more
 *              meaningful information.
 *
 ********************************************************************************
 * Copyright 2023 Cypress Semiconductor Corporation (an Infineon company)
 *******************************************************************************/

#ifndef __APP_BT_UTILS_H__
#define __APP_BT_UTILS_H__

/*******************************************************************************
 *                                INCLUDES
 ******************************************************************************/
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"

/*******************************************************************************
 *                                Constants
 ******************************************************************************/
#define CASE_RETURN_STR(const)          case const: return #const;

#define FROM_BIT16_TO_8(val)            ( (uint8_t)( ( (val) >> 8 ) & 0xff) )

/**
 * @brief Configure the initial BD Address to be generated using
 * External flash ID/ Unique chip ID in EFuse
 * Set true to use External flash ID, set false to use EFuse
 */
#define BD_ADDRESS_USE_EXTERNAL_FLASH_ID    (true)

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
const char* app_bt_util_get_btm_event_name(wiced_bt_management_evt_t event);

const char* app_bt_util_get_btm_advert_mode_name(wiced_bt_ble_advert_mode_t mode);

const char* app_bt_util_get_gatt_disconn_reason_name(wiced_bt_gatt_disconn_reason_t reason);

const char* app_bt_util_get_gatt_status_name(wiced_bt_gatt_status_t status);

const char* app_bt_util_get_pairing_status_name(wiced_bt_smp_status_t smp_status);

void app_bt_util_print_bd_address(const wiced_bt_device_address_t bdadr);

void app_bt_util_print_byte_array(void *to_print, uint16_t len);

void app_bt_util_print_local_identity_key(char *st, wiced_bt_local_identity_keys_t *p_identity_key);

void app_bt_util_print_link_key_data(wiced_bt_device_sec_keys_t *p_key);

void app_bt_util_generate_bd_address(const uint8_t *device_addr);

#endif      /* __APP_BT_UTIS_H__ */

/* [] END OF FILE */
