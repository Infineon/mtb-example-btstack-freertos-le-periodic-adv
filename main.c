/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the LE Periodic Adv Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2023 Cypress Semiconductor Corporation (an Infineon company)
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* FreeRTOS header files */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "timers.h"


#include "wiced_memory.h"
#include "wiced_bt_stack.h"

/* BT header files */
#include "cybt_platform_trace.h"
#include "app_bt_cfg.h"
#include "app_bt_event_handler.h"

#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/*******************************************************************************
* Macros
********************************************************************************/

/* Sufficient Heap size for Bluetooth activities */
#define BT_HEAP_SIZE                        (0x1000)

/*******************************************************************************
* Global Variables
********************************************************************************/


/*******************************************************************************
* Function Prototypes
********************************************************************************/

/**
 * Function Name : main
 *
 * Function Description :
 *   @brief Entry point to the application. Set device configuration and start
 *   BT stack initialization.  The actual application initialization will happen
 *   when stack reports that BT device is ready.
 *
 *   @param: None
 *
 *   @return: None
 */
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

#if (defined ENABLE_BT_SPY_LOG)
    cybt_debug_uart_config_t config = {
        .uart_tx_pin = CYBSP_DEBUG_UART_TX,
        .uart_rx_pin = CYBSP_DEBUG_UART_RX,
        .uart_cts_pin = CYBSP_BT_UART_CTS,
        .uart_rts_pin = CYBSP_BT_UART_RTS,
        .baud_rate = DEBUG_UART_BAUDRATE,
        .flow_control = TRUE};
    cybt_debug_uart_init(&config, NULL);
#else
    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
#endif

    cybt_platform_set_trace_level(CYBT_TRACE_ID_STACK, CYBT_TRACE_LEVEL_MAX);
    cybt_platform_set_trace_level(CYBT_TRACE_ID_MAIN, CYBT_TRACE_LEVEL_MAX);

    /* Initializing the HCI UART for Host control */
    cybt_platform_config_init(&app_bt_platform_cfg_settings);

    printf("\r\n****** Bluetooth LE Periodic ADV Application******\r\n ");
    printf("\r\nThis application implements Periodic ADV over BLE \r\n");

    /* Configure Bluetooth LE configuration & registers Bluetooth LE event callback function
     * with the BT stack
     */
    if (WICED_BT_SUCCESS != wiced_bt_stack_init(app_bt_event_management_callback, &wiced_bt_cfg_settings))
    {
        /* Check if stack initialization was successful */
        printf("Bluetooth Stack Initialization failed!!\r\n");
    }

    /* Create a buffer heap, make it the default heap.  */
    if ( NULL == wiced_bt_create_heap("app", NULL, BT_HEAP_SIZE, NULL, WICED_TRUE))
    {
        printf("Heap create Failed");
    }

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    printf("Scheduler exited unexpectedly\r\n");

    CY_ASSERT(0);
}


/* [] END OF FILE */
