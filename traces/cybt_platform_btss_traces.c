/*
 * cybt_platform_btss_traces.c
 *
 *  Created on:
 *      Author:
 */
#ifdef ENABLE_BTSS_TRACES
#include "cybt_result.h"
#include "cy_ipc_bt.h"
#include "cy_gpio.h"
#include "stdio.h"
#include "cybt_platform_trace.h"
#include "wiced_bt_dev.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define BTSS_TRACE_ENABLE_MORE_DEBUG        (0)
#define BCS_CONFIG_POS(pos)                 ((0x1ULL) << (pos))

#define GPIO_OP_CH                          (1)
#define SPI_OP_CH                           (2)
#define GPIO_AND_SPI_OP_CH                  (GPIO_OP_CH | SPI_OP_CH)

#define HCI_DEBUG_MODE_IO_MUX               ((uint8_t)0x50)

#define HCI_BRCM_BCS_DEBUG_CONFIG           (0x0004 | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_BRCM_BCS_DEBUG_CONFIG_LENGTH    (9)

#define HCI_BTSS_CFG_DEBUG_LINES_LEN        (9)
#define HCI_BTSS_CFG_DEBUG_LINES            (0x0015 | HCI_GRP_VENDOR_SPECIFIC)

#define cr_pds_v2_pad_dbg_ctrl_0_adr        (0x4f04005c)
#define MAC_DEBUG_CFG_VAL                   (0x00010002)     /* [3:0] --> mac_debug, [16:23] --> lines */

enum
{
    BTSS_CFG_DEBUG_LINES_READ  = 0,
    BTSS_CFG_DEBUG_LINES_WRITE = 1,
};

enum
{
    // IMPORTANT: MUST FOLLOW FORMAT EXACTLY, OTHERWISE THE SCRIPT WILL FAIL!
    // SEE NOTES BELOW...
    // The first so many modules are reserved for core scheduler modules
    // NOTE: The BCS_MODULE_DEBUG module below is not meant to be used in BCS_LOG* macros.
    //       Use BCS_DEBUG_LOG* macros for personal debug instead.

    // NOTE: the word following "//" IS IMPORTANT. That is the prefix used
    // before the log in Saleae Logic.  The log appears as "PREFIX: ..."
    // As a convention, it MUST be: (a) SHORT, (b) unique, and
    // (c) in ALLCAPS with no separators
    BCS_MODULE_DEBUG            = 0,    // DBG
    BCS_MODULE_ALERTS           = 1,    // ALERT
    BCS_MODULE_ERROR_CHECK      = 2,    // ERR
    BCS_MODULE_SCHEDULER        = 3,    // BCS
    BCS_MODULE_CLOCK_UTIL       = 4,    // CLK
    BCS_MODULE_TASK_API         = 5,    // TASK
    BCS_MODULE_FREQ             = 6,    // FREQ
    BCS_MODULE_TIMING           = 7,    // TIME
    BCS_MODULE_RESYNC           = 8,    // RESYNC
    BCS_MODULE_HW_SCHEDULER     = 9,    // HWSCH
    BCS_MODULE_HW_EVENTS        = 10,   // HWEVT
    BCS_MODULE_RSSI             = 11,   // RSSI
    BCS_MODULE_COEX             = 12,   // COEX

    BCS_MODULE_PROFILER         = 13,   // PROF
    BCS_MODULE_BCS_THREAD       = 14,   // BTHR
    BCS_MODULE_LRM              = 15,   // LRM
    BCS_MODULE_SYNC_TASK        = 16,   // SYNC

    // The modules are for protocol/task implementation
    BCS_MODULE_HCI              = 17,   // HCI
    BCS_MODULE_BT_SCAN          = 18,   // BTSCAN
    BCS_MODULE_LE_SCAN          = 19,   // LESCAN
    BCS_MODULE_PAGE_INQ         = 20,   // INQ
    BCS_MODULE_LE_INIT          = 21,   // LEINIT
    BCS_MODULE_ACL              = 22,   // BTACL
    BCS_MODULE_LE_ADV           = 23,   // LEADV
    BCS_MODULE_ACL_QOS          = 24,   // BTQOS
    BCS_MODULE_LE_CONN          = 25,   // LECONN
    BCS_MODULE_LMP              = 26,   // LMP
    BCS_MODULE_LE_LCP           = 27,   // LCP
    BCS_MODULE_DHM              = 28,   // DHM
    BCS_MODULE_SCO              = 29,   // SCO
    BCS_MODULE_SNIFF            = 30,   // SNF
    BCS_MODULE_APIPE            = 31,   // APIPE
    BCS_MODULE_ISOC             = 32,   // ISOC
    BCS_MODULE_THREADX          = 33,   // THRDX
    BCS_MODULE_LE_POWER         = 34,   // LE_POWER
    BCS_MODULE_ISOAL            = 35,   // ISOAL
    BCS_MODULE_HADM             = 36,   // HADM
    BCS_MODULE_DF               = 37,   // DF

    // Verbose BCSV3 logs.  Meant only for the short-term...
    // TODO: long term: clean this up and merge with existing modules
    BCS_MODULE_BCSV3            = 38,   // BCSV3
    BCS_MODULE_RFM              = 39,   // RFM
    BCS_MODULE_PMU              = 40,   // PMUB
    BCS_MODULE_AFH              = 41,   // AFH
    BCS_MODULE_DYNAMIC_MEMORY   = 42,   // DYNMEM
    BCS_MODULE_HWRPA_ACC        = 43,   // RPAHW
    BCS_MODULE_PMU_CORE         = 44,   // PMUC
    BCS_MODULE_TIMER_CALLBACK   = 45,   // TIMER
    BCS_MODULE_PAWR             = 46,   // PAWR
    // Add new modules here... and read the notes!

    // NOTE 1: _MUST_ follow "FLAG = <number>[,] // SHORTNAME" format, otherwise the perl script will fail.
    //         SHORTNAME can be empty.
    // NOTE 2: These modules MUST be BCS_MODULE_<SOMETHING>, otherwise the perl script will fail.
    // NOTE 3: Decimal numbers only
    // NOTE 4: Add only in the end
    // NOTE 5: AT MOST 64 MODULES!!
    BCS_MODULE_LAST             = 63
};

/*****************************************************************************
 *                           Type Definitions
 *****************************************************************************/

typedef __PACKED_STRUCT cy_stc_hci_mcu_vsc_cmd_t
{
    BTHCI_CMD_HDR_t       hdr;
    uint8_t               mode;
}cy_stc_hci_mcu_vsc_cmd_t;

/*debug IO mux*/
typedef __PACKED_STRUCT cy_stc_hci_iocfg_t
{
    uint8_t               port;
    uint8_t               pin;
    uint8_t               hsiom_level;
    uint8_t               drivemode;
}cy_stc_hci_iocfg_t;

typedef __PACKED_STRUCT my_cmd_buff_t
{
    cy_stc_hci_mcu_vsc_cmd_t vsc_cmd;
    cy_stc_hci_iocfg_t       iocfg;
}my_cmd_buff_t;

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/

const GPIO_PRT_Type *cy_bt_io_port[] = {GPIO_PRT0, GPIO_PRT1, GPIO_PRT2, GPIO_PRT3, GPIO_PRT4, GPIO_PRT5};
const uint8_t resp_mcu_vsc_success[7] = {0x04,0x0E,0x04,0x01,0xE7,0xFD,0x00};
const uint8_t resp_mcu_vsc_fail[7]    = {0x04,0x0E,0x04,0x01,0xE7,0xFD,0x12};

/******************************************************************************
 *                           Function Definitions
 ******************************************************************************/

/*Does the requested mcu gpio pin setup*/
static cybt_result_t cybt_hci_vsc_io_init(cy_stc_hci_mcu_vsc_cmd_t *cmd, cy_stc_gpio_pin_config_t *cy_bt_pin_config)
{
    cy_stc_hci_iocfg_t *iocfg;
    iocfg = (cy_stc_hci_iocfg_t *)(((uint8_t*)cmd) + sizeof(cy_stc_hci_mcu_vsc_cmd_t));

    MAIN_TRACE_DEBUG("%s entry", __FUNCTION__);

    if((iocfg->port < 6) && (iocfg->pin < 8))
    {
        /* Need set pull_down and driver full for ellisys IO probe capture */
        cy_bt_pin_config->vtrip = CY_GPIO_VTRIP_CMOS;
        cy_bt_pin_config->driveMode = CY_GPIO_DM_PULLDOWN_IN_OFF;
        cy_bt_pin_config->slewRate = CY_GPIO_SLEW_FAST;
        cy_bt_pin_config->driveSel = CY_GPIO_DRIVE_FULL;
        cy_bt_pin_config->vregEn = 0UL;
        cy_bt_pin_config->ibufMode = 0UL;
        cy_bt_pin_config->vtripSel = 0UL;
        cy_bt_pin_config->vrefSel = 0UL;
        cy_bt_pin_config->vohSel = 0UL;
        cy_bt_pin_config->nonSec = 1UL;

        if(CY_GPIO_SUCCESS == Cy_GPIO_Pin_Init((GPIO_PRT_Type*)cy_bt_io_port[iocfg->port], (uint32_t)iocfg->pin, cy_bt_pin_config))
        {
            MAIN_TRACE_DEBUG("%s SUCCESS", __FUNCTION__);
            return CYBT_SUCCESS;
        }
    }
    return CYBT_ERR_BADARG;
}

static cybt_result_t cybt_hci_process_vsc(BTHCI_CMD_HDR_t *cmd)
{
    cybt_result_t result = CYBT_SUCCESS;
    cy_stc_hci_mcu_vsc_cmd_t *vsc_cmd = (cy_stc_hci_mcu_vsc_cmd_t *)cmd;

    MAIN_TRACE_DEBUG("%s %d 0x%04x %d\n", __FUNCTION__, vsc_cmd->hdr.params_len, vsc_cmd->hdr.opcode, vsc_cmd->mode);

    if (vsc_cmd->hdr.params_len == 0)
    {
        return result;
    }

    if(HCI_DEBUG_MODE_IO_MUX == vsc_cmd->mode)
    {
        if(vsc_cmd->hdr.params_len != 5)
        {
            return CYBT_SUCCESS;
        }

        cy_stc_gpio_pin_config_t cy_bt_io_pin_config;
        cy_stc_hci_iocfg_t *iocfg;
        iocfg = (cy_stc_hci_iocfg_t *)(((uint8_t*)cmd) + sizeof(cy_stc_hci_mcu_vsc_cmd_t));

        MAIN_TRACE_DEBUG("%s %d %d %d %d\n", __FUNCTION__, iocfg->port, iocfg->pin, iocfg->hsiom_level, iocfg->drivemode);

        cy_bt_io_pin_config.outVal = 0UL;
        cy_bt_io_pin_config.intEdge = CY_GPIO_INTR_DISABLE;
        cy_bt_io_pin_config.intMask = 0UL;
        cy_bt_io_pin_config.hsiom = (en_hsiom_sel_t)iocfg->hsiom_level;
        cy_bt_io_pin_config.driveMode = (uint32_t)iocfg->drivemode;

        if(CYBT_SUCCESS == cybt_hci_vsc_io_init(vsc_cmd, &cy_bt_io_pin_config))
        {
            MAIN_TRACE_DEBUG("%s SUCCESS", __FUNCTION__);
        }
        else
        {
            MAIN_TRACE_ERROR("%s FAILED", __FUNCTION__);
        }
    }

    return result;
}

static void Send_BTM_SetPin(uint8_t port, uint8_t pin, uint8_t mux_val, uint8_t drive_mode)
{
    // opcode param_len 0x50 port pin mux_val drive mode
    // 0xFDE7   0x05    0x50 port pin mux_val drive mode

    my_cmd_buff_t my_cmd_buff;

    my_cmd_buff.vsc_cmd.hdr.opcode = 0xFDE7;
    my_cmd_buff.vsc_cmd.hdr.params_len = 0x05;
    my_cmd_buff.vsc_cmd.mode = 0x50;
    my_cmd_buff.iocfg.port = port;
    my_cmd_buff.iocfg.pin = pin;
    my_cmd_buff.iocfg.hsiom_level = mux_val;
    my_cmd_buff.iocfg.drivemode = drive_mode;

    cybt_hci_process_vsc((BTHCI_CMD_HDR_t *) &my_cmd_buff);
}

static void bcs_debug_cfg_cmd_complete_cb(
        wiced_bt_dev_vendor_specific_command_complete_params_t *p_command_complete_params)
{
    uint8_t buf[13] = {0};
    MAIN_TRACE_DEBUG("HCI complete callback: opcode:%04x data_len:%d\n",
            p_command_complete_params->opcode, p_command_complete_params->param_len);

    buf[0] = (uint8_t)p_command_complete_params->opcode;
    buf[1] = (uint8_t)(p_command_complete_params->opcode >> 8);
    buf[2] = (uint8_t)p_command_complete_params->param_len;
    memcpy(&buf[3],
            p_command_complete_params->p_param_buf,
            (p_command_complete_params->param_len < 10) ? p_command_complete_params->param_len : 10);

    if(0 != buf[3])
    {
        MAIN_TRACE_ERROR("BCS DEBUG CONFIG status:%x\n", buf[3]);
        return;
    }

    MAIN_TRACE_DEBUG("BCS DEBUG CONFIG [SUCCESS]\n");
}

static void Send_BTM_SetBcsDebugConfig(uint8_t op_channel, uint64_t config)
{
    uint8_t buf[HCI_BRCM_BCS_DEBUG_CONFIG_LENGTH] = {0};
    uint8_t *p = &buf[0];
    uint32_t high_end = (0xFFFFFFFFULL & (config>>32));
    uint32_t low_end = (0xFFFFFFFFULL & config);

    MAIN_TRACE_DEBUG("BCS DEBUG CONFIG OUT:%d, CFG:[32-63]:%X [0-31]:%X\n", op_channel, high_end, low_end);
    UINT8_TO_STREAM(p, op_channel);
    UINT64_TO_STREAM(p, config);
    wiced_bt_dev_vendor_specific_command(HCI_BRCM_BCS_DEBUG_CONFIG,
            HCI_BRCM_BCS_DEBUG_CONFIG_LENGTH,
            buf,
            bcs_debug_cfg_cmd_complete_cb);
}

#if BTSS_TRACE_ENABLE_MORE_DEBUG /* NOTE: should be done by debug firmware */
static void btss_debug_cfg_cmd_complete_cb(
        wiced_bt_dev_vendor_specific_command_complete_params_t *p_command_complete_params)
{
    uint8_t buf[13] = {0};
    MAIN_TRACE_DEBUG("HCI complete callback: opcode:%04x data_len:%d\n",
            p_command_complete_params->opcode, p_command_complete_params->param_len);

    buf[0] = (uint8_t)p_command_complete_params->opcode;
    buf[1] = (uint8_t)(p_command_complete_params->opcode >> 8);
    buf[2] = (uint8_t)p_command_complete_params->param_len;
    memcpy(&buf[3],
            p_command_complete_params->p_param_buf,
            (p_command_complete_params->param_len < 10) ? p_command_complete_params->param_len : 10);

    if(0 != buf[3])
    {
        MAIN_TRACE_ERROR("BTSS Config debug lines VSC status:%x\n", buf[3]);
        return;
    }

    MAIN_TRACE_DEBUG("BTSS Config debug lines VSC [SUCCESS]\n");
}

static void Send_BTM_btssConfigDebugLines(uint8_t access_type, uint32_t addr, uint32_t data)
{
    uint8_t buf[HCI_BTSS_CFG_DEBUG_LINES_LEN] = {0};
    uint8_t *p = &buf[0];

    MAIN_TRACE_DEBUG("BTSS Config debug lines VSC: access_type:%d, addr:%X, data:%X\n", access_type, addr, data);
    UINT8_TO_STREAM(p, access_type);
    UINT32_TO_STREAM(p, addr);
    UINT32_TO_STREAM(p, data);
    wiced_bt_dev_vendor_specific_command(HCI_BTSS_CFG_DEBUG_LINES,
            HCI_BTSS_CFG_DEBUG_LINES_LEN,
            buf,
            btss_debug_cfg_cmd_complete_cb);
}
#endif

static void enable_bcs_traces(void)
{
    uint64_t config = (
            BCS_CONFIG_POS (BCS_MODULE_DEBUG) |
            BCS_CONFIG_POS (BCS_MODULE_ALERTS) |
            BCS_CONFIG_POS (BCS_MODULE_ERROR_CHECK) |
            BCS_CONFIG_POS (BCS_MODULE_FREQ) |
            BCS_CONFIG_POS (BCS_MODULE_TASK_API) |
            BCS_CONFIG_POS (BCS_MODULE_SCHEDULER) |
            BCS_CONFIG_POS (BCS_MODULE_HW_EVENTS) |
            BCS_CONFIG_POS (BCS_MODULE_LE_CONN) |
            BCS_CONFIG_POS (BCS_MODULE_LE_LCP) |
            0
            );

    /*
     * NOTE: for M2-EVK
     *     BCS_MOSI = P1.2 --> D5 (J4.6)
     *     BCS_CLK = P1.3  --> D4 (J4.5)
     */
    Send_BTM_SetPin(0x01, 0x02, 0x1B, 0x06);  //D4 BCS_MOSI
    Send_BTM_SetPin(0x01, 0x03, 0x1B, 0x06);  //D5 BCS_CLK

    Send_BTM_SetBcsDebugConfig(SPI_OP_CH, config);
}

static void enable_fsm_traces(void)
{
#if !BTSS_TRACE_ENABLE_MORE_DEBUG
    /* NOTE: for M2-EVK, CYW20829B0.
     * mac_debug = 0x1
     *     TX_FSM = debug[12] = P0.0 --> J2.5
     *     RX_FSM = debug[13] = P0.1 --> J2.6
     */
    Send_BTM_SetPin(0x00, 0x00, 0x1A, 0x06);
    Send_BTM_SetPin(0x00, 0x01, 0x1A, 0x06);

#else
    /* 0x000F0002: (mac_debug = 0xF)
     * REG32(0x4f04005c) = 0x000F0002; required in BTSS patch replacement
     * Send_BTM_btssConfigDebugLines already taken care of this config.
     */
    Send_BTM_SetPin(0x01, 0x04, 0x1A, 0x06);  // D6 Rx FSM
    Send_BTM_SetPin(0x05, 0x02, 0x1A, 0x06);  // A1 Tx FSM
    Send_BTM_SetPin(0x00, 0x04, 0x1A, 0x06);  // A5 Tx Sym
    Send_BTM_SetPin(0x03, 0x05, 0x1A, 0x06);  // D1 Rx Sym
#endif
}

#if BTSS_TRACE_ENABLE_MORE_DEBUG /* NOTE: should be done by debug firmware */
static void configure_btss_iomux(void)
{
    /* If we change below function parameters, we need to change
     * enable_fsm_traces() lines according to io muxing sheet
     */
    Send_BTM_btssConfigDebugLines(
            BTSS_CFG_DEBUG_LINES_WRITE,
            cr_pds_v2_pad_dbg_ctrl_0_adr,
            MAC_DEBUG_CFG_VAL);
}
#endif

void cybt_enable_btss_traces(void)
{
    /* Please don't change this order */
    enable_bcs_traces();

#if !BTSS_TRACE_ENABLE_MORE_DEBUG    /* NOTE: should be done by debug firmware */
    configure_btss_iomux();
#endif
    enable_fsm_traces();
    /* To send any MCU Command please call Send_BTM_SetPin */
    /* eg : to send MCU CMD "50 01 03 1B 06" - "50-cmd 01-port 03-pin 1b-mode 06-drive" */
    /*  Send_BTM_SetPin(0x01, 0x03, 0x1B, 0x06) */
}
#endif //ENABLE_BTSS_TRACES
