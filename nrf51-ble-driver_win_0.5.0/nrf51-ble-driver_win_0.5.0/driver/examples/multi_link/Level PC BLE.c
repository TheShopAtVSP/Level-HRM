/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <userint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "Level_PC.h" 
#include "Level PC Includes.h"

#define ADVERTISING_INTERVAL_40_MS  64 // * 0.625 ms = 40 ms
#define ADVERTISING_TIMEOUT_3_MIN   180 // * 1 sec = 3 min

#define UART_PORT_NAME "COM6"


enum
{
    UNIT_0_625_MS = 625,                                /**< Number of microseconds in 0.625 milliseconds. */
    UNIT_1_25_MS  = 1250,                               /**< Number of microseconds in 1.25 milliseconds. */
    UNIT_10_MS    = 10000                               /**< Number of microseconds in 10 milliseconds. */
};

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))

#define SCAN_INTERVAL                    0x00A0                                         /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                      0x0050                                         /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONNECTION_INTERVAL          MSEC_TO_UNITS(20, UNIT_1_25_MS)                /**< Determines maximum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL          MSEC_TO_UNITS(75, UNIT_1_25_MS)                /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY                    0                                              /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT              MSEC_TO_UNITS(4000, UNIT_10_MS)                /**< Determines supervision time-out in units of 10 millisecond. */

typedef struct
{
    uint8_t     * p_data;                                                      /**< Pointer to data. */
    uint16_t      data_len;                                                    /**< Length of data. */
}data_t;

static uint8_t number_of_connected_devices = 0;
static uint16_t m_connection_handle = 0; 

static const ble_gap_scan_params_t m_scan_param =
{
     0,                       // Active scanning set.
     0,                       // Selective scanning not set.
     NULL,                    // White-list not set.
     (uint16_t)SCAN_INTERVAL, // Scan interval.
     (uint16_t)SCAN_WINDOW,   // Scan window.
     0                        // Never stop scanning unless explicit asked to.
};

static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,   // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,   // Maximum connection
    0,                                   // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
};

void log_handler(sd_rpc_log_severity_t severity, const char * message)
{
    switch (severity)
    {
    case LOG_ERROR:
		sprintf(Console_str, "Error: %s\n", message); 
		SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
        break;

    case LOG_WARNING:
		sprintf(Console_str, "Warning: %s\n", message); 
		SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
        break;

    default:
		sprintf(Console_str, "Log: %s\n", message); 
		SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
        break;
    }
}

void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    if (p_ble_evt == NULL)
    {
		sprintf(Console_str, "Received empty ble_event\n"); 
		SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        on_evt_connected(p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_evt_disconnected(p_ble_evt);
        break;

    case BLE_GAP_EVT_ADV_REPORT:
        on_evt_adv_report(p_ble_evt);
        break;

    default:
		sprintf(Console_str, "Received event with ID: 0x%02X\n", p_ble_evt->header.evt_id); 
		SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
        break;
    }
}

uint32_t ble_stack_init()
{
    uint32_t err_code;
    ble_enable_params_t ble_enable_params;

    memset(&ble_enable_params, 0, sizeof(ble_enable_params));

    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
    ble_enable_params.gatts_enable_params.service_changed = false;

    err_code = sd_ble_enable(&ble_enable_params);

    if (err_code == NRF_SUCCESS)
    {
        sprintf(Console_str, "BLE stack initalized.\n"); 
		SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
		return err_code;
    }

    if (err_code == NRF_ERROR_INVALID_STATE)
    {
		sprintf(Console_str, "BLE stack already enabled\n"); 
		SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
        return NRF_SUCCESS;
    }

	sprintf(Console_str, "Failed to enable BLE stack.\n"); 
	SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
    return err_code;
}

static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index+2];
            p_typedata->data_len = field_length-1;
            return NRF_SUCCESS;
        }
        index += field_length+1;
    }
    return NRF_ERROR_NOT_FOUND;
}

static bool name_in_connect_list(data_t * type_data)
{
    int i = 0;
	extern char target_list[TARGET_LIST_LENGTH][TARGET_LIST_NAME_MAX_LENGTH];
	
    for (i = 0; i < TARGET_LIST_LENGTH; ++i)
    {
        if (0 == memcmp(target_list[i], type_data->p_data, type_data->data_len))
        {
            sprintf(Console_str, "Found Device %s\n", target_list[i]);
			SetCtrlVal(panelHandle, PANEL_SCAN_LIST, Console_str); 
			return true;
        }
    }

    return false;
}

static void on_evt_connected(ble_evt_t* p_ble_evt)
{
    ble_gap_evt_connected_t connected_evt = p_ble_evt->evt.gap_evt.params.connected;
	
	
	sprintf(Console_str, "Failed to enable BLE stack.\n"); 
	SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);

    sprintf(Console_str, "Connected to device 0x%X%X%X%X%X%X\n",
           connected_evt.peer_addr.addr[5],
           connected_evt.peer_addr.addr[4],
           connected_evt.peer_addr.addr[3],
           connected_evt.peer_addr.addr[2],
           connected_evt.peer_addr.addr[1],
           connected_evt.peer_addr.addr[0]); 
	SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str); 
	SetCtrlVal(panelHandle, PANEL_CONNET_LED, 1);
	
    number_of_connected_devices++;
	sprintf(Console_str, "%d devices connected.\n", number_of_connected_devices); 
	SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
    //scan_start();
	//SVCALL(SD_BLE_GAP_AUTHENTICATE, uint32_t, sd_ble_gap_authenticate(uint16_t conn_handle, ble_gap_sec_params_t const *p_sec_params));
	//sd_ble_gap_authenticate(conn_handle, ble_gap_sec_params_t  *p_sec_params));
	//service_discovery_start();
}

static void on_evt_disconnected(ble_evt_t* p_ble_evt)
{
    ble_gap_evt_disconnected_t disconnected_evt = p_ble_evt->evt.gap_evt.params.disconnected;
	sprintf(Console_str, "Disconnected, reason: %d\n", disconnected_evt.reason); 
	SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
    number_of_connected_devices--;
    printf("%d devices connected.\n", number_of_connected_devices); fflush(stdout);
	sprintf(Console_str, "%d devices connected.\n", number_of_connected_devices); 
	SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
}

static void on_evt_adv_report(ble_evt_t* p_ble_evt)
{
    ble_gap_evt_adv_report_t adv_report;
    data_t adv_data;
    data_t type_data;
    uint32_t err_code;

    //printf("Received advertisment report\n"); fflush(stdout);

    adv_report = p_ble_evt->evt.gap_evt.params.adv_report;

    adv_data.p_data = adv_report.data;
    adv_data.data_len = adv_report.dlen;

    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                &adv_data,
                                &type_data);

    if (err_code != NRF_SUCCESS)
    {
        // Compare short local name in case complete name does not match.
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
                                    &adv_data,
                                    &type_data);

        if (err_code != NRF_SUCCESS)
        {
            return;
        }
    }

    if (name_in_connect_list(&type_data))
    {
        //if (number_of_connected_devices >= MAX_PEER_COUNT)
        //{
        //    return;
        //}

        sd_ble_gap_scan_stop();

        err_code = sd_ble_gap_connect(&adv_report.peer_addr,
                                      &m_scan_param,
                                      &m_connection_param);

        if (err_code != NRF_SUCCESS)
        {
			sprintf(Console_str, "Failed to connect. Error code: 0x%02X\n", err_code); 
			SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
        }
    }
}

void scan_start(void)
{
    uint32_t error_code;
    error_code = sd_ble_gap_scan_start(&m_scan_param);
	sprintf(Console_str, "Started scan with return code: 0x%02X\n", error_code); 
	SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
}



