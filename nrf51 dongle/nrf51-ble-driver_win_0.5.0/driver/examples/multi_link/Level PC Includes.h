//==============================================================================
//
// Title:		Level PC Includes.h
// Purpose:		A short description of the interface.
//
// Created on:	7/7/2016 at 3:46:48 PM by Mitch_Mason.
// Copyright:	CPI. All Rights Reserved.
//
//==============================================================================

#ifndef __IncludeFile_H__
#define __IncludeFile_H__

#ifdef __cplusplus
    extern "C" {
#endif

//==============================================================================
// Include files

#include "cvidef.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_types.h"
#include "nrf_error.h"
#include "sd_rpc.h"
		
//==============================================================================
// Constants
#define UART_PORT_NAME "COM6"
#define MAX_PEER_COUNT   1    /**< Maximum number of peer's application intends to manage. */ 
#define TARGET_LIST_LENGTH 1
#define TARGET_LIST_NAME_MAX_LENGTH 20 /**< Max length of the target names. */
		
//==============================================================================
// Types

//==============================================================================
// External variables
char	Console_str[256];
int 	panelHandle;
int		led_id_index;
//==============================================================================
// Global functions

// In Main
void intro_message_print(void);

// In BLE
void log_handler(sd_rpc_log_severity_t severity, const char * message);
void ble_evt_dispatch(ble_evt_t * p_ble_evt);
uint32_t ble_stack_init(void);
void scan_start();
void on_evt_connected(ble_evt_t* p_ble_evt);
void on_evt_disconnected(ble_evt_t* p_ble_evt);
void on_evt_adv_report(ble_evt_t* p_ble_evt);

#ifdef __cplusplus
    }
#endif

#endif  /* ndef __IncludeFile_H__ */
