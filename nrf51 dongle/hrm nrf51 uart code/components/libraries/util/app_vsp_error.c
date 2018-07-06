/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 *
 * @defgroup app_error Common application error handler
 * @{
 * @ingroup app_common
 *
 * @brief Common application error handler.
 */

#include "nrf.h"
#include "app_error.h"
#include "compiler_abstraction.h"
#include "nordic_common.h"
#ifdef DEBUG
#include "bsp.h"

/* global error variables - in order to prevent removal by optimizers */
uint32_t m_error_code;
uint32_t m_line_num;
const uint8_t * m_p_file_name;
#endif

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 *
 * Function is implemented as weak so that it can be overwritten by custom application error handler
 * when needed.
 */

/*lint -save -e14 */
//__WEAK void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
//{
//    // On assert, the system can only recover with a reset.
//#ifndef DEBUG
//    NVIC_SystemReset();
//#else

//	nrf_gpio_pin_clear(HEART_LED);	

//    // The following variable helps Keil keep the call stack visible, in addition, it can be set to
//    // 0 in the debugger to continue executing code after the error check.
//    volatile bool loop = true;
//    UNUSED_VARIABLE(loop);

//    m_error_code = error_code;
//    m_line_num = line_num;
//    m_p_file_name = p_file_name;

//    UNUSED_VARIABLE(m_error_code);
//    UNUSED_VARIABLE(m_line_num);
//    UNUSED_VARIABLE(m_p_file_name);
//    __disable_irq();

//	do{
//		nrf_gpio_pin_set(HEART_LED);	
//		nrf_gpio_pin_clear(HEART_LED);	
//  }  while(loop);
//#endif // DEBUG
//}
/*lint -restore */
