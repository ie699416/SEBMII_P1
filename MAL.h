/*
 * MAL.h
 *
 *  Created on: Apr 9, 2018
 *      Author: Cursos
 */
#include "definitions.h"


void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void *userData);

void UART1_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
		void *userData);

void UART0_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
		void *userData) ;


/**********************************************************************************************************************
 * UART Tasks
 *********************************************************************************************************************/

/*!
 * @brief Each second the LCD refresh the hour displayed.
 * @param void pointer for input parameters
 */

void UART1_init_task(void * arg);
/*!
 * @brief Each second the LCD refresh the hour displayed.
 * @param void pointer for input parameters
 */

void UART0_init_task(void * arg);

/*!
 * @brief Each second the LCD refresh the hour displayed.
 * @param void pointer for input parameters
 */

void UART1_PrintEcho_task(void * arg);

/*!
 * @brief Each second the LCD refresh the hour displayed.
 * @param void pointer for input parameters
 */

void UART0_PrintEcho_task(void * arg);

/*!
 * @brief Each second the LCD refresh the hour displayed.
 * @param void pointer for input parameters
 */

void UART0_putString(uint8_t * dataToSend);

/*!
 * @brief Each second the LCD refresh the hour displayed.
 * @param void pointer for input parameters
 */

void UART1_putString(uint8_t * dataToSend);

/**********************************************************************************************************************
 * I2C Tasks
 *********************************************************************************************************************/

void I2C_init_task();
