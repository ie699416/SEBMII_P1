/*
 * MAL.c
 *
 *  Created on: Apr 9, 2018
 *      Author: Cursos
 */
#include "MAL.h"


void UART1_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
		void *userData) {
	BaseType_t xHigherPriorityTaskWoken;
	BaseType_t xResult;

	xHigherPriorityTaskWoken = pdFALSE;
	xResult = pdFAIL;
	if (kStatus_UART_TxIdle == status) {
		xResult = xEventGroupSetBitsFromISR(get_g_UART_events(), EVENT_UART1_TX,
				&xHigherPriorityTaskWoken);
	}
	if (kStatus_UART_RxIdle == status) {
		xResult = xEventGroupSetBitsFromISR(get_g_UART_events(), EVENT_UART1_RX,
				&xHigherPriorityTaskWoken);
	}

	if (pdFAIL != xResult) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void UART0_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
		void *userData) {
	BaseType_t xHigherPriorityTaskWoken;
	BaseType_t xResult;

	xHigherPriorityTaskWoken = pdFALSE;
	xResult = pdFAIL;
	if (kStatus_UART_TxIdle == status) {
		xResult = xEventGroupSetBitsFromISR(get_g_UART_events(), EVENT_UART0_TX,
				&xHigherPriorityTaskWoken);
	}
	if (kStatus_UART_RxIdle == status) {
		xResult = xEventGroupSetBitsFromISR(get_g_UART_events(), EVENT_UART0_RX,
				&xHigherPriorityTaskWoken);
	}

	if (pdFAIL != xResult) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}


/**********************************************************************************************************************
 * UART Tasks
 *********************************************************************************************************************/

void UART1_init_task(void * arg) {

	CLOCK_EnableClock(kCLOCK_PortC);

	port_pin_config_t config_uart = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAlt3, kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTC, 4, &config_uart);
	PORT_SetPinConfig(PORTC, 3, &config_uart);

	CLOCK_EnableClock(kCLOCK_Uart1);
	uart_config_t user_config;
	UART_GetDefaultConfig(&user_config);
	user_config.baudRate_Bps = 9600;
	user_config.enableTx = true;
	user_config.enableRx = true;

	UART_TransferCreateHandle(UART1, get_g_uart1Handle(), UART1_UserCallback,
	NULL);

	UART_Init(UART1, &user_config, CLOCK_GetFreq(UART1_CLK_SRC));
	vTaskDelete(NULL);

}

void UART0_init_task(void * arg) {

	CLOCK_EnableClock(kCLOCK_Uart0);
	uart_config_t user_config;
	UART_GetDefaultConfig(&user_config);
	user_config.baudRate_Bps = 115200;
	user_config.enableTx = true;
	user_config.enableRx = true;

	UART_TransferCreateHandle(UART0, get_g_uart0Handle(), UART0_UserCallback,
	NULL);

	UART_Init(UART0, &user_config, CLOCK_GetFreq(UART0_CLK_SRC));
	vTaskDelete(NULL);

}

/****************************************************************************************************************/
/*	PRINT Tasks*/

void UART0_PrintEcho_task(void * arg) {

	uart_transfer_t receiveXfer;
	uart_transfer_t sendXfer0;
	uint8_t receiveData;

	receiveXfer.data = &receiveData;
	receiveXfer.dataSize = sizeof(receiveData);
	sendXfer0.dataSize = sizeof(receiveData);

	xEventGroupSetBits(get_g_TERM0_events(), EVENT_UART_RX);
	xEventGroupSetBits(get_g_TERM0_events(), EVENT_UART_ECHO);

	for (;;) {

		if (EVENT_UART0_RX & xEventGroupGetBits(get_g_UART_events())) {
			xEventGroupClearBits(get_g_UART_events(), EVENT_UART0_RX);

			xEventGroupSetBits(get_g_TERM0_events(), EVENT_UART_RX);

			if (ESCAPE_KEY != receiveXfer.data[0]) {
				if ((receiveXfer.data[0] >= '0' && receiveXfer.data[0] <= '9')

				|| (receiveXfer.data[0] >= 'A' && receiveXfer.data[0] <= 'F')
						|| (receiveXfer.data[0] >= 'a'
								&& receiveXfer.data[0] <= 'f')) {

					xEventGroupSetBits(get_g_TERM0_events(),
					EVENT_INVALID_CHAR);
				} else {
					xEventGroupClearBits(get_g_TERM0_events(),
					EVENT_INVALID_CHAR);
				}
			} else {
				xEventGroupSetBits(get_g_TERM0_events(), EVENT_MENU_WAIT);
			}

			receiveXfer.data[0] = toUpperCase(receiveXfer.data);

			if (EVENT_EEPROM_GET_ADDR & xEventGroupGetBits(get_g_TERM0_events())) {

				xQueueSendToBack(get_g_TERM0_EEPROM_address(), receiveXfer.data,
						10);
			}

			if (EVENT_MENU_WAIT & xEventGroupGetBits(get_g_TERM0_events())) {
				xEventGroupClearBits(get_g_TERM0_events(), EVENT_MENU_WAIT);
				xQueueSendToBack(get_g_UART0_mailbox(), receiveXfer.data, 10);
				xEventGroupSetBits(get_g_TERM0_events(), EVENT_CHAR_SENT);
			}

			if (EVENT_UART_ECHO & xEventGroupGetBits(get_g_TERM0_events())) {

				sendXfer0.data = receiveXfer.data;
				UART_TransferSendNonBlocking(UART0, get_g_uart0Handle(),
						&sendXfer0);
				// Wait send finished.
				xEventGroupWaitBits(get_g_UART_events(), EVENT_UART0_TX, pdTRUE,
				pdFALSE, portMAX_DELAY); // wait fir the the callback flag
			}
		}

		if (EVENT_UART_RX & xEventGroupGetBits(get_g_TERM0_events())) {
			xEventGroupClearBits(get_g_TERM0_events(), EVENT_UART_RX);

			UART_TransferReceiveNonBlocking(UART0, get_g_uart0Handle(),
					&receiveXfer,
					NULL);

		}

		taskYIELD()
		;

	}
}

void UART1_PrintEcho_task(void * arg) {

	uart_transfer_t receiveXfer;
	uart_transfer_t sendXfer0;
	uint8_t receiveData;

	receiveXfer.data = &receiveData;
	receiveXfer.dataSize = sizeof(receiveData);
	sendXfer0.dataSize = sizeof(receiveData);

	xEventGroupSetBits(get_g_TERM1_events(), EVENT_UART_RX);
	xEventGroupSetBits(get_g_TERM1_events(), EVENT_UART_ECHO);

	for (;;) {

		if (EVENT_UART1_RX & xEventGroupGetBits(get_g_UART_events())) {
			xEventGroupClearBits(get_g_UART_events(), EVENT_UART1_RX);

			xEventGroupSetBits(get_g_TERM1_events(), EVENT_UART_RX);

			if (ESCAPE_KEY != receiveXfer.data[0]) {
				if ((receiveXfer.data[0] >= '0' && receiveXfer.data[0] <= '9')

				|| (receiveXfer.data[0] >= 'A' && receiveXfer.data[0] <= 'F')
						|| (receiveXfer.data[0] >= 'a'
								&& receiveXfer.data[0] <= 'f')) {

					xEventGroupSetBits(get_g_TERM1_events(),
					EVENT_INVALID_CHAR);
				} else {
					xEventGroupClearBits(get_g_TERM1_events(),
					EVENT_INVALID_CHAR);
				}
			} else {
				xEventGroupSetBits(get_g_TERM1_events(), EVENT_MENU_WAIT);
			}

			receiveXfer.data[0] = toUpperCase(receiveXfer.data);

			if ( EVENT_EEPROM_GET_ADDR
					& xEventGroupGetBits(get_g_TERM1_events())) {

				xQueueSendToBack(get_g_TERM1_EEPROM_address(), receiveXfer.data,
						10);
			}

			if (EVENT_MENU_WAIT & xEventGroupGetBits(get_g_TERM1_events())) {
				xEventGroupClearBits(get_g_TERM1_events(), EVENT_MENU_WAIT);
				xQueueSendToBack(get_g_UART1_mailbox(), receiveXfer.data, 10);
				xEventGroupSetBits(get_g_TERM1_events(), EVENT_CHAR_SENT);
			}

			if ( EVENT_UART_ECHO & xEventGroupGetBits(get_g_TERM1_events())) {

				sendXfer0.data = receiveXfer.data;
				UART_TransferSendNonBlocking(UART1, get_g_uart1Handle(), &sendXfer0);
				// Wait send finished.
				xEventGroupWaitBits(get_g_UART_events(), EVENT_UART1_TX, pdTRUE,
				pdFALSE, portMAX_DELAY); // wait fir the the callback flag
			}
		}

		if (EVENT_UART_RX & xEventGroupGetBits(get_g_TERM1_events())) {
			xEventGroupClearBits(get_g_TERM1_events(), EVENT_UART_RX);
			UART_TransferReceiveNonBlocking(UART1, get_g_uart1Handle(), &receiveXfer,
			NULL);

		}

		taskYIELD()
		;

	}
}

void UART0_putString(uint8_t * dataToSend) {

	uart_transfer_t sendXfer;
	volatile uint8_t index = 0;

	for (index; (dataToSend[index] != '\0'); index++) {
	}

	uint8_t sendData[index];

	for (index = 0; (dataToSend[index] != '\0'); index++) {
		sendData[index] = dataToSend[index];
	}

	// Prepare to send.
	sendXfer.data = sendData;
	sendXfer.dataSize = sizeof(sendData) / sizeof(sendData[0]);
	// Send out.

	UART_TransferSendNonBlocking(UART0, get_g_uart0Handle(), &sendXfer);
	// Wait send finished.
	xEventGroupWaitBits(get_g_UART_events(), EVENT_UART0_TX, pdTRUE, pdFALSE,
	portMAX_DELAY); // wait fir the the callback flag

}

void UART1_putString(uint8_t * dataToSend) {

	uart_transfer_t sendXfer;
	volatile uint8_t index = 0;

	for (index; (dataToSend[index] != '\0'); index++) {
	}

	uint8_t sendData[index];

	for (index = 0; (dataToSend[index] != '\0'); index++) {
		sendData[index] = dataToSend[index];
	}

	// Prepare to send.
	sendXfer.data = sendData;
	sendXfer.dataSize = sizeof(sendData) / sizeof(sendData[0]);
	// Send out.

	UART_TransferSendNonBlocking(UART1, get_g_uart1Handle(), &sendXfer);
	// Wait send finished.
	xEventGroupWaitBits(get_g_UART_events(), EVENT_UART1_TX, pdTRUE, pdFALSE,
	portMAX_DELAY); // wait fir the the callback flag
}

/**********************************************************************************************************************
 * I2C Tasks
 *********************************************************************************************************************/


void I2C_init_task() {

	CLOCK_EnableClock(kCLOCK_I2c0);
	CLOCK_EnableClock(kCLOCK_PortE);

	port_pin_config_t config_i2c = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAlt5, kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTE, 24, &config_i2c);
	PORT_SetPinConfig(PORTE, 25, &config_i2c);

	i2c_master_config_t masterConfig;

	I2C_MasterGetDefaultConfig(&masterConfig);
	I2C_MasterInit(I2C0_BASEADDR, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
	vTaskDelete(NULL);
}

