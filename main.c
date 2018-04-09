/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    SEBMII_P1.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "fsl_port.h"
#include "fsl_uart.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"

#define QUEUE_LENGTH_UART 1
#define QUEUE_ITEM_SIZE sizeof(uint8_t)

#define EVENT_UART0_TX (1<<0)
#define EVENT_UART0_RX (1<<1)

#define EVENT_UART1_TX (1<<2)
#define EVENT_UART1_RX (1<<3)

#define EVENT_UART0_START_RX (1<<4)
#define EVENT_UART0_START_TX (1<<5)

#define EVENT_UART1_START_RX (1<<6)
#define EVENT_UART1_START_TX (1<<7)

#define EVENT_UART0_ECHO (1<<8)
#define EVENT_UART1_ECHO (1<<9)

/****************************************************************************************************************/
/*	Local types handles*/

SemaphoreHandle_t mutex_TxRx;

QueueHandle_t g_UART1_mailbox;
QueueHandle_t g_UART0_mailbox;

uart_handle_t g_uart0Handle;
uart_handle_t g_uart1Handle;

EventGroupHandle_t g_UART_events;

/****************************************************************************************************************/
/*	UART Callback*/

void UART1_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
		void *userData) {
	BaseType_t xHigherPriorityTaskWoken;
	BaseType_t xResult;

	xHigherPriorityTaskWoken = pdFALSE;
	xResult = pdFAIL;
	if (kStatus_UART_TxIdle == status) {
		xResult = xEventGroupSetBitsFromISR(g_UART_events,
		EVENT_UART1_TX, &xHigherPriorityTaskWoken);
	}
	if (kStatus_UART_RxIdle == status) {
		xResult = xEventGroupSetBitsFromISR(g_UART_events, EVENT_UART1_RX,
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
		xResult = xEventGroupSetBitsFromISR(g_UART_events, EVENT_UART0_TX, &xHigherPriorityTaskWoken);
	}
	if (kStatus_UART_RxIdle == status) {
		xResult = xEventGroupSetBitsFromISR(g_UART_events,
		EVENT_UART0_RX, &xHigherPriorityTaskWoken);
	}

	if (pdFAIL != xResult) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/****************************************************************************************************************/
/*	Init Tasks*/

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
	user_config.baudRate_Bps = 38400;
	user_config.enableTx = true;
	user_config.enableRx = true;

	UART_TransferCreateHandle(UART1, &g_uart1Handle, UART1_UserCallback,
	NULL);

	UART_Init(UART1, &user_config, CLOCK_GetFreq(UART1_CLK_SRC));
	vTaskDelete(NULL);

}

void UART0_init_task(void * arg) {

	CLOCK_EnableClock(kCLOCK_Uart0);
	uart_config_t user_config;
	UART_GetDefaultConfig(&user_config);
	user_config.baudRate_Bps = 38400;
	user_config.enableTx = true;
	user_config.enableRx = true;

	UART_TransferCreateHandle(UART0, &g_uart0Handle, UART0_UserCallback,
	NULL);

	UART_Init(UART0, &user_config, CLOCK_GetFreq(UART0_CLK_SRC));
	vTaskDelete(NULL);

}

void UART0_transfer_task(void * arg) {

	uart_transfer_t sendXfer0;
	uint8_t sendData;
	for (;;) {

		xEventGroupWaitBits(g_UART_events, EVENT_UART0_START_TX, pdTRUE,
		pdFALSE, portMAX_DELAY); // wait fir the the callback flag

		xQueueReceive(g_UART0_mailbox, &sendData, portMAX_DELAY);

		sendXfer0.data = &sendData;
		sendXfer0.dataSize = sizeof(sendData);

		UART_TransferSendNonBlocking(UART0, &g_uart0Handle, &sendXfer0);
		// Wait send finished.

		xEventGroupWaitBits(g_UART_events, EVENT_UART0_TX, pdTRUE, pdFALSE,
		portMAX_DELAY); // wait fir the the callback flag

	}
}

void UART1_transfer_task(void * arg) {

	uart_transfer_t sendXfer0;
	uint8_t sendData;
	for (;;) {

		xEventGroupWaitBits(g_UART_events, EVENT_UART1_START_TX, pdTRUE,
		pdFALSE, portMAX_DELAY); // wait fir the the callback flag

		xQueueReceive(g_UART1_mailbox, &sendData, portMAX_DELAY);

		sendXfer0.data = &sendData;
		sendXfer0.dataSize = sizeof(sendData);

		UART_TransferSendNonBlocking(UART1, &g_uart1Handle, &sendXfer0);
		// Wait send finished.

		xEventGroupWaitBits(g_UART_events, EVENT_UART1_TX, pdTRUE, pdFALSE,
		portMAX_DELAY); // wait fir the the callback flag

	}
}

void UART1_receive_task(void * arg) {

	uart_transfer_t receiveXfer;
	uint8_t receiveData;
	xEventGroupSetBits(g_UART_events, EVENT_UART1_START_RX);

	for (;;) {

		receiveXfer.data = &receiveData;
		receiveXfer.dataSize = sizeof(receiveData);

		if (EVENT_UART1_START_RX & xEventGroupGetBits(g_UART_events)) {
			xEventGroupClearBits(g_UART_events, EVENT_UART1_START_RX);
			UART_TransferReceiveNonBlocking(UART1, &g_uart1Handle, &receiveXfer,
			NULL);
		}

		if (EVENT_UART1_RX & xEventGroupGetBits(g_UART_events)) {
			xEventGroupClearBits(g_UART_events, EVENT_UART1_RX);
			xEventGroupSetBits(g_UART_events, EVENT_UART1_START_RX);
			xQueueSendToBack(g_UART0_mailbox, receiveXfer.data, 10);
			xEventGroupSetBits(g_UART_events, EVENT_UART0_START_TX);
		}

		taskYIELD()
		;
	}
}

void UART0_receive_task(void * arg) {

	uart_transfer_t receiveXfer;
	uint8_t receiveData;
	xEventGroupSetBits(g_UART_events, EVENT_UART0_START_RX);

	for (;;) {

		receiveXfer.data = &receiveData;
		receiveXfer.dataSize = sizeof(receiveData);

		if (EVENT_UART0_START_RX & xEventGroupGetBits(g_UART_events)) {
			xEventGroupClearBits(g_UART_events, EVENT_UART0_START_RX);
			UART_TransferReceiveNonBlocking(UART0, &g_uart0Handle, &receiveXfer,
			NULL);
		}

		if (EVENT_UART0_RX & xEventGroupGetBits(g_UART_events)) {
			xEventGroupClearBits(g_UART_events, EVENT_UART0_RX);
			xEventGroupSetBits(g_UART_events, EVENT_UART0_START_RX);
			xQueueSendToBack(g_UART1_mailbox, receiveXfer.data, 10);
			xEventGroupSetBits(g_UART_events, EVENT_UART1_START_TX);
		}

		taskYIELD()
		;
	}
}

/****************************************************************************************************************/
/*	Main */

int main(void) {

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	mutex_TxRx = xSemaphoreCreateMutex();

	g_UART_events = xEventGroupCreate();

	g_UART0_mailbox = xQueueCreate(QUEUE_LENGTH_UART, QUEUE_ITEM_SIZE);

	g_UART1_mailbox = xQueueCreate(QUEUE_LENGTH_UART, QUEUE_ITEM_SIZE);

	xTaskCreate(UART1_init_task, "UART1_init", 200, NULL,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(UART0_init_task, "UART0 init", 200, NULL,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(UART1_receive_task, "TERM 1 Receive", 200, NULL,
	configMAX_PRIORITIES - 4, NULL);

	xTaskCreate(UART0_receive_task, "TERM 0 Receive", 200, NULL,
	configMAX_PRIORITIES - 4, NULL);

	xTaskCreate(UART0_transfer_task, "TERM 0 Transfer", 200, NULL,
	configMAX_PRIORITIES - 3, NULL);

	xTaskCreate(UART1_transfer_task, "TERM 1 Transfer", 200, NULL,
	configMAX_PRIORITIES - 3, NULL);

	NVIC_EnableIRQ(UART0_RX_TX_IRQn);
	NVIC_EnableIRQ(UART1_RX_TX_IRQn);
	NVIC_SetPriority(UART0_RX_TX_IRQn, 5);
	NVIC_SetPriority(UART1_RX_TX_IRQn, 5);

	vTaskStartScheduler();

	while (1) {
	}
	return 0;
}
