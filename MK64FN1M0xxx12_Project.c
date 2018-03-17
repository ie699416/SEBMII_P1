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
 * @file    MK64FN1M0xxx12_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_uart.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
volatile bool txFinished;
	volatile bool rxFinished;
	void *userData;
	void UART_UserCallback(uart_handle_t *handle, status_t status,
			void *userData) {
		userData = userData;
		if (kStatus_UART_TxIdle == status) {
			txFinished = true;
		}
		if (kStatus_UART_RxIdle == status) {
			rxFinished = true;
		}
	}

	int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

	uart_handle_t g_uartHandle;
	uart_config_t user_config;
	uart_transfer_t sendXfer;
	uart_transfer_t receiveXfer;
//		uart_transfer_callback_t callback1;// it's necessary to include?

	uint8_t sendData[] = { 'H', 'e', 'l', 'l', 'o' };
	uint8_t receiveData[32];


//...
		UART_GetDefaultConfig(&user_config);
		user_config.baudRate_Bps = 115200U;
		user_config.enableTx = true;
		user_config.enableRx = true;
		UART_Init(UART1, &user_config, 120000000U);
		//UART_CreateHandle(&g_uartHandle, UART1, NULL, 0);
		UART_TransferCreateHandle(UART1, &g_uartHandle, UART_UserCallback, userData);
		//UART_SetTransferCallback(&g_uartHandle, UART_UserCallback, NULL);
// Prepare to send.
		sendXfer.data = sendData;
		sendXfer.dataSize = sizeof(sendData) / sizeof(sendData[0]);
		txFinished = false;
// Send out.
//		UART_WriteNonBlocking(UART1, sendData, sendXfer.dataSize);
		UART_TransferSendNonBlocking(UART1, &g_uartHandle, &sendXfer);
// Wait send finished.
		while (!txFinished) {
		}
// Prepare to receive.
		receiveXfer.data = receiveData;
		receiveXfer.dataSize = sizeof(receiveData) / sizeof(receiveData[0]);
		rxFinished = false;
// Receive.
//		UART_ReadNonBlocking(UART1, sendData, receiveXfer.dataSize);
		UART_TransferReceiveNonBlocking(UART1, &g_uartHandle,  &receiveXfer,(void*)(receiveXfer.dataSize));
// Wait receive finished.
		while (!rxFinished) {
		}
// ...
	}
