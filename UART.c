
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
#include "fsl_debug_console.h"
#include "fsl_port.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
volatile bool txFinished;
	volatile bool rxFinished;
	void *userData;
	void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
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

    CLOCK_EnableClock(kCLOCK_PortC);
   	CLOCK_EnableClock(kCLOCK_Uart1);

    	port_pin_config_t config_uart =
    	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
    	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt3,
    	        kPORT_UnlockRegister, };

    	PORT_SetPinConfig(PORTC, 4, &config_uart);
    	PORT_SetPinConfig(PORTC, 3, &config_uart);

	uart_handle_t g_uartHandle;
	uart_config_t user_config;
	uart_transfer_t sendXfer;
	uart_transfer_t receiveXfer;
//		uart_transfer_callback_t callback1;// it's necessary to include?

	uint8_t sendData[] = { 'H', 'e', 'l', 'l', 'o' };
	uint8_t receiveData[32];


//...
		UART_GetDefaultConfig(&user_config);
		user_config.baudRate_Bps = 9600;
		user_config.enableTx = true;
		user_config.enableRx = true;
		UART_Init(UART1, &user_config, CLOCK_GetFreq(UART1_CLK_SRC));
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
		UART_TransferReceiveNonBlocking(UART1, &g_uartHandle,  &receiveXfer,NULL);
// Wait receive finished.
		while (!rxFinished) {
		}
// ...
	}
