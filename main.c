
/*
 * @brief   Application entry point.
 */

#include "definitions.h"

int main(void) {

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	xTaskCreate(sInitLCD_task, "LCD Nokia init", 200, NULL,
	configMAX_PRIORITIES, NULL);

	NVIC_EnableIRQ(I2C0_IRQn);
	NVIC_EnableIRQ(SPI0_IRQn);
	NVIC_EnableIRQ(UART0_RX_TX_IRQn);
	NVIC_EnableIRQ(UART1_RX_TX_IRQn);

	NVIC_SetPriority(I2C0_IRQn, 5);
	NVIC_SetPriority(SPI0_IRQn, 5);
	NVIC_SetPriority(UART0_RX_TX_IRQn, 5);
	NVIC_SetPriority(UART1_RX_TX_IRQn, 5);

	vTaskStartScheduler();

	while (1) {
	}
	return 0;
}
