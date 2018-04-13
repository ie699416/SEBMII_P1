
/*
 * @brief   Application entry point.
 */

#include "definitions.h"
void PORTA_IRQHandler()
{
	static uint8_t state = 0;
	PORT_ClearPinsInterruptFlags(PORTA, 1<<0);
	PORT_ClearPinsInterruptFlags(PORTA, 1<<1);
	PORT_ClearPinsInterruptFlags(PORTA, 1<<2);

	GPIO_WritePinOutput(GPIOB,21,state);
	state = ( 0 == state ) ? 1 : 0;
}

int main(void) {

	/* Init board hardware. */

	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	xTaskCreate(sInitLCD_task, "LCD Nokia init", 200, NULL,
	configMAX_PRIORITIES, NULL);

	CLOCK_EnableClock(kCLOCK_PortA);
	NVIC_EnableIRQ(I2C0_IRQn);
	NVIC_EnableIRQ(SPI0_IRQn);
	NVIC_EnableIRQ(UART1_RX_TX_IRQn);
	NVIC_EnableIRQ(UART4_RX_TX_IRQn);

	NVIC_SetPriority(SPI0_IRQn, 5);
	NVIC_SetPriority(I2C0_IRQn, 5);
	NVIC_SetPriority(PORTA_IRQn, 5);
	NVIC_SetPriority(UART4_RX_TX_IRQn, 5);
	NVIC_SetPriority(UART1_RX_TX_IRQn, 5);

	port_pin_config_t config_led =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTB, 21, &config_led);


	port_pin_config_t config_switch =
	{ kPORT_PullUp, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
			kPORT_UnlockRegister};

	PORT_SetPinConfig(PORTA, 1, &config_switch);
	PORT_SetPinConfig(PORTA, 2, &config_switch);

//	    PORT_SetPinConfig(PORTA, 0, &config_switch);

//	PORT_SetPinInterruptConfig(PORTA, 0, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTA, 1, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTA, 2, kPORT_InterruptFallingEdge);




	gpio_pin_config_t led_config_gpio =
	{ kGPIO_DigitalOutput, 1 };

	GPIO_PinInit(GPIOB, 21, &led_config_gpio);

	gpio_pin_config_t switch_config_gpio =
	{ kGPIO_DigitalInput, 1 };

//	GPIO_PinInit(GPIOA, 0, &switch_config_gpio);
	GPIO_PinInit(GPIOA, 1, &switch_config_gpio);
	GPIO_PinInit(GPIOA, 2, &switch_config_gpio);

	NVIC_EnableIRQ(PORTA_IRQn);






	vTaskStartScheduler();

	while (1) {
	}
	return 0;
}
