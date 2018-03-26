/**
 \file
 \brief
 This is a starter file to use the Nokia 5510 LCD.
 The LCD is connected as follows:
 reset-PDT0
 CE-GND
 CD-PTD3
 DIN-PTD2
 CLK-PTD1
 \author J. Luis Pizano Escalante, luispizano@iteso.mx
 \date	1/08/2015
 \todo
 The SPI device driver needs to be completed.
 */

#include "LCDNokia5110.h"
#include "LCDNokia5110Images.h"

extern const uint8_t ITESO[];

#if example_lcd
/*! This array hold the initial picture that is shown in the LCD*/
extern const uint8 ITESO[504];
#endif

void delay(uint16_t delay) {
	volatile uint16_t counter;
	for (counter = delay; counter > 0; counter--)
		;
}

void nokia_lcd_init_task(void * pvParameters) {
	GPIO_ClearPinsOutput(GPIOD, 1 << LCD_RESET_PIN);
	vTaskDelay(100);/**delay of 100ms for properly reset*/
	GPIO_SetPinsOutput(GPIOD, 1 << LCD_RESET_PIN);

	LCDNokia_init();

	for (;;) {
		LCDNokia_clear();/*! It clears the information printed in the LCD*/
		LCDNokia_bitmap(&ITESO[0]); /*! It prints an array that hold an image, in this case is the initial picture*/
		vTaskDelay(pdMS_TO_TICKS(1000));
		LCDNokia_clear();
		delay(65000);
		LCDNokia_clear();
		LCDNokia_gotoXY(5, 0); /*! It establishes the position to print the messages in the LCD*/
		LCDNokia_sendString("S I S T E M A S", 0); /*! It print a string stored in an array*/
		delay(65000);
		LCDNokia_gotoXY(0, 1);
		LCDNokia_sendString("E M B E B I D O S       II", 0); /*! It print a string stored in an array*/
		delay(65000);
		LCDNokia_gotoXY(25, 4);
		LCDNokia_sendString("2 0 1 8 ", 0); /*! It print a string stored in an array*/
		delay(65000);
		vTaskDelay(pdMS_TO_TICKS(1000));

	}

}

int main(void) {

#if example_lcd
	uint8 string1[]="ITESO"; /*! String to be printed in the LCD*/
	uint8 string2[]="uMs y DSPs"; /*! String to be printed in the LCD*/
#endif

	/**Configure control pins*/
	config_lcd_spi_pins();

	xTaskCreate(nokia_lcd_init_task, "lcd_nokia_init", 200, NULL,
	configMAX_PRIORITIES, NULL);

	NVIC_EnableIRQ(SPI0_IRQn);
	NVIC_SetPriority(SPI0_IRQn, 5);

	vTaskStartScheduler();

	for (;;) {
#if example_lcd
		LCDNokia_clear();/*! It clears the information printed in the LCD*/
		LCDNokia_bitmap(&ITESO[0]); /*! It prints an array that hold an image, in this case is the initial picture*/
		delay(65000);
		LCDNokia_clear();
		delay(65000);
		LCDNokia_clear();
		LCDNokia_gotoXY(25,0); /*! It establishes the position to print the messages in the LCD*/
		LCDNokia_sendString(string1); /*! It print a string stored in an array*/
		delay(65000);
		LCDNokia_gotoXY(10,1);
		LCDNokia_sendString(string2); /*! It print a string stored in an array*/
		delay(65000);
		LCDNokia_gotoXY(25,2);
		LCDNokia_sendChar('2'); /*! It prints a character*/
		LCDNokia_sendChar('0'); /*! It prints a character*/
		LCDNokia_sendChar('1'); /*! It prints a character*/
		LCDNokia_sendChar('5'); /*! It prints a character*/
		delay(65000);
#endif

	}

	return 0;
}

