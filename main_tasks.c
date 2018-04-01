/*
 * main_tasks.c
 *
 *  Created on: Mar 26, 2018
 *      Author: Cursos
 */
#include "main_tasks.h"
#include "LCDNokia5110.h"
#include "LCDNokia5110Images.h"
#include "fsl_uart.h"
#include "fsl_i2c.h"
#include "fsl_debug_console.h"
#include "MK64F12.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"
#include "queue.h"
#include "RTC.h"

/****************************************************************************************************************/
/*	Local definitions*/

#define I2C0_BASEADDR I2C0
#define RTC_slave_address 0x51
#define I2C_MASTER_CLK I2C0_CLK_SRC
#define BUFFER_SIZE 2

#define QUEUE_LENGTH 3
#define QUEUE_ITEM_SIZE sizeof(uint8_t)

#define I2C0_BASEADDR I2C0
#define RTC_slave_address 0x51
#define I2C_MASTER_CLK I2C0_CLK_SRC
#define BUFFER_SIZE 2
#define EEPROM_SLAVE_ADDR 0x50

/****************************************************************************************************************/
/*	Constant terminal info */

const uint8_t menu[] = { "\033[2J\r"
		"\t1) Leer Memoria I2C \n\n\r"
		"\t2) Escribir memoria I2C\n\n\r"
		"\t3) Establecer Hora \n\n\r "
		"\t4) Establecer Fecha \n\n\r"
		"\t5) Formato de hora \n\n\r "
		"\t6) Leer hora \n\n\r"
		"\t7) Leer fecha \n\n\r"
		"\t8) Comunicacion con terminal 2 \n\n\r"
		"\t9) Eco en LCD \n\n\r"
		"\t Ingrese opcion:\n\n\r" };

extern const uint8_t ITESO[];

/****************************************************************************************************************/
/*	Local types handles*/

SemaphoreHandle_t mutex_TxRx;
QueueHandle_t g_RTC_mailbox;

EventGroupHandle_t g_UART_events;



volatile bool g_MasterCompletionFlag = false;
volatile bool txFinished;
volatile bool rxFinished;

/****************************************************************************************************************/
/*	Callback functions*/

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void *userData) {
	//Signal transfer success when received success status.
	if (status == kStatus_Success) {
		g_MasterCompletionFlag = true;
	}
}

void UART1_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
		void *userData) {
	userData = userData;
	if (kStatus_UART_TxIdle == status) {
		txFinished = true;
	}
	if (kStatus_UART_RxIdle == status) {
		rxFinished = true;

	}
}

void UART0_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
		void *userData) {
	userData = userData;
	if (kStatus_UART_TxIdle == status) {
		txFinished = true;
	}
	if (kStatus_UART_IdleLineDetected == status) {
		rxFinished = true;
	}

}

/****************************************************************************************************************/
/*	Init Tasks*/

void I2C_init() {

	CLOCK_EnableClock(kCLOCK_I2c0);
	CLOCK_EnableClock(kCLOCK_PortB);

	port_pin_config_t config_i2c = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTB, 2, &config_i2c);
	PORT_SetPinConfig(PORTB, 3, &config_i2c);

	i2c_master_config_t masterConfig;
	I2C_MasterGetDefaultConfig(&masterConfig);
	I2C_MasterInit(I2C0_BASEADDR, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
}

void sInitLCD_task(void * arg) {

	GPIO_ClearPinsOutput(GPIOD, 1 << LCD_RESET_PIN);
	vTaskDelay(100);/**delay of 100ms for properly reset*/
	GPIO_SetPinsOutput(GPIOD, 1 << LCD_RESET_PIN);

	config_lcd_spi_pins();

	LCDNokia_init();

	mutex_TxRx = xSemaphoreCreateMutex();
	
	g_UART_events = xEventGroupCreate();

	g_RTC_mailbox = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);

	xTaskCreate(UART0_init_task, "UART0 init", 200, NULL,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(sClockLCD_task, "LCD Nokia Print", 200, NULL,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(iReadRTC_task, "Read RTC seconds", 200, NULL,
	configMAX_PRIORITIES - 1, NULL);

	xTaskCreate(UART0_PrintHello_task, "Hello0", 200, NULL,
	configMAX_PRIORITIES - 2, NULL);

	vTaskDelete(NULL);
}



void UART1_init_task(void * arg) {

}

void UART0_init_task(void * arg) {

	CLOCK_EnableClock(kCLOCK_Uart0);
	uart_config_t user_config;
	UART_GetDefaultConfig(&user_config);
	user_config.baudRate_Bps = 115200;
	user_config.enableTx = true;
	user_config.enableRx = true;

	UART_Init(UART0, &user_config, CLOCK_GetFreq(UART0_CLK_SRC));
	vTaskDelete(NULL);

}

/****************************************************************************************************************/
/*	EEPROM Tasks*/


void Write_EEPROM(void * arg) {

	i2c_master_handle_t g_m_handle;
	I2C_MasterTransferCreateHandle(I2C0_BASEADDR, &g_m_handle,
			i2c_master_callback, NULL);

	i2c_master_transfer_t masterXfer;
	uint8_t data_buff = 0x05;

	//Init I2C master.
	masterXfer.slaveAddress = EEPROM_SLAVE_ADDR;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0x00;
	masterXfer.subaddressSize = 2;
	masterXfer.data = &data_buff;
	masterXfer.dataSize = 2;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0_BASEADDR, &g_m_handle, &masterXfer);

	//Wait for transfer completed.
	while (!g_MasterCompletionFlag) {
	}

	g_MasterCompletionFlag = false;

	uint8_t read_buffer;
	masterXfer.slaveAddress = EEPROM_SLAVE_ADDR;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = 0x00;
	masterXfer.subaddressSize = 2;
	masterXfer.data = &read_buffer;
	masterXfer.dataSize = 2;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
//			while (!g_MasterCompletionFlag) {}
	g_MasterCompletionFlag = false;
	PRINTF("%x\r", read_buffer);
	for (;;) {

		vTaskDelay(pdMS_TO_TICKS(1000));

	}
}

/****************************************************************************************************************/
/*	RTC Tasks*/


void iReadRTC_task(void * arg) {

	CLOCK_EnableClock(kCLOCK_I2c0);
	CLOCK_EnableClock(kCLOCK_PortB);

	port_pin_config_t config_i2c = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTB, 2, &config_i2c);
	PORT_SetPinConfig(PORTB, 3, &config_i2c);

	i2c_master_config_t masterConfig;
	I2C_MasterGetDefaultConfig(&masterConfig);
	I2C_MasterInit(I2C0_BASEADDR, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	i2c_master_handle_t g_m_handle;
	I2C_MasterTransferCreateHandle(I2C0_BASEADDR, &g_m_handle,
			i2c_master_callback, NULL);

	i2c_master_transfer_t masterXfer;
	uint8_t data_buff = 0x11;
	uint8_t read_buffer;

	masterXfer.slaveAddress = 0x51;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0x04;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &data_buff;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	while (!g_MasterCompletionFlag) {
	}

	g_MasterCompletionFlag = false;

	for (;;) {

		masterXfer.direction = kI2C_Read;

		masterXfer.subaddress = 0x04;

		I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
		while (!g_MasterCompletionFlag) {
		}
		g_MasterCompletionFlag = false;

		xQueueSendToBack(g_RTC_mailbox, masterXfer.data, 10);

		masterXfer.subaddress = 0x03;

		I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
		while (!g_MasterCompletionFlag) {
		}
		g_MasterCompletionFlag = false;

		xQueueSendToBack(g_RTC_mailbox, masterXfer.data, 10);

		masterXfer.subaddress = 0x02;

		I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
		while (!g_MasterCompletionFlag) {
		}
		g_MasterCompletionFlag = false;

		xQueueSendToBack(g_RTC_mailbox, masterXfer.data, 10);

		vTaskDelay(pdMS_TO_TICKS(1000));

	}
}


/****************************************************************************************************************/
/*	LCD Tasks*/

void sClockLCD_task(void * arg) {

	RTC_timeUnion_t xTimeReceived;
	uint8_t sendData[10] = { };

	LCDNokia_clear();/*! It clears the information printed in the LCD*/
	LCDNokia_bitmap(&ITESO[0]); /*! It prints an array that hold an image, in this case is the initial picture*/
	delay(10000000);
	LCDNokia_clear();
	LCDNokia_clear();
	LCDNokia_gotoXY(5, 0); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendString("S I S T E M A S	", 0); /*! It print a string stored in an array*/
	LCDNokia_gotoXY(0, 1);
	LCDNokia_sendString("E M B E B I D O S       II", 0); /*! It print a string stored in an array*/
	LCDNokia_gotoXY(25, 4);
	LCDNokia_sendString("2 0 1 8 ", 0); /*! It print a string stored in an array*/

	delay(10000000);
#define RTC

	for (;;) {
#ifdef RTC
		if ( xQueueReceive(g_RTC_mailbox, &xTimeReceived,
				portMAX_DELAY) == pdPASS) {
			// Prepare to send.

			sendData[0] = xTimeReceived.hours.hours_tens + '0';
			sendData[1] = xTimeReceived.hours.hours_units + '0';
			sendData[2] = ':';

			xQueueReceive(g_RTC_mailbox, &xTimeReceived, portMAX_DELAY);

			sendData[3] = xTimeReceived.minutes.minutesTens + '0';
			sendData[4] = xTimeReceived.minutes.minutesUnits + '0';
			sendData[5] = ':';

			xQueueReceive(g_RTC_mailbox, &xTimeReceived, portMAX_DELAY);

			sendData[6] = xTimeReceived.seconds.secondsTens + '0';
			sendData[7] = xTimeReceived.seconds.secondsUnits + '0';
			sendData[8] = ' ';
			sendData[9] = '\0';

			LCDNokia_clear();
			LCDNokia_gotoXY(25, 4);
			LCDNokia_sendString(sendData, 0); /*! It print a string stored in an array*/

			UART_putString("\r");
			UART_putString(sendData);

		}
#endif
		vTaskDelay(pdMS_TO_TICKS(500));

	}
}

/****************************************************************************************************************/
/*	PRINT Tasks*/



void UART1_PrintHello_task(void * arg) {

	CLOCK_EnableClock(kCLOCK_Uart1);
	void *userData;
	uart_handle_t g_uartHandle;
	uart_config_t user_config;
	uart_transfer_t sendXfer;
	uart_transfer_t receiveXfer;

	uint8_t sendData[] = { 'H', 'e', 'l', 'l', 'o' };
	uint8_t receiveData[32];

//...
	UART_GetDefaultConfig(&user_config);
	user_config.baudRate_Bps = 9600;
	user_config.enableTx = true;
	user_config.enableRx = true;
//	UART_Init(UART0, &user_config, CLOCK_GetFreq(UART0_CLK_SRC));
//	UART_TransferCreateHandle(UART0, &g_uartHandle, UART_UserCallback,
//			userData);
	UART_Init(UART1, &user_config, CLOCK_GetFreq(UART1_CLK_SRC));
	UART_TransferCreateHandle(UART1, &g_uartHandle, UART1_UserCallback,
			userData);

// Prepare to send.
	sendXfer.data = sendData;
	sendXfer.dataSize = sizeof(sendData) / sizeof(sendData[0]);
	txFinished = false;
// Send out.

	for (;;) {
		UART_TransferSendNonBlocking(UART1, &g_uartHandle, &sendXfer);

		// Wait send finished.
		while (!txFinished) {
		}

// Prepare to receive.

		vTaskDelay(pdMS_TO_TICKS(1000));
// ...

	}

}

void UART0_PrintHello_task(void * arg) {

	void *userData;

	uart_handle_t g_uartHandle;
	uart_transfer_t sendXfer;
	uart_transfer_t receiveXfer;
	uint8_t receiveData[1];
	uint8_t echo[1] = { 0 };

	UART_putString(menu);

	//...

	UART_TransferCreateHandle(UART0, &g_uartHandle, UART0_UserCallback,
			userData);

	for (;;) {

		receiveXfer.data = receiveData;
		receiveXfer.dataSize = sizeof(receiveData) / sizeof(receiveData[0]);

		if (true == rxFinished) {
			rxFinished = false;

			sendXfer.data = receiveXfer.data;
			sendXfer.dataSize = sizeof(echo) / sizeof(echo[0]);
			txFinished = false;

			UART_TransferSendNonBlocking(UART0, &g_uartHandle, &sendXfer);
			// Wait send finished.
			while (!txFinished) {
			}

		}

		UART_TransferReceiveNonBlocking(UART0, &g_uartHandle, &receiveXfer,
		NULL);

	}

}
/****************************************************************************************************************/
/*	Local function*/

void UART_putString(void * arg) {

	uint8_t * dataToSend = (uint8_t*) arg;

	uart_handle_t g_uartHandle;
	uart_transfer_t sendXfer;
	uint8_t index = 0;

	for (index = 0; (dataToSend[index] != '\0'); index++) {

	}

	uint8_t sendData[index];

	UART_TransferCreateHandle(UART0, &g_uartHandle, UART1_UserCallback,
	NULL);

	for (index = 0; (dataToSend[index] != '\0'); index++) {
		sendData[index] = dataToSend[index];
	}

	// Prepare to send.
	sendXfer.data = sendData;
	sendXfer.dataSize = sizeof(sendData) / sizeof(sendData[0]);
	txFinished = false;
	// Send out.

	UART_TransferSendNonBlocking(UART0, &g_uartHandle, &sendXfer);
	// Wait send finished.
	while (!txFinished) {
	}

}

/****************************************************************************************************************/



