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
#include "semphr.h"
#include "queue.h"

#define I2C0_BASEADDR I2C0
#define RTC_slave_address 0x51
#define I2C_MASTER_CLK I2C0_CLK_SRC
#define BUFFER_SIZE 2

#define QUEUE_LENGTH 1
#define QUEUE_ITEM_SIZE sizeof(uint8_t)

typedef enum {
	seconds_type, minutes_type, hours_type
} time_types_t;

typedef struct {
	time_types_t time_type;
	uint8_t value;
} time_msg_t;

SemaphoreHandle_t mutex;
QueueHandle_t g_charEcho_mailbox;

extern const uint8_t ITESO[];

void delay(uint16_t delay) {
	volatile uint16_t counter;
	for (counter = delay; counter > 0; counter--)
		;
}

/*
 * @brief   Application entry point.
 *
 */

typedef enum {
	CONTROL_STATUS,
	HUNDREDS,
	SECONDS,
	MINUTES,
	HOURS,
	YEAR_DATE,
	WEEKDAYS_MONTH,
	TIMER,
	ALARM_CONTROL
} RTC_registers_t;

volatile bool g_MasterCompletionFlag = false;

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void *userData) {
	//Signal transfer success when received success status.
	if (status == kStatus_Success) {
		g_MasterCompletionFlag = true;
	}
}

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
	uint8_t data_buff = 0x00;

	for (;;) {

		//Init I2C master.
		masterXfer.slaveAddress = 0x51;
		masterXfer.direction = kI2C_Write;
		masterXfer.subaddress = 0x00;
		masterXfer.subaddressSize = 1;
		masterXfer.data = &data_buff;
		masterXfer.dataSize = 1;
		masterXfer.flags = kI2C_TransferDefaultFlag;

		I2C_MasterTransferNonBlocking(I2C0_BASEADDR, &g_m_handle, &masterXfer);

		//Wait for transfer completed.
		while (!g_MasterCompletionFlag) {
		}

		g_MasterCompletionFlag = false;

		uint8_t read_buffer;
		masterXfer.slaveAddress = 0x51;
		masterXfer.direction = kI2C_Read;
		masterXfer.subaddress = 0x02;
		masterXfer.subaddressSize = 1;
		masterXfer.data = &read_buffer;
		masterXfer.dataSize = 1;
		masterXfer.flags = kI2C_TransferDefaultFlag;

		I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
		while (!g_MasterCompletionFlag) {
		}
		g_MasterCompletionFlag = false;

		xQueueSendToBack(g_charEcho_mailbox, masterXfer.data, 10);

		vTaskDelay(pdMS_TO_TICKS(1000));

	}
}

void sInitLCD_task(void * arg) {

	GPIO_ClearPinsOutput(GPIOD, 1 << LCD_RESET_PIN);
	vTaskDelay(100);/**delay of 100ms for properly reset*/
	GPIO_SetPinsOutput(GPIOD, 1 << LCD_RESET_PIN);

	config_lcd_spi_pins();

	LCDNokia_init();

	g_charEcho_mailbox = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
	xTaskCreate(sItesoLCD_task, "LCD Nokia Print", 200, NULL,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(UART0_PrintHello_task, "Hello0", 200, NULL,
	configMAX_PRIORITIES - 2, NULL);

	xTaskCreate(iReadRTC_task, "RTC read", 200, NULL, configMAX_PRIORITIES - 2,
	NULL);
	vTaskDelete(NULL);
}

void sItesoLCD_task(void * arg) {

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

volatile bool txFinished;
volatile bool rxFinished;

void UART0_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
		void *userData) {
	userData = userData;

	if (kStatus_UART_TxIdle == status) {
		txFinished = true;
	}
	if (kStatus_UART_RxIdle == status) {
		rxFinished = true;

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

void UART1_PrintHello_task(void * arg) {

//	CLOCK_EnableClock(kCLOCK_Uart0);
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
		receiveXfer.data = receiveData;
		receiveXfer.dataSize = sizeof(receiveData) / sizeof(receiveData[0]);
		rxFinished = false;

		UART_TransferReceiveNonBlocking(UART1, &g_uartHandle, &receiveXfer,
		NULL);

		vTaskDelay(pdMS_TO_TICKS(1000));
// ...

	}

}

void UART0_PrintHello_task(void * arg) {

	CLOCK_EnableClock(kCLOCK_Uart0);
	void *userData;
	uart_handle_t g_uartHandle;
	uart_config_t user_config;
	uart_transfer_t sendXfer;
	uart_transfer_t receiveXfer;
	uint8_t xCharReceived;

	uint8_t sendData[2] = { 0, 0 };
	uint8_t receiveData[32];

//...
	UART_GetDefaultConfig(&user_config);
	user_config.baudRate_Bps = 115200;
	user_config.enableTx = true;
	user_config.enableRx = true;
	UART_Init(UART0, &user_config, CLOCK_GetFreq(UART0_CLK_SRC));
	UART_TransferCreateHandle(UART0, &g_uartHandle, UART0_UserCallback,
			userData);

// Prepare to send.
	sendXfer.data = sendData;
	sendXfer.dataSize = sizeof(sendData) / sizeof(sendData[0]);
	txFinished = false;

// Send out.

	UART_TransferSendNonBlocking(UART0, &g_uartHandle, &sendXfer);
	// Wait send finished.
	while (!txFinished) {
	}

	for (;;) {

		if ( xQueueReceive(g_charEcho_mailbox, &xCharReceived,
				portMAX_DELAY) != pdPASS) {

		} else {
			// Prepare to send.
			xCharReceived = ~(~xCharReceived & 0xFF) & 0x7F;

			sendData[0] = ((xCharReceived & 0x70)>>4) + '0';
			sendData[1] = (xCharReceived & 0x0F) + '0';

			sendXfer.data = sendData;
			sendXfer.dataSize = sizeof(sendData) / sizeof(sendData[0]);
			txFinished = false;

			UART_TransferSendNonBlocking(UART0, &g_uartHandle, &sendXfer);
			// Wait send finished.
			while (!txFinished) {
			}


		}

// Prepare to receive.
		receiveXfer.data = receiveData;
		receiveXfer.dataSize = sizeof(receiveData) / sizeof(receiveData[0]);
		rxFinished = false;

		UART_TransferReceiveNonBlocking(UART0, &g_uartHandle, &receiveXfer,
		NULL);

		vTaskDelay(pdMS_TO_TICKS(1000));
// ...

	}

}

void uPrintEcho_task(void * arg) {


	void *userData;
	uart_handle_t g_uartHandle;
	uart_transfer_t sendXfer;
	uint8_t sendData[1] = { 0 };

	UART_TransferCreateHandle( (UART_Type*) arg, &g_uartHandle, UART0_UserCallback,
			userData);

	// Prepare to send.
	sendData[0] =  ((UART_Type*) arg)->D;
	sendXfer.data = sendData;
	sendXfer.dataSize = sizeof(sendData) / sizeof(sendData[0]);
	txFinished = false;

	// Send out.

	UART_TransferSendNonBlocking( (UART_Type*) arg, &g_uartHandle, &sendXfer);
	// Wait send finished.
	while (!txFinished) {
	}

	vTaskDelete(NULL);

}

