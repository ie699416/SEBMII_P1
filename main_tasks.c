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
#define EEPROM_SLAVE_ADDR 0x50
#define I2C_MASTER_CLK I2C0_CLK_SRC
#define BUFFER_SIZE 2
#define ENTER_KEY 13

#define QUEUE_EEPROM_LENGTH 4
#define QUEUE_LENGTH 3
#define QUEUE_LENGTH_UART 1
#define QUEUE_ITEM_SIZE sizeof(uint8_t)
#define  CHAT_BFFR_LENGTH 20

/****************************************************************************************************************/
/*	Event definitions*/

/* READ EEPROM STATE MACHINE*/

#define EVENT_EEPROM_READ (1<<1)
#define EVENT_EEPROM_GET_ADDR  (1<<12)
#define EVENT_EEPROM_ADDR_FULL (1<<11)
#define EVENT_EEPROM_GET_LENGTH (1<<11)
#define EVENT_EEPROM_START_I2C_READ (1<<11)
#define EVENT_EEPROM_WAIT (1<<11)

#define EVENT_UART_ECHO (1<<0)
#define EVENT_UART_READ_EEPROM (1<<1)
#define EVENT_UART_WRITE_EEPROM (1<<2)
#define EVENT_UART_SET_HOUR (1<<3)
#define EVENT_UART_SET_DATE (1<<4)
#define EVENT_UART_HOUR_FORMAT (1<<5)
#define EVENT_UART_READ_HOUR (1<<6)
#define EVENT_UART_READ_DATE (1<<7)
#define EVENT_UART_RX (1<<8)
#define EVENT_UART_RESTORE_HANDLE (1<<9)
#define EVENT_INVALID_CHAR (1<<10)
#define EVENT_CHAR_SENT (1<<11)
#define EVENENT_CHAT_TASK_ON (1 << 12)

SemaphoreHandle_t smphrTx;
SemaphoreHandle_t smphrRx;

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

const uint8_t clear[] = { "\033[2J" };

const uint8_t goTo[] = { "\033[H" };

const uint8_t READ_EEPROM_address[] =
		{ "\n\n\t Introduzca direccion a leer: 0x" };

const uint8_t jumpLine[] = "\n";
const uint8_t emptyMsg[] = "                                             ";
extern const uint8_t ITESO[];

/****************************************************************************************************************/
/*	Local types handles*/
uart_handle_t g_uartHandle;
SemaphoreHandle_t mutex_TxRx;
QueueHandle_t g_RTC_mailbox;
QueueHandle_t g_EEPROM_address;
QueueHandle_t g_UART_mailbox;

EventGroupHandle_t g_UART_events;
EventGroupHandle_t t_UART_1events;

#define UART_COMPLETE (1<<1)
#define UART1_RX (1<<2)
#define UART_CHAT_ENTER_KEY (1<<3)

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

void UART0_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
		void *userData) {
	BaseType_t xHigherPriorityTaskWoken;
	BaseType_t xResult;

	xHigherPriorityTaskWoken = pdFALSE;
	xResult = pdFAIL;
	if (kStatus_UART_TxIdle == status) {
//		txFinished = true;
		xResult = xEventGroupSetBitsFromISR(t_UART_1events,
		UART_COMPLETE, &xHigherPriorityTaskWoken);
	}
	if (kStatus_UART_RxIdle == status) {
//		rxFinished = true;
		xResult = xEventGroupSetBitsFromISR(t_UART_1events,
		UART_COMPLETE, &xHigherPriorityTaskWoken);
	}
	if (xResult != pdFAIL) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void UART1_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
		void *userData) {

	BaseType_t xHigherPriorityTaskWoken, xResult;
	if (kStatus_UART_TxIdle == status) {

		xResult = xEventGroupSetBitsFromISR(t_UART_1events, UART_COMPLETE,
				&xHigherPriorityTaskWoken);

	}
	if (kStatus_UART_RxIdle == status) {
		xResult = xEventGroupSetBitsFromISR(t_UART_1events, UART_COMPLETE,
				&xHigherPriorityTaskWoken);

	}

	/* Was the message posted successfully? */
	if (xResult != pdFAIL) {
		/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
		 switch should be requested.  The macro used is port specific and will
		 be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
		 the documentation page for the port being used. */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

}

/****************************************************************************************************************/
/*	Local function*/

uint8_t toUpperCase(uint8_t * data) {
	if (data[0] >= 'a' && data[0] <= 'z') {
		data[0] -= 'a' - 'A';
		if (data[0] == 'ñ')
			data[0] = 'N';
	}

	return data[0];
}

void UART0_putString(uint8_t * dataToSend) {

	uart_handle_t g_uartHandle;
	uart_transfer_t sendXfer;
	volatile uint8_t index = 0;

	for (index; (dataToSend[index] != '\0'); index++) {
	}

	uint8_t sendData[index];

	UART_TransferCreateHandle(UART0, &g_uartHandle, UART0_UserCallback,
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

void UART1_putString(uint8_t * dataToSend) {

	EventBits_t uartEvents;
	uart_transfer_t sendXfer;
	uint8_t index = 0;
	uint8_t indexFromChat = 0;
	uint8_t indexFromChat2 = 0;
	uint8_t putStringRet = 0;
	uint8_t ev;
	bool MsgFromChatFlag = pdFALSE;
	uint8_t fixCursorForChat = '\r';


	uint8_t *dataToSendAux = dataToSend;

	/*Stores an event weather or not a Message from chat had been sent.*/
	uartEvents = xEventGroupGetBits(t_UART_1events);
	/* getting the length from the Message to print only valid characters*/
	for (index = 0; (dataToSend[index] != '\0'); index++) {

	}

	uint8_t sendData[index];

	/* Fills a new array with correct length with the message, for printing valid characters.
	 * Then checks weather or not a Message from chat had been sent, this conditional its for
	 * preserve the integrity of the message to print on terminal.*/
	for (index = 0; (dataToSend[index] != '\0'); index++) {
		sendData[index] = dataToSend[index];
		dataToSendAux++;
		if ((*dataToSendAux == ENTER_KEY)
				&& (UART_CHAT_ENTER_KEY == (UART_CHAT_ENTER_KEY & uartEvents))) {
			for (indexFromChat = 0; (indexFromChat <= index); indexFromChat++) {
			}
			MsgFromChatFlag = pdTRUE;
		}

	}
	/* Creating a New array for a Message from chat for integrity of it */
	uint8_t sendDataFromChat[indexFromChat];

	for (indexFromChat2 = 0; indexFromChat2 <= indexFromChat;
			indexFromChat2++) {
		sendDataFromChat[indexFromChat2] = sendData[indexFromChat2];
	}

	/*If the message had a ENTER_KEY and the message is from the chat
	 * sends the SendDataFromChat Array else Send Normal echo*/
	if (( UART_CHAT_ENTER_KEY == ( UART_CHAT_ENTER_KEY & uartEvents))
			&& (pdTRUE == MsgFromChatFlag)) {
		xEventGroupClearBits(t_UART_1events, UART_CHAT_ENTER_KEY);
		// Prepare to send.
		sendXfer.data = &fixCursorForChat;
		sendXfer.dataSize = 1;

		// Send out.d

		UART_TransferSendNonBlocking(UART1, &g_uartHandle, &sendXfer);
		// Wait send finished.

		xEventGroupWaitBits(t_UART_1events, UART_COMPLETE, pdTRUE, pdFALSE,
		portMAX_DELAY); // wait fir the the callback flag

		// Prepare to send.
		sendXfer.data = sendDataFromChat;
		sendXfer.dataSize = sizeof(sendDataFromChat)
				/ sizeof(sendDataFromChat[0]);

		// Send out.d

		UART_TransferSendNonBlocking(UART1, &g_uartHandle, &sendXfer);
		// Wait send finished.

		xEventGroupWaitBits(t_UART_1events, UART_COMPLETE, pdTRUE, pdFALSE,
		portMAX_DELAY); // wait fir the the callback flag

		sendXfer.data = &fixCursorForChat;
		sendXfer.dataSize = 1;

		// Send out.d

		UART_TransferSendNonBlocking(UART1, &g_uartHandle, &sendXfer);
		// Wait send finished.

		xEventGroupWaitBits(t_UART_1events, UART_COMPLETE, pdTRUE, pdFALSE,
		portMAX_DELAY); // wait fir the the callback flag

	} else {
		// Prepare to send.
		sendXfer.data = sendData;
		sendXfer.dataSize = sizeof(sendData) / sizeof(sendData[0]);


		// Send out.d

		UART_TransferSendNonBlocking(UART1, &g_uartHandle, &sendXfer);
		// Wait send finished.

		xEventGroupWaitBits(t_UART_1events, UART_COMPLETE, pdTRUE, pdFALSE,
		portMAX_DELAY); // wait fir the the callback flag

	}

}

/****************************************************************************************************************/
/*	System Tasks*/

void U0_systemMenu_task(void *arg) {

	uint8_t xCharReceived;
	EventBits_t uart_events;
	UART1_putString(menu);

	for (;;) {

		uart_events = xEventGroupWaitBits(g_UART_events, EVENT_CHAR_SENT,
		pdTRUE, pdFALSE, portMAX_DELAY);

		if ( xQueueReceive(g_UART_mailbox, &xCharReceived,
				portMAX_DELAY) == pdPASS) {
			switch (xCharReceived) {
			case '1':
				xEventGroupSetBits(g_UART_events, EVENT_UART_READ_EEPROM);
				break;
			case '2':
				vTaskDelay(pdMS_TO_TICKS(500));
				UART0_putString(clear);
				break;
			case '8':
				xEventGroupSetBits(g_UART_events, EVENENT_CHAT_TASK_ON);
				vTaskDelete(NULL);
			default:
				break;
			}

		}

		vTaskDelay(pdMS_TO_TICKS(500));

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
	vTaskDelete(NULL);
}

void sInitLCD_task(void * arg) {

	GPIO_ClearPinsOutput(GPIOD, 1 << LCD_RESET_PIN);
	vTaskDelay(100);/**delay of 100ms for properly reset*/
	GPIO_SetPinsOutput(GPIOD, 1 << LCD_RESET_PIN);

	config_lcd_spi_pins();

	LCDNokia_init();
	g_UART_mailbox = xQueueCreate(QUEUE_LENGTH_UART, QUEUE_ITEM_SIZE);

	mutex_TxRx = xSemaphoreCreateMutex();

	g_UART_events = xEventGroupCreate();

	t_UART_1events = xEventGroupCreate();

	g_EEPROM_address = xQueueCreate(QUEUE_EEPROM_LENGTH, QUEUE_ITEM_SIZE);

	g_RTC_mailbox = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);

	xTaskCreate(UART1_init_task, "UART1_init", 600, NULL,
	configMAX_PRIORITIES, NULL);

	//xTaskCreate(UART0_init_task, "UART0 init", 200, NULL,
	//configMAX_PRIORITIES, NULL);

	xTaskCreate(U0_systemMenu_task, "UART0 menu ", 600, NULL,
	configMAX_PRIORITIES - 2, NULL);

	xTaskCreate(UART1_GetChar_task, "GetChar1", 200, NULL,
	configMAX_PRIORITIES - 3, NULL);

	xTaskCreate(Chat_Task, "Chat", 400, NULL,
	configMAX_PRIORITIES - 3, NULL);

	//	xTaskCreate(UART0_readEEPROM_task, "UART0 menu 1 ", 200, NULL,
	//	configMAX_PRIORITIES - 3, NULL);

	//	xTaskCreate(UART0_PrintHello_task, "Hello0", 200, NULL,
	//	configMAX_PRIORITIES - 4, NULL);
	//	xTaskCreate(Write_EEPROM, "Write_EEPROM", 200, NULL,
	//	configMAX_PRIORITIES, NULL);

	//	xTaskCreate(sClockLCD_task, "LCD Nokia Print", 200, NULL,
	//	configMAX_PRIORITIES, NULL);
	//
	//	xTaskCreate(iReadRTC_task, "Read RTC seconds", 200, NULL,
	//	configMAX_PRIORITIES - 1, NULL);
	//
	//	xTaskCreate(Read_EEPROM, "Read_EEPROM", 200, NULL,
	//	configMAX_PRIORITIES - 1, NULL);

	vTaskDelete(NULL);
}

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

	UART_Init(UART1, &user_config, CLOCK_GetFreq(UART1_CLK_SRC));
	UART_TransferCreateHandle(UART1, &g_uartHandle, UART1_UserCallback,
	NULL);
	smphrTx = xSemaphoreCreateMutex();

	vTaskDelete(NULL);
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

void Read_EEPROM(void * arg) {

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

	i2c_master_handle_t g_m_handle;
	I2C_MasterTransferCreateHandle(I2C0_BASEADDR, &g_m_handle,
			i2c_master_callback, NULL);

	i2c_master_transfer_t masterXfer;

	//	    uint8_t read_buffer;
	uint8_t read_buffer[7];
	masterXfer.slaveAddress = EEPROM_SLAVE_ADDR;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = 0x0000;
	masterXfer.subaddressSize = 2;
	masterXfer.data = read_buffer;
	masterXfer.dataSize = 7;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	vTaskDelay(pdMS_TO_TICKS(500));

//	uint8_t sendData[];

	for (;;) {

		xSemaphoreTake(mutex_TxRx, portMAX_DELAY);
		I2C_MasterTransferCreateHandle(I2C0_BASEADDR, &g_m_handle,
				i2c_master_callback, NULL);
		I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
		while (!g_MasterCompletionFlag) {
		}
		g_MasterCompletionFlag = false;
		xSemaphoreGive(mutex_TxRx);
		UART1_putString(read_buffer);

//		sendData[0] = ((read_buffer & 0xF0) >> 4) + '0';
//		sendData[1] = (read_buffer & 0x0F) + '0';
//		sendData[2] = '\0';

//UART_putString("\r");

		vTaskDelay(pdMS_TO_TICKS(1000));

	}

}

void Write_EEPROM(void * arg) {
	CLOCK_EnableClock(kCLOCK_I2c0);
	CLOCK_EnableClock(kCLOCK_PortE);

	port_pin_config_t config_i2c = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAlt5, kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTE, 24, &config_i2c);
	PORT_SetPinConfig(PORTE, 25, &config_i2c);

	i2c_master_config_t masterConfig;
	i2c_master_transfer_t masterXfer;
	i2c_master_handle_t g_m_handle;

	I2C_MasterGetDefaultConfig(&masterConfig);
	I2C_MasterInit(I2C0_BASEADDR, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	I2C_MasterTransferCreateHandle(I2C0_BASEADDR, &g_m_handle,
			i2c_master_callback, NULL);

	uint8_t data_buff[] = { "hola\n\r" };

	//Init I2C master.
	masterXfer.slaveAddress = EEPROM_SLAVE_ADDR;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0x0000;
	masterXfer.subaddressSize = 2;
	masterXfer.data = data_buff;
	masterXfer.dataSize = 7;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0_BASEADDR, &g_m_handle, &masterXfer);

	//Wait for transfer completed.
	while (!g_MasterCompletionFlag) {
	}
	g_MasterCompletionFlag = false;

	for (;;) {

		vTaskDelay(pdMS_TO_TICKS(100));

	}
}

/****************************************************************************************************************/
/*	RTC Tasks*/

void iReadRTC_task(void * arg) {

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

	i2c_master_handle_t g_m_handle;

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

		xSemaphoreTake(mutex_TxRx, portMAX_DELAY);
		I2C_MasterTransferCreateHandle(I2C0_BASEADDR, &g_m_handle,
				i2c_master_callback, NULL);
		I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
		while (!g_MasterCompletionFlag) {
		}
		g_MasterCompletionFlag = false;
		xSemaphoreGive(mutex_TxRx);

		xQueueSendToBack(g_RTC_mailbox, masterXfer.data, 10);

		masterXfer.subaddress = 0x03;

		xSemaphoreTake(mutex_TxRx, portMAX_DELAY);
		I2C_MasterTransferCreateHandle(I2C0_BASEADDR, &g_m_handle,
				i2c_master_callback, NULL);
		I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
		while (!g_MasterCompletionFlag) {
		}
		g_MasterCompletionFlag = false;
		xSemaphoreGive(mutex_TxRx);

		xQueueSendToBack(g_RTC_mailbox, masterXfer.data, 10);

		masterXfer.subaddress = 0x02;

		xSemaphoreTake(mutex_TxRx, portMAX_DELAY);
		I2C_MasterTransferCreateHandle(I2C0_BASEADDR, &g_m_handle,
				i2c_master_callback, NULL);
		I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
		while (!g_MasterCompletionFlag) {
		}
		g_MasterCompletionFlag = false;
		xSemaphoreGive(mutex_TxRx);

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
	vTaskDelay(pdMS_TO_TICKS(1000));
	LCDNokia_clear();
	LCDNokia_clear();
	LCDNokia_gotoXY(5, 0); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendString("S I S T E M A S	", 0); /*! It print a string stored in an array*/
	LCDNokia_gotoXY(0, 1);
	LCDNokia_sendString("E M B E B I D O S       II", 0); /*! It print a string stored in an array*/
	LCDNokia_gotoXY(25, 4);
	LCDNokia_sendString("2 0 1 8 ", 0); /*! It print a string stored in an array*/

	vTaskDelay(pdMS_TO_TICKS(1000));
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

			//UART_putString("\r");
			//UART_putString(sendData);

		}
#endif
		vTaskDelay(pdMS_TO_TICKS(500));

	}
}

/****************************************************************************************************************/
/*	PRINT Tasks*/

void UART1_GetChar_task(void * arg) {

	EventBits_t uartEvents;
	uart_transfer_t receiveXfer;
	uint8_t receiveData[1];
	uint8_t echo[1] = { 0 };

	//...

	for (;;) {

		//xSemaphoreTake(smphrRx, portMAX_DELAY);
		xEventGroupSetBits(g_UART_events, EVENT_UART_RX);
		//xEventGroupWaitBits(t_UART_1events, UART_COMPLETE, pdTRUE, pdFALSE, portMAX_DELAY);
		// Prepare to receive.
		receiveXfer.data = receiveData;
		receiveXfer.dataSize = sizeof(receiveData) / sizeof(receiveData[0]);
//		rxFinished = false;

		// Receive.

		UART_TransferReceiveNonBlocking(UART1, &g_uartHandle, &receiveXfer,
		NULL);
		// Wait receive finished.

		xEventGroupWaitBits(t_UART_1events, UART_COMPLETE, pdTRUE, pdFALSE,
		portMAX_DELAY);

		//xSemaphoreGive(smphrRx);

		xQueueSendToBack(g_UART_mailbox, receiveXfer.data, 10);
		xEventGroupSetBits(g_UART_events, EVENT_CHAR_SENT);

		vTaskDelay(pdMS_TO_TICKS(100));
	}

}

void UART0_PrintHello_task(void * arg) {

	EventBits_t uartEvents;
	uart_transfer_t receiveXfer;
	uint8_t receiveData[1];
	uint8_t echo[1] = { 0 };

	//...

	uart_handle_t g_uart0Handle;
	uart_transfer_t sendXfer0;
	UART_TransferCreateHandle(UART0, &g_uart0Handle, UART0_UserCallback,
	NULL);

	uartEvents = xEventGroupGetBits(g_UART_events);

	xEventGroupSetBits(g_UART_events, EVENT_UART_RX);
	xEventGroupSetBits(g_UART_events, EVENT_UART_ECHO);

	for (;;) {

		uartEvents = xEventGroupGetBits(g_UART_events);

		if (EVENT_UART_RESTORE_HANDLE
				== ( EVENT_UART_RESTORE_HANDLE & uartEvents)) {
			xEventGroupClearBits(g_UART_events, EVENT_UART_RESTORE_HANDLE);

			UART_TransferCreateHandle(UART0, &g_uart0Handle, UART0_UserCallback,
			NULL);

			xEventGroupSetBits(g_UART_events, EVENT_UART_RX);

		}
		xEventGroupSetBits(g_UART_events, EVENT_UART_RX);
		receiveXfer.data = receiveData;
		receiveXfer.dataSize = sizeof(receiveData) / sizeof(receiveData[0]);

		if (true == rxFinished) {
			rxFinished = false;

			xEventGroupSetBits(g_UART_events, EVENT_UART_RX);

			if ((receiveXfer.data[0] >= '0' && receiveXfer.data[0] <= '9')
					|| (receiveXfer.data[0] >= 'A' && receiveXfer.data[0] <= 'F')
					|| (receiveXfer.data[0] >= 'a' && receiveXfer.data[0] <= 'f')) {

				xEventGroupSetBits(g_UART_events, EVENT_INVALID_CHAR);
			} else {
				xEventGroupClearBits(g_UART_events, EVENT_INVALID_CHAR);
			}

			receiveXfer.data[0] = toUpperCase(receiveXfer.data);

			if ((EVENT_EEPROM_GET_ADDR == (EVENT_EEPROM_GET_ADDR & uartEvents))) {

				xQueueSendToBack(g_EEPROM_address, receiveXfer.data, 10);
			}

			xQueueSendToBack(g_UART_mailbox, receiveXfer.data, 10);
			xEventGroupSetBits(g_UART_events, EVENT_CHAR_SENT);
			uartEvents = xEventGroupGetBits(g_UART_events);

			if (EVENT_UART_ECHO == (EVENT_UART_ECHO & uartEvents)) {

				sendXfer0.data = receiveXfer.data;
				sendXfer0.dataSize = sizeof(echo) / sizeof(echo[0]);
				txFinished = false;

				UART_TransferSendNonBlocking(UART0, &g_uart0Handle, &sendXfer0);
				// Wait send finished.
				while (!txFinished) {

				}
			}
		}

		//io ize exta parte edsom
		if (EVENT_UART_RX == (EVENT_UART_RX & uartEvents)) {
			xEventGroupClearBits(g_UART_events, EVENT_UART_RX);

			UART_TransferReceiveNonBlocking(UART0, &g_uart0Handle, &receiveXfer,
			NULL);

		}

	}
}

/****************************************************************************************************************/
/*	Menu functions*/

void UART0_readEEPROM_task(void * arg) {
	EventBits_t uartEvents;
	uint8_t buffer[QUEUE_EEPROM_LENGTH];
	uint8_t addrCharLength = 0;

	xEventGroupSetBits(g_UART_events, EVENT_EEPROM_ADDR_FULL);

	for (;;) {
		uartEvents = xEventGroupGetBits(g_UART_events);

		if (EVENT_UART_READ_EEPROM == (EVENT_UART_READ_EEPROM & uartEvents)) {
			xEventGroupClearBits(g_UART_events, EVENT_UART_READ_EEPROM);

			UART0_putString(clear);
			UART0_putString(goTo);
			UART0_putString(READ_EEPROM_address);

			xEventGroupSetBits(g_UART_events, EVENT_UART_RESTORE_HANDLE);

			xEventGroupSetBits(g_UART_events, EVENT_EEPROM_GET_ADDR);

		}

		uartEvents = xEventGroupGetBits(g_UART_events);

		if ((EVENT_EEPROM_ADDR_FULL == (EVENT_EEPROM_ADDR_FULL & uartEvents))) {

			if ((EVENT_EEPROM_GET_ADDR == (EVENT_EEPROM_GET_ADDR & uartEvents))) {

				if ( xQueueReceive(g_EEPROM_address, &buffer[addrCharLength],
						portMAX_DELAY) == pdPASS) {
					addrCharLength++;
					if (QUEUE_EEPROM_LENGTH <= addrCharLength) {
						xEventGroupClearBits(g_UART_events,
						EVENT_EEPROM_GET_ADDR);

						xEventGroupClearBits(g_UART_events,
						EVENT_EEPROM_ADDR_FULL);

					}
				}

			}
		}

		vTaskDelay(pdMS_TO_TICKS(500));

	}

}

void Chat_Task(void *arg) {
	xEventGroupWaitBits(g_UART_events, EVENENT_CHAT_TASK_ON, pdTRUE, pdFALSE,
	portMAX_DELAY);
	const uint8_t comBtwTerm[] = {
			"Empieza comunicacion entre terminales...\n\r\n" };
	const uint8_t MsgFromTerONE[] = { "Terminal UNO dice:\n\r" };
	const uint8_t MsgFromTerTWO[] = { "Terminal DOS dice:\n\r\n" };
	UART1_putString(clear);
	UART1_putString(goTo);
	UART1_putString(comBtwTerm);

	uint8_t chatBffr[2];
	chatBffr[1] = '\0';
	uint8_t addrCharLength = 0;
	uint8_t enterSendBffr[500];
	uint8_t resetChatBffrIndex;
	uint8_t entersendBffrIndex = 0;
	uint8_t index;
	uint8_t *enterSendBffrRealLength;
	for (;;) {

		xEventGroupWaitBits(g_UART_events, EVENT_CHAR_SENT, pdTRUE, pdFALSE,
		portMAX_DELAY);

		xQueueReceive(g_UART_mailbox, &chatBffr[0], portMAX_DELAY);
		enterSendBffr[entersendBffrIndex] = chatBffr[0];
		entersendBffrIndex++;

		UART1_putString(chatBffr);

		if (chatBffr[0] == ENTER_KEY) {
			xEventGroupSetBits(t_UART_1events, UART_CHAT_ENTER_KEY);
			enterSendBffrRealLength = pvPortMalloc(sizeof (uint8_t)*entersendBffrIndex);


			for (index = 0; index <= entersendBffrIndex; index++) {
				enterSendBffrRealLength[index] = enterSendBffr[index];

			}
			UART1_putString(jumpLine);
			UART1_putString(MsgFromTerONE);
			xEventGroupSetBits(t_UART_1events, UART_CHAT_ENTER_KEY);
			UART1_putString(enterSendBffrRealLength);
			UART1_putString(jumpLine);

			for(resetChatBffrIndex = 0; resetChatBffrIndex < 200;resetChatBffrIndex ++){
				enterSendBffr[resetChatBffrIndex] = 0x20;
					}
			vPortFree(enterSendBffrRealLength);
			entersendBffrIndex=0;

		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
