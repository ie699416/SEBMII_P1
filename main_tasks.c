#include "definitions.h"
#include "MAL.h"

#define I2C0_BASEADDR I2C0
#define RTC_slave_address 0x51
#define I2C_MASTER_CLK I2C0_CLK_SRC
#define BUFFER_SIZE 2
#define EEPROM_SLAVE_ADDR 0x50

/****************************************************************************************************************/
/*	Local types handles*/

SemaphoreHandle_t mutex_TxRx;

void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void *userData) {
	BaseType_t xHigherPriorityTaskWoken;
	BaseType_t xResult;

	xHigherPriorityTaskWoken = pdFALSE;
	xResult = pdFAIL;
	//Signal transfer success when received success status.
	if (status == kStatus_Success) {
		xResult = xEventGroupSetBitsFromISR(get_g_I2C_events(),
		EVENT_I2C_MASTER_TX_COMPLETE, &xHigherPriorityTaskWoken);
	}
	if (pdFAIL != xResult) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/****************************************************************************************************************/
/*	Local function*/

uint8_t toUpperCase(uint8_t * data) {
	if (data[0] >= 'a' && data[0] <= 'z') {
		data[0] -= 'a' - 'A';
	}
	return data[0];
}

/****************************************************************************************************************/
/*	System Tasks*/

void U0_systemMenu_task(void *arg) {

	uint8_t xCharReceived;

	UART0_putString(getClearScreen());
	UART0_putString(getMenu());

	xEventGroupSetBits(get_g_TERM0_events(), EVENT_MENU_WAIT);

	for (;;) {

		xEventGroupWaitBits(get_g_TERM0_events(), EVENT_CHAR_SENT,
		pdTRUE, pdFALSE, portMAX_DELAY);

		if ( xQueueReceive(get_g_UART0_mailbox(), &xCharReceived,
				portMAX_DELAY) == pdPASS) {
			switch (xCharReceived) {
			case ESCAPE_KEY:
				UART0_putString(getClearScreen());
				UART0_putString(getMenu());
				xEventGroupSetBits(get_g_TERM0_events(), EVENT_MENU_WAIT);
				break;
			case '1':
				xEventGroupSetBits(get_g_TERM0_events(),
				EVENT_UART_READ_EEPROM);
				break;
			case '2':
				break;
			default:
				xEventGroupSetBits(get_g_TERM0_events(), EVENT_MENU_WAIT);
				break;
			}

		}

	}

}

void U1_systemMenu_task(void *arg) {

	uint8_t xCharReceived;
	UART1_putString(getClearScreen());

	UART1_putString(getMenu());

	xEventGroupSetBits(get_g_TERM1_events(), EVENT_MENU_WAIT);

	for (;;) {

		xEventGroupWaitBits(get_g_TERM1_events(), EVENT_CHAR_SENT,
		pdTRUE, pdFALSE, portMAX_DELAY);

		if ( xQueueReceive(get_g_UART1_mailbox(), &xCharReceived,
				portMAX_DELAY) == pdPASS) {
			switch (xCharReceived) {
			case ESCAPE_KEY:
				UART1_putString(getClearScreen());
				UART1_putString(getMenu());
				xEventGroupSetBits(get_g_TERM1_events(), EVENT_MENU_WAIT);
				break;
			case '1':
				xEventGroupSetBits(get_g_TERM1_events(),
				EVENT_UART_READ_EEPROM);
				break;
			case '2':
				break;
			default:
				xEventGroupSetBits(get_g_TERM1_events(), EVENT_MENU_WAIT);
				break;
			}

		}

	}

}

/****************************************************************************************************************/
/*	Init Tasks*/

void sInitLCD_task(void * arg) {

	GPIO_ClearPinsOutput(GPIOD, 1 << LCD_RESET_PIN);
	vTaskDelay(100);/**delay of 100ms for properly reset*/
	GPIO_SetPinsOutput(GPIOD, 1 << LCD_RESET_PIN);

	config_lcd_spi_pins();

	LCDNokia_init();

	mutex_TxRx = xSemaphoreCreateMutex();

	createEvents();

	xTaskCreate(I2C_init_task, "I2C_init", 200, NULL,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(UART1_init_task, "UART1_init", 200, NULL,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(UART0_init_task, "UART0 init", 200, NULL,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(Write_EEPROM, "Write_EEPROM", 200, NULL,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(I2C_Write_task, "I2C Write task", 200, NULL,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(sClockLCD_task, "LCD Nokia Print", 200, NULL,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(iReadRTC_task, "Read RTC seconds", 200, NULL,
	configMAX_PRIORITIES - 1, NULL);

	xTaskCreate(Read_EEPROM, "Read_EEPROM", 200, NULL,
	configMAX_PRIORITIES - 1, NULL);

	xTaskCreate(U0_systemMenu_task, "UART0 menu ", 200, NULL,
	configMAX_PRIORITIES - 2, NULL);

	xTaskCreate(U1_systemMenu_task, "UART1 menu ", 200, NULL,
	configMAX_PRIORITIES - 2, NULL);

	xTaskCreate(UART0_readEEPROM_task, "READ TERM 0 ", 200, NULL,
	configMAX_PRIORITIES - 3, NULL);

	xTaskCreate(UART1_readEEPROM_task, "READ TERM 1 ", 200, NULL,
	configMAX_PRIORITIES - 3, NULL);

	xTaskCreate(UART0_PrintEcho_task, "TERM 0 Echo", 200, NULL,
	configMAX_PRIORITIES - 4, NULL);

	xTaskCreate(UART1_PrintEcho_task, "TERM 1 Echo", 200, NULL,
	configMAX_PRIORITIES - 4, NULL);

	vTaskDelete(NULL);
}

/****************************************************************************************************************/
/*	EEPROM Tasks*/

void Read_EEPROM(void * arg) {

	i2c_master_transfer_t masterXfer;
	i2c_master_transfer_t * masterXferPtr;

	EEPROM_mailbox_t eepromData;

	masterXfer.slaveAddress = EEPROM_SLAVE_ADDR;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = &eepromData.stopAddress - &eepromData.startAddress;
	masterXfer.subaddressSize = 2;
	masterXfer.data = &eepromData.data;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	masterXferPtr = pvPortMalloc(sizeof(masterXfer));
	masterXferPtr[0] = masterXfer;

	for (;;) {
		if ( xQueueReceive(get_g_EEPROM_mailbox(), &eepromData,
				portMAX_DELAY) == pdPASS) {

			xQueueSendToBack(get_g_I2C_read_mailbox(), &masterXferPtr,
					pdMS_TO_TICKS(100));

			xQueueReceive(get_g_EEPROM_mailbox(), &eepromData, portMAX_DELAY);

			UART0_putString(eepromData.data);
		}
	}

}

void Write_EEPROM(void * arg) {
	i2c_master_transfer_t masterXfer;
	i2c_master_transfer_t * masterXferPtr;

	uint8_t msg[] = "perro";
//Init I2C master.
	masterXfer.slaveAddress = EEPROM_SLAVE_ADDR;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0x3750;
	masterXfer.subaddressSize = 2;
	masterXfer.data = msg;
	masterXfer.dataSize = sizeof(msg);
	masterXfer.flags = kI2C_TransferDefaultFlag;

	masterXferPtr = pvPortMalloc(sizeof(masterXfer));
	masterXferPtr[0] = masterXfer;

	xQueueSendToBack(get_g_I2C_write_mailbox(), &masterXferPtr,
			pdMS_TO_TICKS(100));

	for (;;) {

		vTaskDelay(pdMS_TO_TICKS(100));

	}
}
/****************************************************************************************************************/
/*	RTC Tasks*/
void iReadRTC_task(void * arg) {

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

	I2C_MasterTransferNonBlocking(I2C0, get_g_m_handle(), &masterXfer);
	xEventGroupWaitBits(get_g_I2C_events(),
	EVENT_I2C_MASTER_TX_COMPLETE, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));

	for (;;) {

		masterXfer.direction = kI2C_Read;

		masterXfer.subaddress = 0x04;

		xSemaphoreTake(mutex_TxRx, portMAX_DELAY);
		I2C_MasterTransferCreateHandle(I2C0_BASEADDR, get_g_m_handle(),
				i2c_master_callback, NULL);
		I2C_MasterTransferNonBlocking(I2C0, get_g_m_handle(), &masterXfer);

		xEventGroupWaitBits(get_g_I2C_events(),
		EVENT_I2C_MASTER_TX_COMPLETE, pdTRUE, pdFALSE, pdMS_TO_TICKS(100));
		xSemaphoreGive(mutex_TxRx);

		xQueueSendToBack(get_g_RTC_mailbox(), masterXfer.data, 10);

		masterXfer.subaddress = 0x03;

		xSemaphoreTake(mutex_TxRx, portMAX_DELAY);

		I2C_MasterTransferNonBlocking(I2C0, get_g_m_handle(), &masterXfer);
		xEventGroupWaitBits(get_g_I2C_events(),
		EVENT_I2C_MASTER_TX_COMPLETE, pdTRUE, pdFALSE, pdMS_TO_TICKS(100));
		xSemaphoreGive(mutex_TxRx);

		xQueueSendToBack(get_g_RTC_mailbox(), masterXfer.data, 10);

		masterXfer.subaddress = 0x02;

		xSemaphoreTake(mutex_TxRx, portMAX_DELAY);

		I2C_MasterTransferNonBlocking(I2C0, get_g_m_handle(), &masterXfer);
		xEventGroupWaitBits(get_g_I2C_events(),
		EVENT_I2C_MASTER_TX_COMPLETE, pdTRUE, pdFALSE, pdMS_TO_TICKS(100));
		xSemaphoreGive(mutex_TxRx);

		xQueueSendToBack(get_g_RTC_mailbox(), masterXfer.data, 10);

		vTaskDelay(pdMS_TO_TICKS(1000));

	}
}

/****************************************************************************************************************/
/*	LCD Tasks*/

void sClockLCD_task(void * arg) {

	RTC_timeUnion_t xTimeReceived;
	uint8_t sendData[10] = { };

	LCDNokia_clear();/*! It clears the information printed in the LCD*/
	LCDNokia_bitmap(getITESO()); /*! It prints an array that hold an image, in this case is the initial picture*/
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

	for (;;) {
		if ( xQueueReceive(get_g_RTC_mailbox(), &xTimeReceived,
				portMAX_DELAY) == pdPASS) {
			// Prepare to send.

			sendData[0] = xTimeReceived.hours.hours_tens + '0';
			sendData[1] = xTimeReceived.hours.hours_units + '0';
			sendData[2] = ':';

			xQueueReceive(get_g_RTC_mailbox(), &xTimeReceived, portMAX_DELAY);

			sendData[3] = xTimeReceived.minutes.minutesTens + '0';
			sendData[4] = xTimeReceived.minutes.minutesUnits + '0';
			sendData[5] = ':';

			xQueueReceive(get_g_RTC_mailbox(), &xTimeReceived, portMAX_DELAY);

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
		vTaskDelay(pdMS_TO_TICKS(500));

	}
}

/****************************************************************************************************************/
/*	Menu functions*/

void UART0_readEEPROM_task(void * arg) {
	uint8_t buffer[QUEUE_EEPROM_LENGTH];
	uint8_t addrCharLength = 0;

	uint16_t realHexAddress = 0;

	uint8_t READ_EEPROM_address[] = { "\r\n\t Introduzca direccion a leer: 0x" };

	uint8_t address_length[] = { "\r\n\t Introduzca bytes a leer: " };

	uint8_t address_data[] = { "\r\n\t" };

	xEventGroupClearBits(get_g_TERM0_events(),
	EVENT_EEPROM_ADDR_FULL);

	for (;;) {

		if (EVENT_UART_READ_EEPROM & xEventGroupGetBits(get_g_TERM0_events())) {
			addrCharLength = 0;
			xEventGroupClearBits(get_g_TERM0_events(), EVENT_UART_READ_EEPROM);
			UART0_putString(getClearScreen());
			UART0_putString(READ_EEPROM_address);
			xEventGroupSetBits(get_g_TERM0_events(), EVENT_EEPROM_GET_ADDR);

		}

		if (EVENT_EEPROM_GET_ADDR & xEventGroupGetBits(get_g_TERM0_events())) {

			if ( xQueueReceive( get_g_TERM0_EEPROM_address(), &buffer[addrCharLength],
					portMAX_DELAY) == pdPASS) {

				if (buffer[addrCharLength] >= '0'
						&& buffer[addrCharLength] <= '9') {
					buffer[addrCharLength] -= 48;
				} else if (buffer[addrCharLength] >= 'A'
						&& buffer[addrCharLength] <= 'F') {
					buffer[addrCharLength] -= 55;
				} else {
					xEventGroupSetBits(get_g_TERM0_events(),
					EVENT_INVALID_CHAR);
				}

				addrCharLength++;
				if (QUEUE_EEPROM_LENGTH == addrCharLength) {
					vTaskDelay(pdMS_TO_TICKS(10));
					addrCharLength = 0;

					realHexAddress = buffer[3] * 1 + buffer[2] * 16
							+ buffer[1] * 16 * 16 + buffer[0] * 16 * 16 * 16;

					if (EVENT_INVALID_CHAR
							& xEventGroupGetBits(get_g_TERM0_events())) {
						xEventGroupClearBits(get_g_TERM0_events(),
						EVENT_INVALID_CHAR);

						UART0_putString(READ_EEPROM_address);

					} else {

						xEventGroupClearBits(get_g_TERM0_events(),
						EVENT_EEPROM_GET_ADDR);
						xEventGroupSetBits(get_g_TERM0_events(),
						EVENT_EEPROM_ADDR_FULL);

					}
				}
			}

		}

		if (EVENT_EEPROM_ADDR_FULL & xEventGroupGetBits(get_g_TERM0_events())) {
			xEventGroupClearBits(get_g_TERM0_events(),
			EVENT_EEPROM_ADDR_FULL);
			xEventGroupSetBits(get_g_I2C_events(),
			EVENT_I2C_START_ADDRESS_READY);
			UART0_putString(address_data);
			xQueueSendToBack(get_g_EEPROM_mailbox(), &realHexAddress, 10);
			//UART0_putString(address_length);

		}

		vTaskDelay(pdMS_TO_TICKS(500));

	}

}

void UART1_readEEPROM_task(void * arg) {
	uint8_t buffer[QUEUE_EEPROM_LENGTH];
	uint8_t addrCharLength = 0;
	uint8_t READ_EEPROM_address[] = { "\r\n\t Introduzca direccion a leer: 0x" };
	uint8_t address_length[] = { "\r\n\t Introduzca bytes a leer: " };

	uint16_t realHexAddress = 0;

	xEventGroupClearBits(get_g_TERM1_events(), EVENT_EEPROM_ADDR_FULL);

	for (;;) {

		if (EVENT_UART_READ_EEPROM & xEventGroupGetBits(get_g_TERM1_events())) {
			xEventGroupClearBits(get_g_TERM1_events(), EVENT_UART_READ_EEPROM);

			UART1_putString(getClearScreen());
			UART1_putString(READ_EEPROM_address);
			xEventGroupSetBits(get_g_TERM1_events(), EVENT_EEPROM_GET_ADDR);

		}

		if (EVENT_EEPROM_GET_ADDR & xEventGroupGetBits(get_g_TERM1_events())) {

			if ( xQueueReceive( get_g_TERM1_EEPROM_address(), &buffer[addrCharLength],
					portMAX_DELAY) == pdPASS) {

				if (buffer[addrCharLength] >= '0'
						&& buffer[addrCharLength] <= '9') {
					buffer[addrCharLength] -= 48;
				} else if (buffer[addrCharLength] >= 'A'
						&& buffer[addrCharLength] <= 'F') {
					buffer[addrCharLength] -= 55;
				} else {
					xEventGroupSetBits(get_g_TERM1_events(),
					EVENT_INVALID_CHAR);
				}

				addrCharLength++;
				if (QUEUE_EEPROM_LENGTH == addrCharLength) {
					vTaskDelay(pdMS_TO_TICKS(10));
					addrCharLength = 0;

					realHexAddress = buffer[3] * 1 + buffer[2] * 16
							+ buffer[1] * 16 * 16 + buffer[0] * 16 * 16 * 16;

					if (EVENT_INVALID_CHAR
							& xEventGroupGetBits(get_g_TERM1_events())) {
						xEventGroupClearBits(get_g_TERM1_events(),
						EVENT_INVALID_CHAR);

						UART0_putString(READ_EEPROM_address);

					} else {

						xEventGroupClearBits(get_g_TERM1_events(),
						EVENT_EEPROM_GET_ADDR);
						xEventGroupSetBits(get_g_TERM1_events(),
						EVENT_EEPROM_ADDR_FULL);

					}
				}
			}

		}

		if (EVENT_EEPROM_ADDR_FULL & xEventGroupGetBits(get_g_TERM1_events())) {
			xEventGroupClearBits(get_g_TERM1_events(),
			EVENT_EEPROM_ADDR_FULL);
			xEventGroupSetBits(get_g_I2C_events(),
			EVENT_I2C_START_ADDRESS_READY);
			//UART1_putString(address_length);
		}

		vTaskDelay(pdMS_TO_TICKS(500));

	}

}
