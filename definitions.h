/*
 * definitions.h
 *
 *  Created on: Apr 9, 2018
 *      Author: Cursos
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include "char_arrays.h"
#include "LCDNokia5110.h"
#include "fsl_uart.h"
#include "fsl_i2c.h"
#include "fsl_debug_console.h"
#include "MK64F12.h"

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "main_tasks.h"
#include "FreeRTOS.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"
#include "queue.h"
#include "RTC.h"

typedef struct {
	uint8_t * data;
	uint16_t startAddress;
	uint16_t stopAddress;

} EEPROM_mailbox_t;

/****************************************************************************************************************/
/*	Local definitions*/

#define ESCAPE_KEY_CELL 127
#define ESCAPE_KEY 0x1B
#define ENTER_KEY 0x0D

#define I2C0_BASEADDR I2C0
#define RTC_slave_address 0x51
#define EEPROM_SLAVE_ADDR 0x50
#define I2C_MASTER_CLK I2C0_CLK_SRC
#define BUFFER_SIZE 2

#define QUEUE_EEPROM_LENGTH 4
#define QUEUE_LENGTH 3
#define QUEUE_LENGTH_UART 1
#define QUEUE_ITEM_SIZE sizeof(uint8_t)
#define QUEUE_EEPROM_ITEM_SIZE sizeof(EEPROM_mailbox_t)
#define QUEUE_I2C_ITEM_SIZE sizeof(i2c_master_transfer_t)

/****************************************************************************************************************/
/*	Event definitions*/

/* READ EEPROM STATE MACHINE*/

#define EVENT_UART_ECHO (1<<0)
#define EVENT_UART_READ_EEPROM (1<<1)
#define EVENT_UART_WRITE_EEPROM (1<<2)
#define EVENT_UART_SET_HOUR (1<<3)
#define EVENT_UART_SET_DATE (1<<4)
#define EVENT_UART_HOUR_FORMAT (1<<5)
#define EVENT_UART_READ_HOUR (1<<6)
#define EVENT_UART_READ_DATE (1<<7)
#define EVENT_UART_RX (1<<8)
#define EVENT_MENU_WAIT (1<<9)
#define EVENT_INVALID_CHAR (1<<10)
#define EVENT_CHAR_SENT (1<<11)
#define EVENT_EEPROM_READ (1<<12)
#define EVENT_EEPROM_GET_ADDR  (1<<13)
#define EVENT_EEPROM_ADDR_FULL (1<<14)
#define EVENT_EEPROM_GET_LENGTH (1<<15)
#define EVENT_EEPROM_START_I2C_READ (1<<16)
#define EVENT_EEPROM_WAIT (1<<17)
#define EVENT_CHAT_TASK_ON (1 << 18)
#define EVENT_ESC_OVR (1<<19)
#define UART_CHAT_ENTER_KEY (1<<20)

#define EVENT_UART0_TX (1<<0)
#define EVENT_UART0_RX (1<<0)
#define EVENT_UART1_TX (1<<2)
#define EVENT_UART1_RX (1<<2)

#define EVENT_I2C_MASTER_TX_COMPLETE (1<<0)
#define EVENT_I2C_START_ADDRESS_READY (1<<1)
#define EVENT_I2C_END_ADDRESS_READY (1<<2)

#define EVENT_I2C_MASTER_TX_COMPLETE (1<<0)
#define EVENT_I2C_START_ADDRESS_READY (1<<1)
#define EVENT_I2C_END_ADDRESS_READY (1<<2)

/****************************************************************************************************************/
/*	Global variables getters */

uint8_t toUpperCase(uint8_t * data);

uart_handle_t * get_g_uart0Handle();

uart_handle_t * get_g_uart1Handle();

i2c_master_handle_t * get_g_m_handle();

void createEvents();

void createQueues();

EventGroupHandle_t get_g_TERM0_events();

EventGroupHandle_t get_g_TERM1_events();

EventGroupHandle_t get_g_UART_events();

EventGroupHandle_t get_g_I2C_events();

QueueHandle_t get_g_RTC_mailbox();

QueueHandle_t get_g_EEPROM_mailbox();

QueueHandle_t get_g_TERM0_EEPROM_address();

QueueHandle_t get_g_TERM1_EEPROM_address();

QueueHandle_t get_g_UART0_mailbox();

QueueHandle_t get_g_UART0_Chat_mailbox();

QueueHandle_t get_g_UART1_Chat_mailbox();

QueueHandle_t get_g_UART1_mailbox();

QueueHandle_t get_g_I2C_write_mailbox();

QueueHandle_t get_g_I2C_read_mailbox();

#endif /* DEFINITIONS_H_ */
