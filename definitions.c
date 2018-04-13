/*
 * definitions.c
 *
 *  Created on: Apr 9, 2018
 *      Author: Cursos
 */
#include "definitions.h"

EventGroupHandle_t g_TERM0_events;

EventGroupHandle_t g_TERM1_events;

EventGroupHandle_t g_UART_events;

EventGroupHandle_t g_I2C_events;

QueueHandle_t g_TERM0_EEPROM_address;

QueueHandle_t g_TERM1_EEPROM_address;

QueueHandle_t g_UART0_mailbox;

QueueHandle_t g_UART1_mailbox;

QueueHandle_t g_UART1_Chat_mailbox;

QueueHandle_t g_UART0_Chat_mailbox;

QueueHandle_t g_RTC_mailbox;

uart_handle_t g_uart0Handle;

uart_handle_t g_uart1Handle;


void createEvents() {

	g_TERM0_events = xEventGroupCreate();

	g_TERM1_events = xEventGroupCreate();

	g_UART_events = xEventGroupCreate();

	g_I2C_events = xEventGroupCreate();

	g_TERM0_EEPROM_address = xQueueCreate(QUEUE_EEPROM_LENGTH, QUEUE_ITEM_SIZE);

	g_TERM1_EEPROM_address = xQueueCreate(QUEUE_EEPROM_LENGTH, QUEUE_ITEM_SIZE);

	g_UART0_mailbox = xQueueCreate(QUEUE_LENGTH_UART, QUEUE_ITEM_SIZE);

	g_UART1_mailbox = xQueueCreate(QUEUE_LENGTH_UART, QUEUE_ITEM_SIZE);

	g_UART1_Chat_mailbox =xQueueCreate(QUEUE_LENGTH_UART, QUEUE_ITEM_SIZE);

	g_UART0_Chat_mailbox =xQueueCreate(QUEUE_LENGTH_UART, QUEUE_ITEM_SIZE);

	g_RTC_mailbox = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);

}

QueueHandle_t get_g_TERM0_EEPROM_address() {
	return g_TERM0_EEPROM_address;
}

QueueHandle_t get_g_TERM1_EEPROM_address() {
	return g_TERM1_EEPROM_address;
}

QueueHandle_t get_g_UART0_mailbox() {
	return g_UART0_mailbox;
}

QueueHandle_t get_g_UART1_mailbox() {
	return g_UART1_mailbox;
}

QueueHandle_t get_g_UART1_Chat_mailbox() {
	return g_UART1_Chat_mailbox;
}

QueueHandle_t get_g_UART0_Chat_mailbox() {
	return g_UART0_Chat_mailbox;
}


QueueHandle_t get_g_RTC_mailbox() {
	return g_RTC_mailbox;
}


EventGroupHandle_t get_g_TERM0_events() {
	return g_TERM0_events;
}

EventGroupHandle_t get_g_TERM1_events() {
	return g_TERM1_events;
}

EventGroupHandle_t get_g_UART_events() {
	return g_UART_events;
}

EventGroupHandle_t get_g_I2C_events() {
	return g_I2C_events;
}


uart_handle_t * get_g_uart0Handle() {
	return &g_uart0Handle;
}

uart_handle_t * get_g_uart1Handle(){
	return &g_uart1Handle;

}


