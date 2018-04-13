/*
 * app_tasks.h
 *
 *  Created on: Apr 8, 2018
 *      Author: Cursos
 */

#ifndef APP_TASKS_H_
#define APP_TASKS_H_

/****************************************************************************************************************/
/*	Application Layer Tasks terminal info */
/****************************************************************************************************************/

/*!
 * @brief
 * NOTE:
 * 		For all functions described bellow there is a complementary task definition for each terminal
 * 			- TERM0_app_
 * 			- TERM1_app_
 *
 * 		The main tasks to perform an application execution are defined as app_ followed by the task descriptor:
 *
 * 	0) 	app_menu_task:
 * 		-	Local events are declared here.
 * 		-	This task constantly listens to an input character and chooses the next task to
 * 			be created into the task queue.
 *
 * 	1)	app_read_EEPROM_task:
 * 		-	When created a state machine is followed in order to access the memory information.
 * 		- 	First the task displays a msg through the terminal that requested the task. asking for the address to read
 * 		- 	The address will be received through the UART RX queue char by char and converted later on to an hex address
 *
 * 	2) 	app_write_EEPROM_task:
 *
 * 	3)	app_setDate_RTC_task:
 *
 * 	4)	app_setHour_RTC_task:
 *
 * 	5)	app_hourFormat_RTC_task:
 *
 * 	6)	app_readHour_RTC_task:
 *
 * 	7)	app_readDate_RTC_task:
 *
 * 	8)	app_chatBetweenTerms_task:
 *
 * 	9)	app_echo_LCD_task:
 *
 */

/****************************************************************************************************************/
/*	TERM0 tasks declatation */
/****************************************************************************************************************/

void TERM0_app_menu_task(void * arg);

void TERM0_app_read_EEPROM_task(void * arg);

void TERM0_app_write_EEPROM_task(void * arg);

void TERM0_app_setDate_RTC_task(void * arg);

void TERM0_app_setHour_RTC_task(void * arg);

void TERM0_app_hourFormat_RTC_task(void * arg);

void TERM0_app_readHour_RTC_task(void * arg);

void TERM0_app_readDate_RTC_task(void * arg);

void TERM0_app_chatBetweenTerms_task(void * arg);

void TERM0_app_echo_LCD_task(void * arg);

/****************************************************************************************************************/
/*	TERM1 tasks declatation */


void TERM1_app_menu_task(void * arg);

void TERM1_app_read_EEPROM_task(void * arg);

void TERM1_app_write_EEPROM_task(void * arg);

void TERM1_app_setDate_RTC_task(void * arg);

void TERM1_app_setHour_RTC_task(void * arg);

void TERM1_app_hourFormat_RTC_task(void * arg);

void TERM1_app_readHour_RTC_task(void * arg);

void TERM1_app_readDate_RTC_task(void * arg);

void TERM1_app_chatBetweenTerms_task(void * arg);

void TERM1_app_echo_LCD_task(void * arg);

#endif /* APP_TASKS_H_ */
