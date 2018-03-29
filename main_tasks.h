/*
 * main_tasks.h
 *
 *  Created on: Mar 26, 2018
 *      Author: Cursos
 */

#ifndef MAIN_TASKS_H_
#define MAIN_TASKS_H_

/**********************************************************************************************************************
 * System Tasks
 *********************************************************************************************************************/

/*!
 * @brief This task initialize all drivers configurations
 * 	- SPI
 * 	- UART0 & UART1
 * 	- I2C
 * 	- RTC
 * 	- MEM
 *
 * 	Task is terminated within first execution
 *
 *
 * @param void pointer for input parameters
 */

void systemInit_task(void *arg);


/*!
 * @brief UART0 COM with terminal, all possible tasks listed can be created here.
 * @param void pointer for input parameters
 */

void PrintSystemMenu_task(void *arg);

/**********************************************************************************************************************
 * Menu Tasks
 *********************************************************************************************************************/


/*!
 * @brief UART0 COM with terminal, all possible tasks listed can be created here.
 * @param void pointer for input parameters
 */

void mReadMemory_task(void *arg);

/*!
 * @brief UART0 COM with terminal, all possible tasks listed can be created here.
 * @param void pointer for input parameters
 */

void mWriteMemory_task(void *arg);

/*!
 * @brief UART0 COM with terminal, all possible tasks listed can be created here.
 * @param void pointer for input parameters
 */

void mDefineHour_task(void *arg);

/*!
 * @brief UART0 COM with terminal, all possible tasks listed can be created here.
 * @param void pointer for input parameters
 */

void mDefineDate_task(void *arg);

/*!
 * @brief UART0 COM with terminal, all possible tasks listed can be created here.
 * @param void pointer for input parameters
 */

void mHourFormat_task(void *arg);

/*!
 * @brief UART0 COM with terminal, all possible tasks listed can be created here.
 * @param void pointer for input parameters
 */

void mReadHour_task(void *arg);

/*!
 * @brief UART0 COM with terminal, all possible tasks listed can be created here.
 * @param void pointer for input parameters
 */

void mReadDate_task(void *arg);

/**********************************************************************************************************************
 * SPI Tasks
 *********************************************************************************************************************/

/*!
 * @brief When speaking to the terminal, information is displayed as a local echo through SPI
 * @param void pointer for input parameters
 */

void sInitLCD_task(void *arg);

/*!
 * @brief When speaking to the terminal, information is displayed as a local echo through SPI
 * @param void pointer for input parameters
 */



void sEchoLCD_task(void *arg);

/*!
 * @brief Each second the LCD refresh the hour displayed.
 * @param void pointer for input parameters
 */

 void sItesoLCD_task(void *arg);

/**********************************************************************************************************************
 * UART Tasks
 *********************************************************************************************************************/

 /*!
  * @brief Each second the LCD refresh the hour displayed.
  * @param void pointer for input parameters
  */

 void UART1_PrintHello_task(void * arg);
 /*!
  * @brief Each second the LCD refresh the hour displayed.
  * @param void pointer for input parameters
  */

 void UART0_PrintHello_task(void * arg);

 /*!
  * @brief This function print a character received from queue
  * @param void pointer for input parameters
  */

 void UART0_PrintSeconds_task(void * arg);

 /**********************************************************************************************************************
  * I2C Tasks
  *********************************************************************************************************************/

  /*!
   * @brief Each second the LCD refresh the hour displayed.
   * @param void pointer for input parameters
   */

  void iReadRTC_task(void * arg);








#endif /* MAIN_TASKS_H_ */
