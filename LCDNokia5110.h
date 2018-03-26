/*
 * LCDNokia5110.h
 *
 *  Created on: Jun 11, 2014
 *      Author: Luis
 */

#ifndef LCDNOKIA5110_H_
#define LCDNOKIA5110_H_

#include "fsl_gpio.h"
#include "fsl_dspi.h"
#include "fsl_port.h"
#include "freeRTOS.h"
#include "task.h"
#include "semphr.h"

typedef enum
{
    first_row = 0,
    second_row,
    third_row,
    fourth_row,
    fifth_row,
    sixth_row

} lcd_row_type_e;

typedef enum
{
    Normal_print = 0,
    Inverse_print

} print_line_type_e;


#define SCREENW 84
#define SCREENH 48

#define LCD_DSPI_MASTER_BASEADDR SPI0
#define LCD_DSPI_SLAVE_BASEADDR SPI1
#define LCD_DSPI_MASTER_IRQ SPI0_IRQn
#define LCD_DSPI_MASTER_IRQHandler SPI0_IRQHandler
#define LCD_DSPI_SLAVE_IRQ SPI1_IRQn
#define LCD_DSPI_SLAVE_IRQHandler SPI1_IRQHandler
#define TRANSFER_BAUDRATE 500000U /*! Transfer baudrate - 500k */
#define TRANSFER_SIZE 1U         /*! Transfer dataSize */

#define LCD_X 84
#define LCD_Y 48
#define LCD_DATA 1
#define LCD_CMD 0
#define LCD_DC_PIN 3
#define LCD_DIN_PIN 2
#define LCD_CLK_PIN 1
#define LCD_RESET_PIN 0

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* DSPI user callback */
void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);
/*It configures all the pins needed on LCD*/
void config_lcd_spi_pins(void);
/*It configures the LCD*/
void LCDNokia_init(void);
/*It writes a byte in the LCD memory. The place of writing is the last place that was indicated by LCDNokia_gotoXY. In the reset state
 * the initial place is x=0 y=0*/
void LCDNokia_writeByte(uint8_t, uint8_t);
/*it clears all the figures in the LCD*/
void LCDNokia_clear(void);
/*It is used to indicate the place for writing a new character in the LCD. The values that x can take are 0 to 84 and y can take values
 * from 0 to 5*/
void LCDNokia_gotoXY(uint8_t x, uint8_t y);
/*It allows to write a figure represented by constant array*/
void LCDNokia_bitmap(const uint8_t*);
/*It write a character in the LCD*/
void LCDNokia_sendChar(uint8_t, uint8_t bw);
/*It write a string into the LCD*/
void LCDNokia_sendString(uint8_t*, uint8_t bw);
/*It used in the initialization routine*/
void LCD_delay(void);
/*It's used to print a line on the LCD without a line padding and format color*/
void printline (print_line_type_e NormalOrInverse, uint8_t * string,
        lcd_row_type_e row);


#endif /* LCDNOKIA5110_H_ */
