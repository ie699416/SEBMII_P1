/*
 * LCDNokia5110.h
 *
 *  Created on: Jun 11, 2014
 *      Author: Luis
 */

#ifndef LCDNOKIA5110_H_
#define LCDNOKIA5110_H_

#define SCREENW 84
#define SCREENH 48

#define LCD_X 84
#define LCD_Y 48
#define LCD_DATA 1
#define LCD_CMD 0
#define DATA_OR_CMD_PIN 3
#define RESET_PIN 0

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
void LCDNokia_sendChar(uint8_t);
/*It write a string into the LCD*/
void LCDNokia_sendString(uint8_t*);
/*It used in the initialization routine*/
void LCD_delay(void);



#endif /* LCDNOKIA5110_H_ */
