/*
 * RTC.c
 *
 *  Created on: 06/11/2017
 *      Author: Oscar
 */

#include "RTC.h"

uint16_t leapYear = 0;

uint16_t getLeapYear() {
	return leapYear;
}
void setLeapYear(uint16_t year) {
	leapYear = year / 4;
}

BCD_struct yearToBCD(RTC_timeStruct_t * time) {
	BCD_struct yearBCD;
	uint16_t auxLeapYear = getLeapYear();
	uint16_t auxYear = (auxLeapYear * 4) + time->yearAndDate.yearAndDate.year;
	yearBCD.units = auxYear % 10;
	yearBCD.tens = (auxYear / 10) % 10;
	yearBCD.hundreds = (auxYear / 100) % 10;
	yearBCD.thousands = (auxYear / 1000) % 10;
	return yearBCD;
}

#ifdef I2C
uint8_t RTC_singleRead(uint8_t BUSaddress, RTC_registers_t timeRegister) {
	uint8_t data;
	I2C_start(I2C0);

	I2C_writeByte(I2C0, BUSaddress);
	I2C_wait(I2C0);
	I2C_getACK(I2C0);
	delay(600);		//Compensate for the missing clock cycle for RX_ACK

	I2C_writeByte(I2C0, timeRegister);
	I2C_wait(I2C0);
	I2C_getACK(I2C0);
	delay(600);		//Compensate for the missing clock cycle for RX_ACK

	I2C_repeatedStart(I2C0);
	I2C_writeByte(I2C0, (BUSaddress | READ));
	I2C_wait(I2C0);
	I2C_getACK(I2C0);
	delay(600);		//Compensate for the missing clock cycle for RX_ACK

	I2C_TXRXMode(I2C0, RECEIVE);

	I2C_NACK(I2C0);
	data = I2C_readByte(I2C0);
	I2C_wait(I2C0);
	delay(600);		//Compensate for the missing clock cycle for RX_ACK

	I2C_stop(I2C0);
	I2C_busy(I2C0);		//Wait until the bus is done
	data = I2C_readByte(I2C0);

	return data;
}

void RTC_singleWrite(uint8_t BUSaddress, RTC_registers_t timeRegister,
		uint8_t dataToWrite) {
	I2C_start(I2C0);

	I2C_writeByte(I2C0, BUSaddress);
	I2C_wait(I2C0);
	I2C_getACK(I2C0);
	delay(600);		//Compensate for the missing clock cycle for RX_ACK

	I2C_writeByte(I2C0, timeRegister);
	I2C_wait(I2C0);
	I2C_getACK(I2C0);
	delay(600);		//Compensate for the missing clock cycle for RX_ACK

	I2C_writeByte(I2C0, dataToWrite);
	I2C_wait(I2C0);
	I2C_getACK(I2C0);
	delay(600);		//Compensate for the missing clock cycle for RX_ACK

	I2C_stop(I2C0);
	I2C_busy(I2C0);		//Wait until the bus is done
}

RTC_timeStruct_t RTC_timeRead(uint8_t BUSaddress) {
	RTC_timeStruct_t time;
	time.hundreds.data = RTC_singleRead(BUSaddress, HUNDREDS);
	time.seconds.data = RTC_singleRead(BUSaddress, SECONDS);
	time.minutes.data = RTC_singleRead(BUSaddress, MINUTES);
	time.hours.data = RTC_singleRead(BUSaddress, HOURS);
	time.monthsAndWeekday.data = RTC_singleRead(BUSaddress, WEEKDAYS_MONTH);
	time.yearAndDate.data = RTC_singleRead(BUSaddress, YEAR_DATE);
	time.yearBCD.yearBCD = yearToBCD(&time);
	return time;
}

void RTC_setTime(uint8_t BUSaddress, RTC_timeStruct_t * time) {
	RTC_singleWrite(BUSaddress, HUNDREDS, time->hundreds.data);
	RTC_singleWrite(BUSaddress, SECONDS, time->seconds.data);
	RTC_singleWrite(BUSaddress, MINUTES, time->minutes.data);
	RTC_singleWrite(BUSaddress, HOURS, time->hours.data);
	RTC_singleWrite(BUSaddress, WEEKDAYS_MONTH, time->monthsAndWeekday.data);
	RTC_singleWrite(BUSaddress, YEAR_DATE, time->yearAndDate.data);
}

void RTC_printWeekday(RTC_timeStruct_t time) {
	switch (time.monthsAndWeekday.monthsAndWeekday.weekday) {
	case MONDAY:
		LCDNokia_sendString("Monday");
		break;
	case TUESDAY:
		LCDNokia_sendString("Tuesday");
		break;
	case WEDNESDAY:
		LCDNokia_sendString("Wednesday");
		break;
	case THURSDAY:
		LCDNokia_sendString("Thursday");
		break;
	case FRIDAY:
		LCDNokia_sendString("Friday");
		break;
	case SATURDAY:
		LCDNokia_sendString("Saturday");
		break;
	case SUNDAY:
		LCDNokia_sendString("Sunday");
		break;
	default:
		break;
	}
}

void RTC_printTime(RTC_timeStruct_t time) {
	LCDNokia_clear();
	LCDNokia_gotoXY(17, 0); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendString("Time:"); /*! It print a string stored in an array*/
	/*!Print weekday by separate*/

	LCDNokia_gotoXY(5, 2);
	RTC_printWeekday(time);
	/*!Print date with MM/DD/YYYY format*/
	LCDNokia_gotoXY(5, 3);
	LCDNokia_sendChar(time.monthsAndWeekday.monthsAndWeekday.months_tens + '0');
	LCDNokia_sendChar(
			time.monthsAndWeekday.monthsAndWeekday.months_units + '0');
	LCDNokia_sendChar('/');
	LCDNokia_sendChar(time.yearAndDate.yearAndDate.days_tens + '0');
	LCDNokia_sendChar(time.yearAndDate.yearAndDate.days_units + '0');
	LCDNokia_sendChar('/');
	LCDNokia_sendChar(time.yearBCD.yearBCD.thousands + '0');
	LCDNokia_sendChar(time.yearBCD.yearBCD.hundreds + '0');
	LCDNokia_sendChar(time.yearBCD.yearBCD.tens + '0');
	LCDNokia_sendChar(time.yearBCD.yearBCD.units + '0');
	/*!Print hour with hh:mm:ss format*/
	LCDNokia_gotoXY(5, 4);
	LCDNokia_sendChar(time.hours.hours.hours_tens + '0');
	LCDNokia_sendChar(time.hours.hours.hours_units + '0');
	LCDNokia_sendChar(':');
	LCDNokia_sendChar(time.minutes.minutes.minutesTens + '0');
	LCDNokia_sendChar(time.minutes.minutes.minutesUnits + '0');
	LCDNokia_sendChar(':');
	LCDNokia_sendChar(time.seconds.seconds.secondsTens + '0');
	LCDNokia_sendChar(time.seconds.seconds.secondsUnits + '0');

	/*!Print AM/PM if flag is valid*/
	if (_12_HOURS == time.hours.hours.format) {
		if (AM == time.hours.hours.AMorPM_flag) {
			LCDNokia_sendString(" AM");
		} else {
			LCDNokia_sendString(" PM");
		}
	}
}

uint8_t setYear(uint16_t year, RTC_timeStruct_t * time) {
	setLeapYear(year);
	uint8_t auxData;
	uint8_t auxMask = 0xC0;

	if (year % 4 == 0) {
		time->yearAndDate.yearAndDate.year = 0;
		auxData = RTC_singleRead(0xA0, YEAR_DATE);
		auxData &= ~(auxMask);
		RTC_singleWrite(0xA0, YEAR_DATE, (auxData | 0));

		return 0;
	}

	year--;

	if (year % 4 == 0) {
		time->yearAndDate.yearAndDate.year = 1;
		auxData = RTC_singleRead(0xA0, YEAR_DATE);
		auxData &= ~(auxMask);

		RTC_singleWrite(0xA0, YEAR_DATE, (auxData | 0x40));
		return 1;
	}

	year--;

	if (year % 4 == 0) {
		time->yearAndDate.yearAndDate.year = 2;
		auxData = RTC_singleRead(0xA0, YEAR_DATE);
		auxData &= ~(auxMask);

		RTC_singleWrite(0xA0, YEAR_DATE, (auxData | 0x80));
		return 2;
	}

	year--;

	if (year % 4 == 0) {
		time->yearAndDate.yearAndDate.year = 3;
		auxData = RTC_singleRead(0xA0, YEAR_DATE);
		auxData &= ~(auxMask);
		RTC_singleWrite(0xA0, YEAR_DATE, (auxData | 0xC0));

		return 3;
	}
	return 0;
}

#endif
