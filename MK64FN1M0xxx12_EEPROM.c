/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    MK64FN1M0xxx12_EEPROM.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
#include "Task.h"

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

#define I2C0_BASEADDR I2C0
#define RTC_slave_address 0x51
#define I2C_MASTER_CLK I2C0_CLK_SRC
#define BUFFER_SIZE 2
#define EEPROM_SLAVE_ADDR 0x50

/*
 * @brief   Application entry point.
 */

//TaskHandle_t Handle_I2CInit;
volatile bool g_MasterCompletionFlag = false;
static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void *userData) {
	//Signal transfer success when received success status.
	if (status == kStatus_Success) {
		g_MasterCompletionFlag = true;
	}
}

void Read_EEPROM(void * arg) {

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

	    uint8_t read_buffer;
		masterXfer.slaveAddress = EEPROM_SLAVE_ADDR;
		masterXfer.direction = kI2C_Read;
		masterXfer.subaddress = 0x00FF;
		masterXfer.subaddressSize = 2;
		masterXfer.data = &read_buffer;
		masterXfer.dataSize = 1;
		masterXfer.flags = kI2C_TransferDefaultFlag;


		for (;;) {

			I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
						while (!g_MasterCompletionFlag) {}
							g_MasterCompletionFlag = false;
							PRINTF("%c\r", read_buffer);

			vTaskDelay(pdMS_TO_TICKS(1000));

		}

}



void Write_EEPROM(void * arg) {
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


	uint8_t data_buff = 'a';

	//Init I2C master.
	masterXfer.slaveAddress = EEPROM_SLAVE_ADDR;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0x00FF;
	masterXfer.subaddressSize = 2;
	masterXfer.data = &data_buff;
	masterXfer.dataSize = 1;
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




int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();


//    xTaskCreate(I2C_init, "I2C_init", 200, NULL, configMAX_PRIORITIES - 1, NULL);


    xTaskCreate(Write_EEPROM, "Write_EEPROM", 200, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(Read_EEPROM, "Read_EEPROM", 200, NULL, configMAX_PRIORITIES - 2, NULL);


    vTaskStartScheduler();

    while(1) {

    }
    return 0 ;
}
