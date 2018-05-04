/**************************************************************************//**
 * @file
 * @brief Empty Project
 * @author Energy Micro AS
 * @version 3.20.2
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"
 * for details. Before using this software for any purpose, you must agree to the
 * terms of that agreement.
 *
 ******************************************************************************/
//#include "em_device.h"
//#include "em_chip.h"

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/


#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_acmp.h"
#include "em_timer.h"
#include "em_dma.h"
#include "em_adc.h"
#include "dmactrl.h"
#include "em_i2c.h"
#include "LP_MAX44009.h"





void MAXI2C_Setup(void)
{
	I2C_Init_TypeDef i2cinit = I2C_INIT_DEFAULT;
	I2C_Init(I2C0, &i2cinit);

	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C0, true);


	GPIO_PinModeSet(MAX_I2C_Port, MAX_I2C_SCLpin, gpioModeWiredAndPullUpFilter, 1);				//SCL
	GPIO_PinModeSet(MAX_I2C_Port, MAX_I2C_SDApin, gpioModeWiredAndPullUpFilter, 1);				//SDA

	for (int i = 0; i < 9; i++)
		  {
		    /* TBD: Seems to be clocking at appr 80kHz-120kHz depending on compiler
		     * optimization when running at 14MHz. A bit high for standard mode devices,
		     * but DVK only has fast mode devices. Need however to add some time
		     * measurement in order to not be dependable on frequency and code executed.
		     */
		    GPIO_PinModeSet(MAX_I2C_Port, MAX_I2C_SCLpin, gpioModeWiredAnd, 0);
		    GPIO_PinModeSet(MAX_I2C_Port, MAX_I2C_SCLpin, gpioModeWiredAnd, 1);

		  }

	I2C0->ROUTE = I2C_ROUTE_SDAPEN |										// route pins used for I2c buses SDA and SCL
	    	              I2C_ROUTE_SCLPEN |
	    	       (I2C_ROUTE_LOCATION_LOC0);

	if(I2C0->STATE & I2C_STATE_BUSY){										// reset registers and device
	    	I2C0->CMD = I2C_CMD_ABORT ;
	    	}



}

void ClearACK(void)
{
    while(! (I2C0->IF & I2C_IF_ACK));
	int fl = (I2C0->IF & I2C_IF_ACK);
	I2C0->IFC = fl;
}


void MAXI2C_Transfer(void)
{

		I2C0->TXDATA = MAX_ADDRESS | WRITE;
		I2C0->CMD = I2C_CMD_START;


	   ClearACK();

	   I2C0->TXDATA = CONFIGURATION;
	   ClearACK();

	   I2C0->TXDATA = CONFIGREGISTER_DATA;
	   ClearACK();

	   I2C0->CMD = I2C_CMD_STOP;

	   I2C0->TXDATA = MAX_ADDRESS | WRITE;
	   I2C0->CMD = I2C_CMD_START;
	   ClearACK();

	   I2C0->TXDATA = LUXHIGHBYTE;
	   ClearACK();

	   I2C0->CMD = I2C_CMD_START;
	   I2C0->TXDATA = MAX_ADDRESS | READ;
	   ClearACK();

	   while (!(I2C0->IF & I2C_IF_RXDATAV));						// start reading
	   int tsl_txBuffer2 = I2C0->RXDATA;							// low : 4 high : 10 for normal
	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	    // low : 8 high 150 for flashlight ON
	   	I2C0->CMD = I2C_CMD_NACK;
	   	I2C0->CMD = I2C_CMD_STOP;




	   		   if (tsl_txBuffer2 > 150)								// check for brightness condition
	   		   {
	   			   GPIO_PinOutSet(LED_PORT1, LED_PIN1);
	   		   }

	   		   else if (tsl_txBuffer2 < 150)							// check for darkness condition
	   		   {
	   			GPIO_PinOutSet(LED_PORT2, LED_PIN2);


	   			LEUART0_Init();
	   			BLE_ACTIVE_LIGHTSON = true;


	   		   }



}




