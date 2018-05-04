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
#include "em_timer.h"
#include "em_leuart.h"
#include "LP_BLE.h"




void LEUART0_Init(void)

{

	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFRCO);



	/* Defining the LEUART0 initialization data */

	LEUART_Init_TypeDef leuart0Init =

	{

			.enable   = leuartEnableTx,       /* Activate data reception on LEUn_TX pin. */
			.refFreq  = 0,                    /* Inherit the clock frequenzy from the LEUART clock source */
			.baudrate = 9600,                 /* Baudrate = 9600 bps */
			.databits = leuartDatabits8,      /* Each LEUART frame containes 8 databits */
			.parity   = leuartNoParity,       /* No parity bits in use */
			.stopbits = leuartStopbits2,      /* Setting the number of stop bits in a frame to 2 bitperiods */
	};


	LEUART_Reset(LEUART0);
	LEUART_Init(LEUART0, &leuart0Init);

	while((LEUART0->SYNCBUSY) != 0);


	LEUART0->ROUTE |= LEUART_ROUTE_TXPEN | LEUART_ROUTE_LOCATION_LOC2;

	/* Enable GPIO for BLE_RX. EFM32_TX is on E14(#2) */

  GPIO_PinModeSet(BLE_PORT,                /* GPIO port */

		  	  	  LEUART_TXPIN,                        /* GPIO port number */

                  gpioModePushPull,         /* Pin mode is set to push pull */

                  1);                       /* High idle state */



  /* Enable GPIO for BLE_TX. EFM32_RX is on E15 */

    GPIO_PinModeSet(BLE_PORT,                /* GPIO port */

    				LEUART_RXPIN,                        /* GPIO port number */

                    gpioModePushPull,         /* Pin mode is set to push pull */

                    1);



    GPIO_PinModeSet(BLE_PORT,                /* GPIO port */

    				BLE_POWERPIN,                        /* GPIO port number */

                    gpioModePushPull,         /* Pin mode is set to push pull */

                    0);

    GPIO_PinOutSet(gpioPortE, BLE_POWERPIN);






    LEUART0->CTRL |= LEUART_CTRL_LOOPBK;
    LEUART0->CMD |= LEUART_CMD_RXEN;
  //  LEUART0->IEN = LEUART_IEN_TXBL;
    //blockSleepMode(EM2);
  //  NVIC_EnableIRQ(LEUART0_IRQn);

    int timer_seq;

	for(timer_seq = 1; timer_seq < 12000; timer_seq++)
	{
	  TIMER_Init_TypeDef timer0 =

	    {

	      .enable     = false,                                 // start counting after TIMER_Init is completed
	      .debugRun   = false,                                 // Counter shall keep counting during debug halt
	      .prescale   = timerPrescale1,
	      .clkSel     = timerClkSelHFPerClk,
	      .fallAction = timerInputActionNone,
	      .riseAction = timerInputActionNone,
	      .mode       = timerModeUp,
	      .dmaClrAct  = false,
	      .quadModeX4 = false,
	      .oneShot    = false,
	      .sync       = false,

	    };

	  TIMER_Init(TIMER0, &timer0);
	  TIMER0->CNT = 0X00;
	  TIMER0->CMD = TIMER_CMD_START;
	  while( TIMER0->CNT <= 14000);
	  TIMER0->CMD = TIMER_CMD_STOP;


	  timer_seq++;
	}

	timer_seq = 1;

	LEUART0->TXDATA = 0x78;
	while(!(LEUART0->IF & LEUART_IF_TXC));
	GPIO_PinModeSet(BLE_PORT, BLE_POWERPIN, gpioModeDisabled, 0);
	GPIO_PinOutClear(BLE_PORT, BLE_POWERPIN);



}





/*void LEUART0_IRQHandler(void)

  {

	int flags;
	flags = LEUART0->IF;
	LEUART0->IFC = flags;


	LEUART0->TXDATA = 0x78;
	while(!(LEUART0->IF & LEUART_IF_TXC));

	GPIO_PinModeSet(BLE_PORT, BLE_POWERPIN, gpioModeDisabled, 0);
	GPIO_PinOutClear(BLE_PORT, BLE_POWERPIN);
	NVIC_DisableIRQ(LEUART0_IRQn);

  }
*/


























