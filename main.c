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
#include "LP_MAX44009.h"
#include "LP_PIR.h"
//#include "MAX44009.c"
//#include "PIR.c"
//#include "BLE.c"


/***************************************************************************/ /**

  * Sleep routines

 ******************************************************************************* */









/* This code is originally Silicon Labs and copyrighted by Silicon Labs in 2015 and Silicon Labs grants

 * permission to anyone to use the software for any purpose, including commercial applications, and to alter it,

 * and to redistribute it freely subject that the origin is not miss represented, altered source version must be

 * plainly marked, and this notice cannot be altered or removed from any source distribution.

 *

 * Routine include :

 *

 * void blockSleepMode(uint32_t Minimummode)

 * void unblockSleepMode(uint32_t Minimummode)

 * void(Sleep)void

*/



void blockSleepMode(uint32_t Minimummode)	// block an energy mode so that it does not go lower

{

	sleep_block_counter[Minimummode]++;

}



void unblockSleepMode(uint32_t Minimummode)	// unblock an energy mode after operation completes

{

	if (sleep_block_counter[Minimummode]>0)
	{

			sleep_block_counter[Minimummode]--;

	}

	else sleep_block_counter[Minimummode] = 0;

}







void Sleep(void)	// enter EMX sleep routine

{

	if (sleep_block_counter[EM0] > 0){}

	else if (sleep_block_counter[EM1] > 0)

			EMU_EnterEM1();

	else if (sleep_block_counter[EM2] > 0)

			EMU_EnterEM2(true);

	else if (sleep_block_counter[EM3] > 0)

			EMU_EnterEM3(true);

}


void Delay(void)
{

	for(timer_seq = 1; timer_seq < 8000; timer_seq++)
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
	TIMER0->CNT = 0X00;
}









void CMU_Setup(void)

{


	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	CMU_ClockEnable(cmuClock_LEUART0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_I2C0, true);




	if(LETIMER0_EM_Mode == EM3)

	{

		if(CALIBRATION == 1)

		{

			LETIMER0_Calibration();

		}

		CMU_OscillatorEnable(cmuOsc_LFXO, false, false);
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

	}



	else CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);


}


void GPIO_Setup(void)
{

	GPIO_PinModeSet(PORT_PIRSENSOR, PIN_PIRSENSOR, gpioModeInputPullFilter,1);
	GPIO->P[4].DOUTCLR = GPIO->P[4].DOUTCLR | 0x02;
	GPIO_IntConfig(PORT_PIRSENSOR, PIN_PIRSENSOR, true, false, true);
	GPIO_PinModeSet(LED_PORT1, LED_PIN1, gpioModePushPullDrive, 0);
	GPIO_PinModeSet(LED_PORT2, LED_PIN2, gpioModePushPullDrive, 0);



}


void LETIMER0_Calibration(void)

{

// Setup both the timers

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



  TIMER_Init_TypeDef timer1 =

    {
    		.enable     = false,                                 // start counting after TIMER_Init is completed
    		.debugRun   = false,                                 // Counter shall keep counting during debug halt
    		.prescale   = timerPrescale1,
    		.clkSel     = timerClkSelCascade,
    		.fallAction = timerInputActionNone,
    		.riseAction = timerInputActionNone,
    		.mode       = timerModeUp,
    		.dmaClrAct  = false,
    		.quadModeX4 = false,
    		.oneShot    = false,
    		.sync       = false,

    };

  TIMER_Init(TIMER0, &timer0);
  TIMER_Init(TIMER1, &timer1);


//LFXO_COUNT

      TIMER0->CNT = 0x0000;
      TIMER1->CNT = 0x0000;



      LETIMER0_CalInit();


      CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
      CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
      CMU_ClockEnable(cmuClock_LETIMER0, true);


      LETIMER0->CNT = LETIMER0_LFXO_count;

      LETIMER_Enable(LETIMER0, true);

      TIMER0->CMD = TIMER_CMD_START;
      TIMER1->CMD = TIMER_CMD_START;

      while(LETIMER0->CNT != 0);

      TIMER0->CMD = TIMER_CMD_STOP;
      TIMER1->CMD = TIMER_CMD_STOP;



      int a,b,LFXO_COUNT;
      a = TIMER1->CNT;
      b = TIMER0->CNT;
      LFXO_COUNT = ( a << 16 | b );







      // ULFRCO_COUNT

      TIMER0->CNT = 0x0000;

      TIMER1->CNT = 0x0000;



      LETIMER0_CalInit();


      CMU_OscillatorEnable(cmuOsc_LFXO, false, false);
      CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
      CMU_ClockEnable(cmuClock_LETIMER0, true);



      LETIMER0->CNT = LETIMER0_ULFRCO_count;

      LETIMER_Enable(LETIMER0, true);



      TIMER0->CMD = TIMER_CMD_START;
      TIMER1->CMD = TIMER_CMD_START;

      while(LETIMER0->CNT != 0);

      TIMER0->CMD = TIMER_CMD_STOP;
      TIMER1->CMD = TIMER_CMD_STOP;



      int c,d,ULFRCO_COUNT;

      c = TIMER1->CNT;

      d = TIMER0->CNT;



      ULFRCO_COUNT = ( c << 16 | d);
      float osc_ratio;
      osc_ratio = LFXO_COUNT/(float)ULFRCO_COUNT;
      ULFRCO_Calibrated = osc_ratio*LETIMER0_ULFRCO_count;


}


void LETIMER0_CalInit(void)

{

	LETIMER_Init_TypeDef LETIMER0_Calinit;

    LETIMER0_Calinit.bufTop = false;                       // do not load COMP! into COMP0 when REP0 = 0
    LETIMER0_Calinit.comp0Top = true;                      // load COMP0 into CNT on underflow
    LETIMER0_Calinit.debugRun = false;                     // stop counting when debug stops
    LETIMER0_Calinit.enable = false;
    LETIMER0_Calinit.out0Pol = 0;                          // idle output 0
    LETIMER0_Calinit.out1Pol = 1;
    LETIMER0_Calinit.repMode = letimerRepeatOneshot;        // count until stopped by software
    LETIMER0_Calinit.rtcComp0Enable = false;                // do not start counting on COMP0 RTC match
    LETIMER0_Calinit.rtcComp1Enable = false;                // do not start counting on COMP1 RTC match
    LETIMER0_Calinit.ufoa0 = letimerUFOANone;	        // underflow output actions
    LETIMER0_Calinit.ufoa1 = letimerUFOANone;

    LETIMER_Init(LETIMER0, &LETIMER0_Calinit);

}





void LETIMER0_Setup(void)

{

   LETIMER_Init_TypeDef LETIMER0_init;




   // LETIMER initialization

   LETIMER0_init.bufTop = false;                 // do not load COMP! into COMP0 when REP0 = 0
   LETIMER0_init.comp0Top = true;                // load COMP0 into CNT on underflow
   LETIMER0_init.debugRun = false;               // stop counting when debug stops
   LETIMER0_init.enable = false;
   LETIMER0_init.out0Pol = 0;                    // idle output 0
   LETIMER0_init.out1Pol = 1;
   LETIMER0_init.repMode = letimerRepeatFree;    // count until stopped by software
   LETIMER0_init.rtcComp0Enable = false;         // do not start counting on COMP0 RTC match
   LETIMER0_init.rtcComp1Enable = false;         // do not start counting on COMP1 RTC match

   // underflow output actions
   LETIMER0_init.ufoa0 = letimerUFOANone;
   LETIMER0_init.ufoa1 = letimerUFOANone;


   LETIMER_Init(LETIMER0, &LETIMER0_init);
   int Comp0_init;
   int Comp1_int_init;


   // initialize period in ULFRCO for EM3 and LFXO for all other energy modes

   if (LETIMER0_EM_Mode == EM3)

	{

	if(CALIBRATION == 1)

		{

		float Comp1_float_init;

		Comp0_init = LETIMER0_period * ULFRCO_Calibrated;

		Comp1_int_init = LETIMER0_LEDontime * ULFRCO_Calibrated;

		Comp1_float_init = LETIMER0_LEDontime * ULFRCO_Calibrated;



		if(Comp1_float_init > Comp1_int_init)                            /* Compare int and float values of the multiplication for minimum */

		{                                                                 /*  accurate excite time of 4ms*/

			Comp1_int_init++;

		}

		else

		{

			Comp1_int_init +=0;
		}

	}



	else

	{

		CMU_OscillatorEnable(cmuOsc_LFXO, false, false);

		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

		Comp0_init = LETIMER0_period * LETIMER0_ULFRCO_count;

		Comp1_int_init = LETIMER0_LEDontime * LETIMER0_ULFRCO_count ;

	}



   }



else

{

	int LETIMER0_prescalar;
	LETIMER0_prescalar = LETIMER0_period/2;

	Comp0_init = LETIMER0_period * LETIMER0_LFXO_count;

	CMU->LFAPRESC0 &= 0xfffff0ff;	// clear LETIMER bits in LFPRESCO register

	CMU->LFAPRESC0 |= LETIMER0_prescalar <<8 ;

	LETIMER0_prescalar = 1 << LETIMER0_prescalar;



	Comp0_init = LETIMER0_period * (LETIMER0_LFXO_count /LETIMER0_prescalar);

	Comp1_int_init = LETIMER0_LEDontime * (LETIMER0_LFXO_count/LETIMER0_prescalar);



}



LETIMER_CompareSet(LETIMER0,0,Comp0_init);

LETIMER_CompareSet(LETIMER0,1,Comp1_int_init);



while((LETIMER0->SYNCBUSY) != 0);                            	//wait till synchronization bit goes low

	// setting corresponding flag bits in LETIMER_IEN

LETIMER0->IEN = LETIMER_IEN_UF | LETIMER_IEN_COMP1;
blockSleepMode(LETIMER0_EM_Mode);
NVIC_EnableIRQ(LETIMER0_IRQn);



}


void LETIMER0_IRQHandler(void)

{



	int currentFlags;
	currentFlags = LETIMER0->IF;                                                          // save IF register contents/state in a variable
	LETIMER0->IFC = currentFlags;


	if ((currentFlags & LETIMER_IF_COMP1) != 0)
	{


	}


	else if ((currentFlags & LETIMER_IF_UF) != 0)

	{

		NVIC_EnableIRQ(GPIO_ODD_IRQn);



	}

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

  }*/


void GPIO_ODD_IRQHandler(void)
{
	INT_Disable();
	GPIO_IntClear(0x0002);


	if(PIRint_number == 1)
	{
		GPIO_PinOutSet(LED_PORT1, LED_PIN1);
		GPIO_IntConfig(PORT_PIRSENSOR, PIN_PIRSENSOR, true, false, false);


		MAXI2C_Setup();
		MAXI2C_Transfer();

		//timer for LPM of BLE


	if( BLE_ACTIVE_LIGHTSON == true)
	{
		Delay();

		GPIO_PinModeSet(BLE_PORT, LEUART_TXPIN, gpioModeDisabled, 0);					// module to work properly.
		GPIO_PinOutClear(BLE_PORT, LEUART_TXPIN);
		GPIO_PinModeSet(BLE_PORT, LEUART_RXPIN, gpioModeDisabled, 0);
		GPIO_PinOutClear(BLE_PORT, LEUART_RXPIN);
	}

	PIRint_number = 2;
//	GPIO->P[4].DOUTCLR = GPIO->P[4].DOUTCLR | 0x02;

	GPIO_PinOutClear(LED_PORT1, LED_PIN1);
	GPIO_IntConfig(PORT_PIRSENSOR, PIN_PIRSENSOR, true, false, true);




	}

	else if(PIRint_number == 2)
	{
		GPIO_IntConfig(PORT_PIRSENSOR, PIN_PIRSENSOR, true, false, false);

		if(BLE_ACTIVE_LIGHTSON == true)
		{
		GPIO_PinOutSet(LED_PORT1, LED_PIN1);


			Delay();




   			LEUART0_Init();

   			Delay();


   			GPIO_PinModeSet(BLE_PORT, LEUART_TXPIN, gpioModeDisabled, 0);					// module to work properly.
   			GPIO_PinOutClear(BLE_PORT, LEUART_TXPIN);
   			GPIO_PinModeSet(BLE_PORT, LEUART_RXPIN, gpioModeDisabled, 0);
   			GPIO_PinOutClear(BLE_PORT, LEUART_RXPIN);

   			BLE_ACTIVE_LIGHTSON = false;
   			GPIO_PinOutClear(LED_PORT2, LED_PIN2);
   	//		GPIO->P[4].DOUTCLR = GPIO->P[4].DOUTCLR | 0x02;
   			GPIO_PinOutClear(LED_PORT1, LED_PIN1);

   			GPIO_IntConfig(PORT_PIRSENSOR, PIN_PIRSENSOR, true, false, true);





		}

		PIRint_number = 1;
	}



	INT_Enable();
}





int main(void)

{

  /* Chip errata */

  CHIP_Init();

  global = 1;

  blockSleepMode(EM3);



  CMU_Setup();
  GPIO_Setup();
  LETIMER0_Setup();

  LETIMER_Enable(LETIMER0, true);





  /* Infinite loop */

  while (1)
  {

	  Sleep();

  }

}


