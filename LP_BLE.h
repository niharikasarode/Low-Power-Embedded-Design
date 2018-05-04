/*
 * LP_BLE.h
 * 						Connections:  	EFM32				Adafruit BLE UART Friend
 * 										E14(TX)  :			BLE_RX
 * 										E15(RX)  :			BLE_TX
 * 										E12		 :			BLE_Vin
 * 										GND		 :			CTS
 * 										GND		 :			GND
 *
 *
 *  Created on: Apr 23, 2017
 *      Author: niharikasarode
 */

#ifndef LP_BLE_H_
#define LP_BLE_H_



void Sleep(void);
void blockSleepMode(uint32_t Minimummode);
void unblockSleepMode(uint32_t Minimummode);
void CMU_Setup(void);
void LETIMER0_Setup(void);
void LETIMER0_Calibration(void);
void LETIMER0_CalInit(void);
void LEUART0_Init(void);


#define EM0 0
#define EM1 1
#define EM2 2
#define EM3 3
#define EM4 4

#define LETIMER0_EM_Mode 			3
#define LETIMER0_period 			4
#define LETIMER0_ULFRCO_count 		1000
#define LETIMER0_LFXO_count 		32768
#define LETIMER0_LEDontime	        1              // excitation time
#define CALIBRATION   				1


#define	BLE_PORT					gpioPortE
#define LEUART_TXPIN				14
#define LEUART_RXPIN				15
#define BLE_POWERPIN				12






uint32_t sleep_block_counter[EM4+1];
int inter, ULFRCO_Calibrated,global;



#endif /* LP_BLE_H_ */
