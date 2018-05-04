/*
 * LP_MAX44009.h
 *
 *  Created on: Apr 23, 2017
 *      Author: niharikasarode
 */

#ifndef LP_MAX44009_H_
#define LP_MAX44009_H_

void MAXI2C_Setup(void);
void MAXI2C_Transfer(void);
void CMU_Setup(void);
void ClearACK(void);

#define LED_PORT1 gpioPortC
#define LED_PORT2 gpioPortE
#define LED_PIN1     9
#define LED_PIN2     6

#define MAX_I2C_Port   gpioPortA
#define MAX_I2C_SDApin     0
#define MAX_I2C_SCLpin     1

#define MAX_ADDRESS  0x94
#define WRITE         0
#define READ 		  1
#define CONFIGREGISTER_DATA   0x00

bool BLE_ACTIVE_LIGHTSON ;


enum MAX_REGISTERADDRESS{
	INTERRUPT_STATUS = 0x00,
	INTERRUPT_ENABLE = 0x01,
	CONFIGURATION = 0x02,
	LUXHIGHBYTE = 0x03,
	LUXLOWBYTE = 0x04,
	UPPERTHRESHOLD_HIHGBYTE = 0x05,
	LOWERTHRESHOLD_HIGHBYTE = 0x06,
	THRESHOLDTIMER = 0x07
};





#endif /* LP_MAX44009_H_ */
