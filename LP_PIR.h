/*
 * LP_PIR.h
 *
 *  Created on: Apr 24, 2017
 *      Author: niharikasarode
 */

#ifndef LP_PIR_H_
#define LP_PIR_H_
#define EM0 0
#define EM1 1
#define EM2 2
#define EM3 3
void CMU_Setup(void);
void GPIO_Setup(void);
void Sleep(void);
void blockSleepMode(uint32_t Minimummode);
void unblockSleepMode(uint32_t Minimummode);

#define PORT_PIRSENSOR  gpioPortE
#define PIN_PIRSENSOR		1

int PIRint_number=1;
int timer_seq = 1;
bool BLE_ACTIVE ;
#endif /* LP_PIR_H_ */
