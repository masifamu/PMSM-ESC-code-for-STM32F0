#ifndef _PMSM_FUNC_LIB_H_
#define _PMSM_FUNC_LIB_H_

#include "string.h"
#include "stdint.h"
#include "stdio.h"

#define ENABLE_UART_DEBUG

#define PWM_PERIOD 					480 //8Mhz/(pwmfre*prescalar)

#define PMSM_ADC_START 			200
#define PMSM_ADC_STOP 			50
#define PMSM_ADC_MAX 				4000

#define PMSM_CW							0
#define PMSM_CCW						1

//#ifdef ENABLE_UART_DEBUG
//uint8_t stringToUART[50] = {'\0',};
//#endif
void sendToUART(char *);
void PMSM_SetPWMWidthToYGB(uint16_t mThrottle);
void PMSM_startPWMToYGB(uint16_t mappedThrottle);
void PMSM_setPWMFreq(uint16_t sfreq);
uint16_t PMSM_getPWMFreq(uint16_t gfreq);
uint16_t PMSM_ADCToPWM(uint16_t ADC_VALUE);
uint8_t PMSM_HallSensorsGetPosition(void);
uint8_t	PMSM_GetState(uint8_t SensorsPosition);
void PMSM_MotorSetSpin(uint8_t spin); 
#endif
