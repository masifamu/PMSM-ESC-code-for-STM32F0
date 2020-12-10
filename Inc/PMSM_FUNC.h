#ifndef _PMSM_FUNC_LIB_H_
#define _PMSM_FUNC_LIB_H_

#include "string.h"
#include "stdint.h"
#include "stdio.h"

#define ENABLE_UART_DEBUG
//#define ENABLE_THROTTLE

#define HS_PINS 						GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7

#define PWM_PERIOD 					2884 //48Mhz/(pwmfre*prescalar)
#define LOOKUP_ENTRIES			512
#define M_PI								3.14159265358979323846

#ifndef ENABLE_THROTTLE
#define PMSM_ADC_START 			200
#define PMSM_ADC_STOP 			50
#define PMSM_ADC_MAX 				4000
#endif

#ifdef ENABLE_THROTTLE
#define PMSM_ADC_START 			1150
#define PMSM_ADC_STOP 			1090
#define PMSM_ADC_MAX 				4000
#endif

#define SPEEDING_FACTOR			0.8

#define PMSM_CW							0
#define PMSM_CCW						1

#define PMSM_MODE_ENABLED		1
#define PMSM_MODE_DISABLED  2

#define UH	0
#define UL	1
#define VH	2
#define VL	3
#define WH	4
#define WL	5

#define CH1 1
#define CH2 2
#define CH3 3

#define PMSM_TIMER14_PRESCALER		48
#define PMSM_TIMER14_PERIOD			0xFFFF // 65535

//#ifdef ENABLE_UART_DEBUG
//uint8_t stringToUART[50] = {'\0',};
//#endif
void sendToUART(char *);
void PMSM_SetPWMWidthToYGB(uint8_t val);
void PMSM_startPWMToYGB(void);
void PMSM_setPWMFreq(uint16_t sfreq);
uint16_t PMSM_getPWMFreq(uint16_t gfreq);
void PMSM_updatePMSMPWMVariable(uint16_t PWM);
uint16_t PMSM_ADCToPWM(uint16_t ADC_VALUE);
uint8_t PMSM_HallSensorsGetPosition(void);
void PMSM_MotorSetSpin(uint8_t spin); 

void PMSM_MotorStop(void);
void PMSM_MotorSetRun(void);
uint16_t PMSM_GetSpeed(void);
uint8_t PMSM_MotorIsRun(void);
void PMSM_Init(void);
void BLDC_MotorCommutation(uint16_t hallpos);

uint16_t PMSM_setFreq(uint16_t _freq);
void PMSM_generateLookUpTable(void);
uint16_t getPhase(uint16_t sensorPos);
#endif
