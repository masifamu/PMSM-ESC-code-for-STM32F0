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

#define UH	0
#define UL	1
#define VH	2
#define VL	3
#define WH	4
#define WL	5

#define TIM1CH1(x) TIM1->CCR1=x
#define TIM1CH2(x) TIM1->CCR2=x
#define TIM1CH3(x) TIM1->CCR3=x

#define CH1 1
#define CH2 2
#define CH3 3

#define PMSM_TIMER14_PRESCALER		8
#define PMSM_TIMER14_PERIOD			0xFFFF // 65535

//#ifdef ENABLE_UART_DEBUG
//uint8_t stringToUART[50] = {'\0',};
//#endif
void sendToUART(char *);
void PMSM_SetPWMWidthToYGB(uint16_t wY,uint16_t wG,uint16_t wB);
void PMSM_startPWMToYGB(void);
void PMSM_setPWMFreq(uint16_t sfreq);
uint16_t PMSM_getPWMFreq(uint16_t gfreq);
void PMSM_updatePMSMPWMVariable(uint16_t PWM);
uint16_t PMSM_ADCToPWM(uint16_t ADC_VALUE);
uint8_t PMSM_HallSensorsGetPosition(void);
uint8_t	PMSM_GetState(uint8_t SensorsPosition);
void PMSM_MotorSetSpin(uint8_t spin); 
uint8_t PMSM_MotorSpeedIsOK(void);
void PMSM_MotorStop(void);
void PMSM_MotorSetRun(void);
uint16_t PMSM_GetSpeed(void);
uint8_t PMSM_MotorIsRun(void);
void PMSM_Init(void);
void BLDC_MotorCommutation(uint16_t hallpos);
void BLDC_UpdatePWMWidth(uint8_t update);
void BLDC_SetPWM(uint16_t pwm);
#endif
