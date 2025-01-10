#ifndef _PMSM_FUNC_LIB_H_
#define _PMSM_FUNC_LIB_H_

#include "string.h"
#include "stdint.h"
#include "stdio.h"
#include "gpio.h"
#include "stdbool.h"




//macros
// Use const for mode values
enum {
	PMSM_CW=0,
	PMSM_CCW
};
// Use enum for related constants
enum {
    UH = 0,
    UL,
    VH,
    VL,
    WH,
    WL
};

enum {
    CH1 = 1,
    CH2,
    CH3
};

//******************************************************************throttle related
typedef struct{
	uint16_t reading;
	uint8_t pin;
}Throttle;
uint16_t getThrottleStartValue();
uint16_t getThrottleReading(Throttle *throttle);
Throttle *getThrottleHandle(void);

//*****************************************************************Hall sensor related
typedef struct{
	uint8_t sector;
	uint8_t pins;
}HallSensor;
uint8_t getRotorSector(void);
HallSensor *getHallSensorHandle(void);

typedef struct{
	uint8_t ch;
}Comm;
Comm *getCommHandle();

//*****************************************************************Motor related
typedef struct{
	uint8_t type;
	uint16_t speed;
	uint8_t spin_direction;
	bool running_state;
}Motor;
void setMotorSpinDirection(uint8_t spin_direction);
uint8_t getMotorSpinDirection();
void setMotorRunningState(bool running_state);
bool isMotorRunning(void);
Motor *getMotorHandle(void);
//******************************************************************Drive Timer related
typedef struct{

}DriveTimer;

//******************************************************************Controller related
typedef struct{
	Throttle throttle_t;
	HallSensor hall_sensor_t;
	Comm uart_t;
	Motor motor_t;
	DriveTimer drive_timer_t;
}Controller;
void initController();
Controller *getControllerHandle(void);

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

void PMSM_MotorStop(void);
void PMSM_MotorSetRun(void);
uint16_t PMSM_GetSpeed(void);
uint8_t PMSM_MotorIsRun(void);
void PMSM_Init(void);
void BLDC_MotorCommutation(uint16_t hallpos);

uint16_t PMSM_setFreq(uint16_t _freq);
void PMSM_generateLookUpTable(void);
uint16_t getPhase(uint16_t sensorPos);


bool isReverseButtonPressed(void);
#endif
