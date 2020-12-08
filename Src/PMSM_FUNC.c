#include "PMSM_FUNC.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"


//defining variables
// BLDC motor steps tables
static const uint8_t PMSM_BRIDGE_STATE_FORWARD[8][6] =   // Motor steps
{
//	UH,UL		VH,VL	WH,WL
   { 0,0	,	0,0	,	0,0 },  // 0 //000
   { 0,1	,	0,0	,	1,0 },
   { 0,0	,	1,0	,	0,1 },
   { 0,1	,	1,0	,	0,0 },
   { 1,0	,	0,1	,	0,0 },
   { 0,0	,	0,1	,	1,0 },
   { 1,0	,	0,0	,	0,1 },
   { 0,0	,	0,0	,	0,0 },  // 0 //111
};

// BLDC motor backwared steps tables
static const uint8_t PMSM_BRIDGE_STATE_BACKWARD[8][6] =   // Motor steps
{
//	UH,UL		VH,VL	WH,WL
   { 0,0	,	0,0	,	0,0 },  // 0 //000
   { 1,0	,	0,0	,	0,1 },
   { 0,0	,	0,1	,	1,0 },
   { 1,0	,	0,1	,	0,0 },
   { 0,1	,	1,0	,	0,0 },
   { 0,0	,	1,0	,	0,1 },
   { 0,1	,	0,0	,	1,0 },
   { 0,0	,	0,0	,	0,0 },  // 0 //111
};

uint8_t PMSM_STATE[6] = {0,0,0,0,0,0};

volatile uint8_t	PMSM_Sensors = 0;
volatile uint8_t PMSM_MotorSpin = PMSM_CW;
volatile uint16_t PMSM_Speed = 0;
volatile uint16_t PMSM_PWM = 0;
volatile uint8_t PMSM_MotorRunFlag = 0;
volatile uint8_t PMSM_Mode = PMSM_MODE_DISABLED;
volatile uint16_t toUpdate=0,toUpdatePrev=0;//for BLDC start
char stringToUARTF[100] = "buffer here\r\n";//{'\0',};
extern uint32_t globalTime;

static uint16_t sinFreq;
volatile uint32_t phaseInc=0,phase=0;
double phaseIncMult=0.0;
static uint16_t lookUP[LOOKUP_ENTRIES];

//defining the callbacks here
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin) {
	
	PMSM_Sensors = PMSM_HallSensorsGetPosition();//get rotor postion
	BLDC_MotorCommutation(PMSM_Sensors);//commutate stator
	
	//calculate the current speed of rotor by getting the counter value of TIM14
	PMSM_Speed = TIM14->CNT;//get speed
	TIM14->CR1|=TIM_CR1_CEN;//enable
	TIM14->CNT = 0;//set
	
	PMSM_updatePhaseInc();
	phase=getPhase(PMSM_Sensors);
	
	PMSM_Mode=PMSM_MODE_ENABLED;
	//snprintf(stringToUARTF,100,"PhaseInc = %d PhaseIncMult=%lf\r\n",phaseInc>>23,phaseIncMult);
	//snprintf(stringToUARTF,100,"PMSM_Speed = %d RPM\r\n",(uint16_t)((uint32_t)225564/PMSM_Speed));
	//sendToUART(stringToUARTF);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
		//routine for TIM14 to calculate the speed//equivalent of TIM3
    if(htim->Instance == TIM14){
			// Overflow - the motor is stopped
			//take action here
			toUpdate = 0;
			toUpdatePrev=0;
			//PMSM_MotorRunFlag = 0;
		}
		//This routine should not be processed before HALL sensor routing.
		if(htim->Instance == TIM1 && PMSM_Mode == PMSM_MODE_ENABLED){//runs every 60us
			phase += phaseInc;
			//depending upon the the active phase update PWM width////////////////////////////start from here
			if(toUpdate == CH1) TIM1->CCR1=lookUP[phase%512];//(uint16_t)((uint32_t)lookUP[phase%512]*PMSM_PWM/PWM_PERIOD);
			else if(toUpdate == CH2) TIM1->CCR2=lookUP[phase%512];//(uint16_t)((uint32_t)lookUP[phase%512]*PMSM_PWM/PWM_PERIOD);
			else if(toUpdate == CH3) TIM1->CCR3=lookUP[phase%512];//(uint16_t)((uint32_t)lookUP[phase%512]*PMSM_PWM/PWM_PERIOD);
			toUpdatePrev=toUpdate;
		}
}

//defining functions

// Initialize of all needed peripheral
void PMSM_Init(void) {
	PMSM_MotorStop();
	PMSM_startPWMToYGB();
	PMSM_generateLookUpTable();
}

uint8_t PMSM_HallSensorsGetPosition(void) {
	uint8_t temp=(uint8_t)((GPIOB->IDR) & (GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7))>>5;
	return temp;
}

#ifdef ENABLE_UART_DEBUG
void sendToUART(char *st){
	HAL_UART_Transmit(&huart1,(uint8_t *)st,strlen(st),HAL_MAX_DELAY);
}
#endif

void PMSM_setPWMFreq(uint16_t sfreq){
	__HAL_TIM_SetCounter(&htim1,sfreq);
}

uint16_t PMSM_getPWMFreq(uint16_t gfreq){
	//in futur, frequency calculation can be performed here 
	return gfreq;
}

void PMSM_startPWMToYGB(void){
	//setting frequency of PWM signal to the Motor
	PMSM_setPWMFreq(PMSM_getPWMFreq(PWM_PERIOD));
	
	//putting all the phases in starting condition.
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	
	//starting the PWM channels
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
}

// Transform ADC value to value for writing to the timer register
uint16_t PMSM_ADCToPWM(uint16_t ADC_VALUE) {
	uint32_t tmp;

	if (ADC_VALUE < PMSM_ADC_STOP) {
		return 0;
	} else {
		if (ADC_VALUE > PMSM_ADC_MAX) {
			return PWM_PERIOD+1;
		}
		else {
			tmp = (uint32_t)(ADC_VALUE-PMSM_ADC_STOP) * (uint32_t)PWM_PERIOD / (uint32_t)(PMSM_ADC_MAX - PMSM_ADC_START);
			return (uint16_t) tmp;
		}
	}
}

// Get index in sine table based on the sensor data, the timing and the direction of rotor rotation
uint8_t	PMSM_GetState(uint8_t index) {
	return 0;
}

void PMSM_MotorSetSpin(uint8_t spin) {
	PMSM_MotorSpin = spin;
}

uint8_t PMSM_MotorSpeedIsOK(void) {
	return 0;
}

uint8_t PMSM_MotorIsRun(void) {
	return PMSM_MotorRunFlag;
}

uint16_t PMSM_GetSpeed(void) {
	return PMSM_Speed;
}

void PMSM_MotorSetRun(void) {
	PMSM_MotorRunFlag = 1;
}

// Stop a motor
void PMSM_MotorStop(void){
	//PMSM_SetPWMWidthToYGB(0);
	//upper switches
	//turn them off
	
	//lower swithes
	//lower switches are already off right after the power in ON
	
	//stopping the timers
	__HAL_TIM_DISABLE(&htim14);

	PMSM_Speed = 0;
	PMSM_MotorRunFlag = 0;
}

void BLDC_MotorCommutation(uint16_t hallpos){
	
	if (PMSM_MotorSpin == PMSM_CW) {
		memcpy(PMSM_STATE, PMSM_BRIDGE_STATE_FORWARD[hallpos], sizeof(PMSM_STATE));
	}
	else if(PMSM_MotorSpin == PMSM_CCW){
		memcpy(PMSM_STATE, PMSM_BRIDGE_STATE_BACKWARD[hallpos], sizeof(PMSM_STATE));
	}

	// Disable if need
	if (!PMSM_STATE[UH]) TIM1->CCR3=0;
	if (!PMSM_STATE[UL]) HAL_GPIO_WritePin(punchYL_GPIO_Port, punchYL_Pin, GPIO_PIN_SET);
	if (!PMSM_STATE[VH]) TIM1->CCR2=0;
	if (!PMSM_STATE[VL]) HAL_GPIO_WritePin(punchGL_GPIO_Port, punchGL_Pin, GPIO_PIN_SET);
	if (!PMSM_STATE[WH]) TIM1->CCR1=0;
	if (!PMSM_STATE[WL]) HAL_GPIO_WritePin(punchBL_GPIO_Port, punchBL_Pin, GPIO_PIN_SET);

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (PMSM_STATE[UH] & !PMSM_STATE[UL]) { toUpdate = CH3; }
	if (PMSM_STATE[UL] & !PMSM_STATE[UH]) { HAL_GPIO_WritePin(punchYL_GPIO_Port, punchYL_Pin, GPIO_PIN_RESET); }
	if (PMSM_STATE[VH] & !PMSM_STATE[VL]) {	toUpdate = CH2; }
	if (PMSM_STATE[VL] & !PMSM_STATE[VH]) { HAL_GPIO_WritePin(punchGL_GPIO_Port, punchGL_Pin, GPIO_PIN_RESET); }
	if (PMSM_STATE[WH] & !PMSM_STATE[WL]) {	toUpdate = CH1; }
	if (PMSM_STATE[WL] & !PMSM_STATE[WH]) {	HAL_GPIO_WritePin(punchBL_GPIO_Port, punchBL_Pin, GPIO_PIN_RESET); }
}

uint16_t PMSM_setFreq(uint16_t _freq){
	if(_freq > 1000){
		return 0;
	}else{
		sinFreq=_freq;
		phaseInc = (uint32_t) phaseIncMult*_freq;
		return 1;
	}
}
void PMSM_generateLookUpTable(void){
	double temp;
	for(uint16_t i=0;i<LOOKUP_ENTRIES;i++){
		temp = sin((i*M_PI)/LOOKUP_ENTRIES)*PWM_PERIOD*SPEEDING_FACTOR;
		lookUP[i] = (uint16_t)(temp+0.5);
		snprintf(stringToUARTF,100,"lookUp[%d] = %d\r\n",i,lookUP[i]);
		sendToUART(stringToUARTF);
	}
}

void PMSM_updatePhaseInc(void){
	phaseIncMult = (double)PMSM_Speed/60;
	phaseInc = (uint32_t)(((double)LOOKUP_ENTRIES/phaseIncMult))/2;
}

uint16_t getPhase(uint16_t sensorPos){
	if(toUpdatePrev == 0) return 0;
	if(toUpdatePrev == toUpdate) return (LOOKUP_ENTRIES/2);
	if(toUpdatePrev != toUpdate) return 0;
}

// input values ??are: input value, a minimum value input, the maximum input, output minimum, maximum output
float map(float val, float I_Min, float I_Max, float O_Min, float O_Max){
		return(((val-I_Min)*((O_Max-O_Min)/(I_Max-I_Min)))+O_Min);
}
  
void PMSM_SetPWMWidthToYGB(uint8_t val){
	if(PMSM_Mode == PMSM_MODE_DISABLED){
		if(toUpdate == CH1) TIM1->CCR1=val;
		else if(toUpdate == CH2) TIM1->CCR2=val;
		else if(toUpdate == CH3) TIM1->CCR3=val;
	}
}

void PMSM_updatePMSMPWMVariable(uint16_t PWM){
	PMSM_PWM=PWM;
}

