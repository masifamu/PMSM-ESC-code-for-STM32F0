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
volatile uint16_t toUpdate=0,toUpdatePrev=0;//for BLDC start
char stringToUARTF[100] = "buffer here\r\n";//{'\0',};
extern uint32_t globalTime;

volatile uint32_t phaseInc=1U,phase=0;
static uint16_t lookUP[LOOKUP_ENTRIES];
volatile uint16_t throttledPWMWidth=0;

//defining the callbacks here
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin) {
	
	PMSM_Sensors = (uint8_t)((GPIOB->IDR) & (HS_PINS))>>5;//get rotor postion
	
	BLDC_MotorCommutation(PMSM_Sensors);//commutate stator
	
	//calculate the current speed of rotor by getting the counter value of TIM14
	PMSM_Speed = TIM14->CNT;//get speed
	TIM14->CR1|=TIM_CR1_CEN;//enable
	TIM14->CNT = 0;//set
	
	if((PMSM_Speed > 100) & (PMSM_Speed < 15360U)){//check here
		phaseInc = (uint32_t)LOOKUP_ENTRIES*30/PMSM_Speed;
	}
	
	if((PMSM_Sensors > 0) & (PMSM_Sensors < 7)){
		phase=getPhase(PMSM_Sensors);
	}
	//(GPIOA->ODR & 0x00001000U) ? (GPIOA->BRR = 0x00001000U):(GPIOA->BSRR = 0x00001000U);//don't use here
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	
		//at every TIM14 overflow this routine executes.
    if(htim->Instance == TIM14){
			phase=0;
			toUpdate = 0;
			toUpdatePrev=0;
			PMSM_MotorRunFlag = 0;
		}
		
		if(htim->Instance == TIM1){//runs every 60us
			phase += phaseInc;
			throttledPWMWidth=(uint16_t)((uint32_t)lookUP[(phase & 0x000001FF)]*PMSM_PWM/PWM_PERIOD);
			//depending upon the the active phase update PWM width
			if(toUpdate == CH1) TIM1->CCR1=throttledPWMWidth;
			else if(toUpdate == CH2) TIM1->CCR2=throttledPWMWidth;
			else if(toUpdate == CH3) TIM1->CCR3=throttledPWMWidth;
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
	return (uint8_t)((GPIOB->IDR) & (HS_PINS))>>5;
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

void PMSM_MotorSetSpin(uint8_t spin) {
	PMSM_MotorSpin = spin;
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

	//upper switches
	//turn them off
	
	//lower swithes
	//lower switches are already off right after the power in ON
	
	//stopping the timer
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
	if (!PMSM_STATE[UL]) GPIOB->BSRR = 0x0002;//Y
	if (!PMSM_STATE[VH]) TIM1->CCR2=0;
	if (!PMSM_STATE[VL]) GPIOB->BSRR = 0x0001;//G
	if (!PMSM_STATE[WH]) TIM1->CCR1=0;
	if (!PMSM_STATE[WL]) GPIOA->BSRR = 0x0080;//B

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (PMSM_STATE[UH] & !PMSM_STATE[UL]) { toUpdate = CH3; }
	if (PMSM_STATE[UL] & !PMSM_STATE[UH]) GPIOB->BRR = 0x0002;//Y
	if (PMSM_STATE[VH] & !PMSM_STATE[VL]) {	toUpdate = CH2; }
	if (PMSM_STATE[VL] & !PMSM_STATE[VH]) GPIOB->BRR = 0x0001;//G
	if (PMSM_STATE[WH] & !PMSM_STATE[WL]) {	toUpdate = CH1; }
	if (PMSM_STATE[WL] & !PMSM_STATE[WH]) GPIOA->BRR = 0x0080;//B
}

void PMSM_generateLookUpTable(void){
	double temp;
	for(uint16_t i=0;i<LOOKUP_ENTRIES;i++){
		temp = sin((i*M_PI)/LOOKUP_ENTRIES)*PWM_PERIOD*SPEEDING_FACTOR;
		lookUP[i] = (uint16_t)(temp+0.5);
		//snprintf(stringToUARTF,100,"lookUp[%d] = %d\r\n",i,lookUP[i]);
		//sendToUART(stringToUARTF);
	}
}

//void PMSM_updatePhaseInc(void){
//	//phaseIncMult = (double)PMSM_Speed/60;
//	//phaseInc = (uint32_t)(((double)LOOKUP_ENTRIES/phaseIncMult))/2;//2 is used here, because we are using only half sinewave
//	//phaseInc = LOOKUP_ENTRIES*30/PMSM_Speed;
//}

uint16_t getPhase(uint16_t sensorPos){
	if(toUpdatePrev == 0) return 0;
	if(toUpdatePrev == toUpdate) return (LOOKUP_ENTRIES/2);
	return 0;
}
  
void PMSM_SetPWMWidthToYGB(uint8_t val){
		if(toUpdate == CH1) TIM1->CCR1=val;
		else if(toUpdate == CH2) TIM1->CCR2=val;
		else if(toUpdate == CH3) TIM1->CCR3=val;
}

void PMSM_updatePMSMPWMVariable(uint16_t PWM){
	PMSM_PWM=PWM;
}


