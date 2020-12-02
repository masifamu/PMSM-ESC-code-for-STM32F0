#include "PMSM_FUNC.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


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

// Sin table
#define PMSM_SINTABLESIZE	192
static const uint8_t PMSM_SINTABLE [PMSM_SINTABLESIZE][3] =
{
		{0,       0,      221},//0 index, HS=3 and VL can be switched ON here.
		{8,       0,      225},
		{17,      0,      229},
		{25,      0,      232},
		{33,      0,      236},
		{42,      0,      239},
		{50,      0,      241},
		{58,      0,      244},
		{66,      0,      246},
		{74,      0,      248},
		{82,      0,      250},
		{90,      0,      252},
		{98,      0,      253},
		{105,     0,      254},
		{113,     0,      254},
		{120,     0,      255},
		{128,     0,      255},
		{135,     0,      255},
		{142,     0,      254},
		{149,     0,      254},
		{155,     0,      253},
		{162,     0,      252},
		{168,     0,      250},
		{174,     0,      248},
		{180,     0,      246},
		{186,     0,      244},
		{192,     0,      241},
		{197,     0,      239},
		{202,     0,      236},
		{207,     0,      232},
		{212,     0,      229},
		{217,     0,      225},
		{221,     0,      221},//32 index, here HS=2 and VLower can be switched ON
		{225,     0,      217},
		{229,     0,      212},
		{232,     0,      207},
		{236,     0,      202},
		{239,     0,      197},
		{241,     0,      192},
		{244,     0,      186},
		{246,     0,      180},
		{248,     0,      174},
		{250,     0,      168},
		{252,     0,      162},
		{253,     0,      155},
		{254,     0,      149},
		{254,     0,      142},
		{255,     0,      135},
		{255,     0,      127},
		{255,     0,      120},
		{254,     0,      113},
		{254,     0,      105},
		{253,     0,      98},
		{252,     0,      90},
		{250,     0,      82},
		{248,     0,      74},
		{246,     0,      66},
		{244,     0,      58},
		{241,     0,      50},
		{239,     0,      42},
		{236,     0,      33},
		{232,     0,      25},
		{229,     0,      17},
		{225,     0,      8},
		{221,     0,      0},//64 index, HS=6 and WL can be switched ON
		{225,     8,      0},
		{229,     17,     0},
		{232,     25,     0},
		{236,     33,     0},
		{239,     42,     0},
		{241,     50,     0},
		{244,     58,     0},
		{246,     66,     0},
		{248,     74,     0},
		{250,     82,     0},
		{252,     90,     0},
		{253,     98,     0},
		{254,     105,    0},
		{254,     113,    0},
		{255,     120,    0},
		{255,     127,    0},
		{255,     135,    0},
		{254,     142,    0},
		{254,     149,    0},
		{253,     155,    0},
		{252,     162,    0},
		{250,     168,    0},
		{248,     174,    0},
		{246,     180,    0},
		{244,     186,    0},
		{241,     192,    0},
		{239,     197,    0},
		{236,     202,    0},
		{232,     207,    0},
		{229,     212,    0},
		{225,     217,    0},
		{221,     221,    0},//96 index,HS=4 and WL can be switched ON
		{217,     225,    0},
		{212,     229,    0},
		{207,     232,    0},
		{202,     236,    0},
		{197,     239,    0},
		{192,     241,    0},
		{186,     244,    0},
		{180,     246,    0},
		{174,     248,    0},
		{168,     250,    0},
		{162,     252,    0},
		{155,     253,    0},
		{149,     254,    0},
		{142,     254,    0},
		{135,     255,    0},
		{128,     255,    0},
		{120,     255,    0},
		{113,     254,    0},
		{105,     254,    0},
		{98,      253,    0},
		{90,      252,    0},
		{82,      250,    0},
		{74,      248,    0},
		{66,      246,    0},
		{58,      244,    0},
		{50,      241,    0},
		{42,      239,    0},
		{33,      236,    0},
		{25,      232,    0},
		{17,      229,    0},
		{8,       225,    0},
		{0,       221,    0},//128 index, HS=5 and UL can be switched ON
		{0,       225,    8},
		{0,       229,    17},
		{0,       232,    25},
		{0,       236,    33},
		{0,       239,    42},
		{0,       241,    50},
		{0,       244,    58},
		{0,       246,    66},
		{0,       248,    74},
		{0,       250,    82},
		{0,       252,    90},
		{0,       253,    98},
		{0,       254,    105},
		{0,       254,    113},
		{0,       255,    120},
		{0,       255,    128},
		{0,       255,    135},
		{0,       254,    142},
		{0,       254,    149},
		{0,       253,    155},
		{0,       252,    162},
		{0,       250,    168},
		{0,       248,    174},
		{0,       246,    180},
		{0,       244,    186},
		{0,       241,    192},
		{0,       239,    197},
		{0,       236,    202},
		{0,       232,    207},
		{0,       229,    212},
		{0,       225,    217},
		{0,       221,    221},//160 index, HS=1 and UL can be switched ON
		{0,       217,    225},
		{0,       212,    229},
		{0,       207,    232},
		{0,       202,    236},
		{0,       197,    239},
		{0,       192,    241},
		{0,       186,    244},
		{0,       180,    246},
		{0,       174,    248},
		{0,       168,    250},
		{0,       162,    252},
		{0,       155,    253},
		{0,       149,    254},
		{0,       142,    254},
		{0,       135,    255},
		{0,       128,    255},
		{0,       120,    255},
		{0,       113,    254},
		{0,       105,    254},
		{0,       98,     253},
		{0,       90,     252},
		{0,       82,     250},
		{0,       74,     248},
		{0,       66,     246},
		{0,       58,     244},
		{0,       50,     241},
		{0,       42,     239},
		{0,       33,     236},
		{0,       25,     232},
		{0,       17,     229},
		{0,       8,      225}
};

// Phase correction table(check values in these tables
static const uint8_t PMSM_STATE_TABLE_INDEX_FORWARD[8] = {0, 160, 32, 0, 96, 128, 64, 0};
static const uint8_t PMSM_STATE_TABLE_INDEX_BACKWARD[8] = {0, 32, 160, 0, 96, 64, 128, 0};

volatile uint8_t PMSM_SinTableIndex = 0;
volatile uint8_t	PMSM_Sensors = 0;
volatile uint8_t PMSM_MotorSpin = PMSM_CW;
volatile uint16_t PMSM_Speed = 0;
volatile uint16_t PMSM_Speed_prev = 0;
volatile uint16_t PMSM_PWM = 0;
volatile uint8_t PMSM_ModeEnabled = 0;
volatile uint8_t PMSM_MotorRunFlag = 0;
volatile uint16_t toUpdate=0;//for BLDC start
// Timing (points in sine table)
// sine table contains 192 items; 360/192 = 1.875 degrees per item
volatile static int8_t PMSM_Timing = 10; // 15 * 1.875 = 28.125 degrees


//defining the callbacks here
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin) {
  //calculate the hall sensor position
	PMSM_Sensors = PMSM_HallSensorsGetPosition();
	PMSM_MotorManageLowerSwitches(PMSM_Sensors);
	//calculate the current speed of rotor by getting the counter value of TIM3/TIM14
	PMSM_Speed_prev = PMSM_Speed;
  PMSM_Speed =__HAL_TIM_GetCounter(&htim14);
  
	__HAL_TIM_ENABLE(&htim14);
	__HAL_TIM_ENABLE_IT(&htim14,TIM_IT_UPDATE);
	HAL_TIM_Base_Start(&htim14);
	
	__HAL_TIM_SetCounter(&htim14,0);
	
	//setting the TIM4/TIM16 counter value calculated from TIM3/TIM14
	if (PMSM_MotorSpeedIsOK()) {
		// Enable timer TIM4 to generate sine
		__HAL_TIM_SetCounter(&htim16,0);
		// Set timer period
		//TIM4->ARR = PMSM_Speed / 32; //32 - number of items in the sine table between commutations (192/6 = 32)
		__HAL_TIM_SetAutoreload(&htim16,PMSM_Speed/32);
		
		__HAL_TIM_ENABLE(&htim16);
		__HAL_TIM_ENABLE_IT(&htim16,TIM_IT_UPDATE);
		HAL_TIM_Base_Start(&htim16);
	}

	// If Hall sensors value is valid(check here?)
  if ((PMSM_Sensors > 0 ) & (PMSM_Sensors < 7)) {
  // Do a phase correction
  	PMSM_SinTableIndex = PMSM_GetState(PMSM_Sensors);
	}
			
	//blinking the yellow LED
	HAL_GPIO_TogglePin(ledY_GPIO_Port,ledY_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		//routine for TIM14 to calculate the speed//equivalent of TIM3
    if(htim->Instance == TIM14){
			// Overflow - the motor is stopped
			if (PMSM_MotorSpeedIsOK()) {
				PMSM_MotorStop();
			}
		}
		
		//routine for TIM16//equivalent of TIM4
		if(htim->Instance == TIM16){
			HAL_GPIO_TogglePin(ledB_GPIO_Port,ledB_Pin);
			//enable PMSM mode here
			// If time to enable PMSM mode
			if (PMSM_ModeEnabled == 0) {
				// Turn PWM outputs for working with sine wave
				PMSM_startPWMToYGB();
				PMSM_ModeEnabled = 1;
			}
			//calculate the pwm widths for three phases.
			uint16_t pwmY,pwmG,pwmB;
			pwmY = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][0]/255);
			pwmG = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][1]/255);
			pwmB = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][2]/255);
			
			//depending upon the spin set PWM width to three phases
			if (PMSM_MotorSpin == PMSM_CW) {
				// Forward rotation
				PMSM_SetPWMWidthToYGB(pwmY,pwmG,pwmB);
			}
			else {
				// Backward rotation
				PMSM_SetPWMWidthToYGB(pwmY,pwmB,pwmG);
			}
			
			//managing sineTable index
			// Increment position in sine table
			PMSM_SinTableIndex++;
			if (PMSM_SinTableIndex > PMSM_SINTABLESIZE-1) {
				PMSM_SinTableIndex = 0;
			}
		}
}

//defining functions

// Initialize of all needed peripheral
void PMSM_Init(void) {
	PMSM_MotorStop();
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

void PMSM_SetPWMWidthToYGB(uint16_t wY,uint16_t wG,uint16_t wB){
	//set pwm width for the three phases.
	if(PMSM_ModeEnabled == 1){
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,wY);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,wG);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,wB);
	}
}
// Set PWM (same for all phases)
void PMSM_updatePMSMPWMVariable(uint16_t PWM){
	if (PMSM_ModeEnabled == 0) {
		if(toUpdate == CH1){
			TIM1->CCR1=PWM;
		}else if(toUpdate == CH2){
			TIM1->CCR2=PWM;
		}else if(toUpdate == CH3){
			TIM1->CCR3=PWM;
		}
	}else {
		PMSM_PWM = PWM;
	}
}

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
	//settting the PWM width of each channel
	//PMSM_SetPWMWidthToYGB(0,0,0);
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
uint8_t	PMSM_GetState(uint8_t SensorsPosition) {
	int16_t index;

	if (PMSM_MotorSpin == PMSM_CW) {
		index = PMSM_STATE_TABLE_INDEX_FORWARD[SensorsPosition];
	}else {
		index = PMSM_STATE_TABLE_INDEX_BACKWARD[SensorsPosition];
	}

	index = index + (int16_t)PMSM_Timing;
	if (index > PMSM_SINTABLESIZE-1) {
		index = index - PMSM_SINTABLESIZE;
	}else {
		if (index < 0) {
			index = PMSM_SINTABLESIZE + index;
		}
	}

	return index;
}

void PMSM_MotorSetSpin(uint8_t spin) {
	PMSM_MotorSpin = spin;
}

uint8_t PMSM_MotorSpeedIsOK(void) {
	return ((PMSM_Speed_prev > 0) & (PMSM_Speed > 0));
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
	PMSM_SetPWMWidthToYGB(0,0,0);
	//upper switches
	//turn them off
	
	//lower swithes
	//turn them off
	HAL_GPIO_WritePin(YL_GPIO_Port, YL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GL_GPIO_Port, GL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_SET);
	
	//stopping the timers
	__HAL_TIM_DISABLE(&htim14);
	__HAL_TIM_DISABLE(&htim16);

	PMSM_Speed = 0;
	PMSM_Speed_prev = 0;
	PMSM_MotorRunFlag = 0;
	PMSM_ModeEnabled = 0;
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
	if (!PMSM_STATE[UL]) HAL_GPIO_WritePin(YL_GPIO_Port, YL_Pin, GPIO_PIN_SET);
	if (!PMSM_STATE[VH]) TIM1->CCR2=0;
	if (!PMSM_STATE[VL]) HAL_GPIO_WritePin(GL_GPIO_Port, GL_Pin, GPIO_PIN_SET);
	if (!PMSM_STATE[WH]) TIM1->CCR1=0;
	if (!PMSM_STATE[WL]) HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_SET);

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (PMSM_STATE[UH] & !PMSM_STATE[UL]) { toUpdate = CH3; }
	if (PMSM_STATE[UL] & !PMSM_STATE[UH]) { HAL_GPIO_WritePin(YL_GPIO_Port, YL_Pin, GPIO_PIN_RESET); }
	if (PMSM_STATE[VH] & !PMSM_STATE[VL]) {	toUpdate = CH2; }
	if (PMSM_STATE[VL] & !PMSM_STATE[VH]) { HAL_GPIO_WritePin(GL_GPIO_Port, GL_Pin, GPIO_PIN_RESET); }
	if (PMSM_STATE[WH] & !PMSM_STATE[WL]) {	toUpdate = CH1; }
	if (PMSM_STATE[WL] & !PMSM_STATE[WH]) {	HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_RESET); }
}

void PMSM_MotorManageLowerSwitches(uint16_t hallpos){
	
	if (PMSM_MotorSpin == PMSM_CW) {
		memcpy(PMSM_STATE, PMSM_BRIDGE_STATE_FORWARD[hallpos], sizeof(PMSM_STATE));
	}
	else if(PMSM_MotorSpin == PMSM_CCW){
		memcpy(PMSM_STATE, PMSM_BRIDGE_STATE_BACKWARD[hallpos], sizeof(PMSM_STATE));
	}

	// Disable if need
	if (!PMSM_STATE[UL]) HAL_GPIO_WritePin(YL_GPIO_Port, YL_Pin, GPIO_PIN_SET);
	if (!PMSM_STATE[VL]) HAL_GPIO_WritePin(GL_GPIO_Port, GL_Pin, GPIO_PIN_SET);
	if (!PMSM_STATE[WL]) HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_SET);

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if ((PMSM_STATE[UL] & !PMSM_STATE[UH]) && __HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_3) == 0) { HAL_GPIO_WritePin(YL_GPIO_Port, YL_Pin, GPIO_PIN_RESET); }
	if ((PMSM_STATE[VL] & !PMSM_STATE[VH]) && __HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_2) == 0) { HAL_GPIO_WritePin(GL_GPIO_Port, GL_Pin, GPIO_PIN_RESET); }
	if ((PMSM_STATE[WL] & !PMSM_STATE[WH]) && __HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_3) == 0) {	HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_RESET); }
}


