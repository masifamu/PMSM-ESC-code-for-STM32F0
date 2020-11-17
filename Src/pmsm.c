#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

#include "pmsm.h"

#define TIM1CH1(x) TIM1->CCR1=x
#define TIM1CH2(x) TIM1->CCR2=x
#define TIM1CH3(x) TIM1->CCR3=x

#define CH1 1
#define CH2 2
#define CH3 3


// Variables
volatile uint8_t PMSM_MotorRunFlag = 0;
volatile uint8_t PMSM_MotorSpin = PMSM_CW;
uint8_t PMSM_State[6] = {0, 0, 0, 0, 0, 0};
uint8_t PMSM_State_Prev[6] = {0, 0, 0, 0, 0, 0};
volatile uint8_t	PMSM_Sensors = 0;
volatile uint8_t PMSM_SinTableIndex = 0;
volatile uint16_t PMSM_PWM = 0;
volatile uint16_t PMSM_Speed = 0;
volatile uint16_t PMSM_Speed_prev = 0;
volatile uint8_t PMSM_ModeEnabled = 0;

volatile uint8_t toUpdate=0;
volatile uint16_t PWMWIDTH=0;

#ifdef HALL_SEQUENCE_DEBUG
char printDataString1[50] = {'\0',};
#endif
#ifdef TIM3_DEBUG
char tim3debugstring[50] = {'\0',};
#endif
#ifdef TIM14_DEBUG
char tim14debugstring[50] = {'\0',};
#endif

// Timing (points in sine table)
// sine table contains 192 items; 360/192 = 1.875 degrees per item
volatile static int8_t PMSM_Timing = 10; // 15 * 1.875 = 28.125 degrees

// Forward Motor steps
static const uint8_t PMSM_BRIDGE_STATE_FORWARD[8][6] =
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

// Backward Motor steps
static const uint8_t PMSM_BRIDGE_STATE_BACKWARD[8][6] =
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

// Sin table
#define PMSM_SINTABLESIZE	192
static const uint8_t PMSM_SINTABLE [PMSM_SINTABLESIZE][3] =
{
		{0,       0,      221},
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
		{221,     0,      221},
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
		{221,     0,      0},
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
		{221,     221,    0},
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
		{0,       221,    0},
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
		{0,       221,    221},
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

// Phase correction table
static const uint8_t PMSM_STATE_TABLE_INDEX_FORWARD[8] = {0, 160, 32, 0, 96, 128, 64, 0};
static const uint8_t PMSM_STATE_TABLE_INDEX_BACKWARD[8] = {0, 32, 160, 0, 96, 64, 128, 0};

#define UH	0
#define UL	1
#define VH	2
#define VL	3
#define WH	4
#define WL	5

// Initialize of all needed peripheral

void PMSM_Init(void) {
	PMSM_MotorStart();
}


// Stop a motor
void PMSM_MotorStart(void)
{
	PMSM_SetPWM(0);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); 

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	
	TIM3->CR1 = 0;																						//disabling the timer3
	TIM14->CR1 = 0;																						//disabling the timer14
	
	PMSM_Speed = 0;
	PMSM_Speed_prev = 0;
	PMSM_MotorRunFlag = 0;
	PMSM_ModeEnabled = 0;
}

void PMSM_MotorStop(void){
	PMSM_SetPWM(1);
}
// Every time when hall sensors change state executed this IRQ handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);																//yellow LED
	// Get current rotor position
	PMSM_Sensors = PMSM_HallSensorsGetPosition();

	// Get rotation time (in inverse ratio speed) from timer TIM3
	PMSM_Speed_prev = PMSM_Speed;
	PMSM_Speed = __HAL_TIM_GET_COUNTER(&htim3);														//get counter CNT current value
#ifdef TIM3_DEBUG
	snprintf(tim3debugstring,50, "PMSM speed = %d\n\r", PMSM_Speed);
	HAL_UART_Transmit(&huart1, (uint8_t*)tim3debugstring, strlen(tim3debugstring), HAL_MAX_DELAY);
#endif
	TIM3->CR1 = 1;																												//1=enable the timer and 0 disables the timer
	__HAL_TIM_SET_COUNTER(&htim3,0);																			//initialize timer CNT

	// It requires at least two measurement to correct calculate the rotor speed
	if (PMSM_MotorSpeedIsOK()) {
		// Enable timer TIM4 to generate sine
#ifdef TIM14_DEBUG
		snprintf(tim14debugstring,50, "timer14counter = %d\n\r",PMSM_Speed/*__HAL_TIM_GET_COUNTER(&htim14)*/);
		HAL_UART_Transmit(&huart1, (uint8_t*)tim14debugstring, strlen(tim14debugstring), HAL_MAX_DELAY);
#endif
		__HAL_TIM_SET_COUNTER(&htim14,0);
		// Set timer period
		TIM14->ARR = PMSM_Speed / 32; //32 - number of items in the sine table between commutations (192/6 = 32)
		TIM14->CR1 = 1;
	}

	// If Hall sensors value is valid
	if ((PMSM_Sensors > 0 ) & (PMSM_Sensors < 7)) {
		// Do a phase correction
		PMSM_SinTableIndex = PMSM_GetState(PMSM_Sensors);
	}

	// If motor is started then used a block commutation
	if (PMSM_ModeEnabled == 0) {
		PMSM_MotorCommutation(PMSM_Sensors);
	}
#ifdef HALL_SEQUENCE_DEBUG
	snprintf(printDataString1,50, "PMSM_Sensors = %d\n\r", PMSM_Sensors);
	HAL_UART_Transmit(&huart1, (uint8_t*)printDataString1, strlen(printDataString1), HAL_MAX_DELAY);
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		// Overflow - the motor is stopped
		if (PMSM_MotorSpeedIsOK()){ PMSM_MotorStop(); }
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12);																	//blue LED
#ifdef TIM3_DEBUG
		HAL_UART_Transmit(&huart1, (uint8_t*)"Timer3 overflow\n\r", 17, HAL_MAX_DELAY);
#endif
	}
	if(htim->Instance == TIM14){
#ifdef TIM14_DEBUG
		HAL_UART_Transmit(&huart1, (uint8_t*)"Timer14 called\n\r", 16, HAL_MAX_DELAY);
#endif
		uint16_t PWM1, PWM2, PWM3;
		// If time to enable PMSM mode
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);																		//green LED	
 		if (PMSM_ModeEnabled == 0) {
			// Turn PWM outputs for working with sine wave
/*			TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
			TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
			TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

			TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
			TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
			TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

			TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
			TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
			TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
*/
			PMSM_ModeEnabled = 1;
		}

		// Calculate PWM for 3-phase
		PWM1 = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][0]/255);
		PWM2 = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][1]/255);
		PWM3 = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][2]/255);

		if (PMSM_MotorSpin == PMSM_CW) {
			// Forward rotation
			PMSM_SetPWM_UVW(PWM1, PWM2, PWM3);
		}	else {
			// Backward rotation
			PMSM_SetPWM_UVW(PWM1, PWM3, PWM2);
		}

		// Increment position in sine table
		PMSM_SinTableIndex++;
		if (PMSM_SinTableIndex > PMSM_SINTABLESIZE-1) {
			PMSM_SinTableIndex = 0;
		}
	}
}

// Get data from hall sensors
uint8_t PMSM_HallSensorsGetPosition(void) {
	uint8_t temp=(uint8_t)((GPIOB->IDR) & (GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7))>>5;
	return temp;
}


// This commutation is used just when motor start
void PMSM_MotorCommutation(uint16_t hallpos) {
	if (PMSM_MotorSpin == PMSM_CW) {
		memcpy(PMSM_State, PMSM_BRIDGE_STATE_FORWARD[hallpos], sizeof(PMSM_State));
	}
	else {
		memcpy(PMSM_State, PMSM_BRIDGE_STATE_BACKWARD[hallpos], sizeof(PMSM_State));
	}

// Disable if need
	if (!PMSM_State[UH]) TIM1CH3(0);
	if (!PMSM_State[UL]) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	if (!PMSM_State[VH]) TIM1CH2(0);
	if (!PMSM_State[VL]) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	if (!PMSM_State[WH]) TIM1CH1(0);
	if (!PMSM_State[WL]) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (PMSM_State[UH] & !PMSM_State[UL] & !PMSM_State_Prev[UH]) {
		TIM1CH3(PWMWIDTH); 
		toUpdate=CH3;
	}
	if (PMSM_State[UL] & !PMSM_State[UH] & !PMSM_State_Prev[UL]) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	if (PMSM_State[VH] & !PMSM_State[VL] & !PMSM_State_Prev[VH]) {
		TIM1CH2(PWMWIDTH); 
		toUpdate=CH2;
	}
	if (PMSM_State[VL] & !PMSM_State[VH] & !PMSM_State_Prev[VL]) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	}
	if (PMSM_State[WH] & !PMSM_State[WL] & !PMSM_State_Prev[WH]) {
		TIM1CH1(PWMWIDTH); 
		toUpdate=CH1;
	}
	if (PMSM_State[WL] & !PMSM_State[WH] & !PMSM_State_Prev[WL]) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	}
	memcpy(PMSM_State_Prev, PMSM_State, sizeof(PMSM_State));
}

// Transform ADC value to value for writing to the timer register
uint16_t PMSM_ADCToPWM(uint16_t ADC_VALUE) {
	uint32_t tmp;

	if (ADC_VALUE < PMSM_ADC_STOP) {
		return 0;
	} else {
		if (ADC_VALUE > PMSM_ADC_MAX) {
			return PMSM_CHOPPER_PERIOD+1;
		}
		else {
			tmp = (uint32_t)(ADC_VALUE-PMSM_ADC_STOP) * (uint32_t)PMSM_CHOPPER_PERIOD / (uint32_t)(PMSM_ADC_MAX - PMSM_ADC_START);
			return (uint16_t) tmp;
		}
	}
}

// Set PWM (same for all phases)
void PMSM_SetPWM(uint16_t PWM)
{
	if (PMSM_ModeEnabled == 0) {
		PWMWIDTH=PWM;
		if(toUpdate == CH1){
			TIM1CH1(PWMWIDTH);
		}else if(toUpdate == CH2){
			TIM1CH2(PWMWIDTH);
		}else if(toUpdate == CH3){
			TIM1CH3(PWMWIDTH);
		}
		//TIM1->CCR1 = PWM;
		//TIM1->CCR2 = PWM;
		//TIM1->CCR3 = PWM;
	}	else {
		PMSM_PWM = PWM;
	}
}

// Set PWM
void PMSM_SetPWM_UVW(uint16_t PWM1, uint16_t PWM2, uint16_t PWM3)
{
	if (PMSM_ModeEnabled == 1) {
		if(toUpdate==CH1){
			TIM1CH1(PWM1);
			PWMWIDTH=PWM1;
		}else if(toUpdate==CH2){
			TIM1CH2(PWM2);
			PWMWIDTH = PWM2;
		}else if(toUpdate == CH3){
			TIM1CH3(PWM3);
			PWMWIDTH = PWM3;
		}
		//TIM1->CCR1 = PWM1;
		//TIM1->CCR2 = PWM2;
		//TIM1->CCR3 = PWM3;
	}
}

// Get index in sine table based on the sensor data, the timing and the direction of rotor rotation
uint8_t	PMSM_GetState(uint8_t SensorsPosition) {
	int16_t index;

	if (PMSM_MotorSpin == PMSM_CW) {
		index = PMSM_STATE_TABLE_INDEX_FORWARD[SensorsPosition];
	} else {
		index = PMSM_STATE_TABLE_INDEX_BACKWARD[SensorsPosition];
	}

	index = index + (int16_t)PMSM_Timing;
	if (index > PMSM_SINTABLESIZE-1) {
		index = index - PMSM_SINTABLESIZE;
	} else {
		if (index < 0) {
			index = PMSM_SINTABLESIZE + index;
		}
	}

	return index;
}

uint8_t PMSM_MotorIsRun(void) {
	return PMSM_MotorRunFlag;
}

uint8_t PMSM_MotorSpeedIsOK(void) {
	return ((PMSM_Speed_prev > 0) & (PMSM_Speed > 0));
}

uint16_t PMSM_GetSpeed(void) {
	return PMSM_Speed;
}

void PMSM_MotorSetRun(void) {
	PMSM_MotorRunFlag = 1;
}

void PMSM_MotorSetSpin(uint8_t spin) {
	PMSM_MotorSpin = spin;
}
