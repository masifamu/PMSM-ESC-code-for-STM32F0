#include "PMSM_FUNC.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


//defining variables
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

// Phase correction table(check values in these tables
static const uint8_t PMSM_STATE_TABLE_INDEX_FORWARD[8] = {0, 160, 32, 0, 96, 128, 64, 0};
static const uint8_t PMSM_STATE_TABLE_INDEX_BACKWARD[8] = {0, 32, 160, 0, 96, 64, 128, 0};

volatile uint8_t PMSM_SinTableIndex = 0;
volatile uint8_t	PMSM_Sensors = 0;
volatile uint8_t PMSM_MotorSpin = PMSM_CW;
// Timing (points in sine table)
// sine table contains 192 items; 360/192 = 1.875 degrees per item
volatile static int8_t PMSM_Timing = 10; // 15 * 1.875 = 28.125 degrees


//defining the callbacks here
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin) {
  //calculate the hall sensor position
	PMSM_Sensors = PMSM_HallSensorsGetPosition();
	
	//calculate the current speed of rotor by getting the counter value of TIM3
	//setting the TIM4 counter value calculated from TIM3
	
	// If Hall sensors value is valid(check here?)
  if ((PMSM_Sensors > 0 ) & (PMSM_Sensors < 7)) {
  // Do a phase correction
  	PMSM_SinTableIndex = PMSM_GetState(PMSM_Sensors);
	}
			
	//blinking the yellow LED
	HAL_GPIO_TogglePin(ledY_GPIO_Port,ledY_Pin);
}

//defining functions
uint8_t PMSM_HallSensorsGetPosition(void) {
	uint8_t temp=(uint8_t)((GPIOB->IDR) & (GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7))>>5;
	return temp;
}

#ifdef ENABLE_UART_DEBUG
void sendToUART(char *st){
	HAL_UART_Transmit(&huart1,(uint8_t *)st,strlen(st),HAL_MAX_DELAY);
}
#endif

void PMSM_SetPWMWidthToYGB(uint16_t mThrottle){
	uint16_t pwmY,pwmG,pwmB;
	//calculate the pwm widths for three phases.
	pwmY = (uint16_t)((uint32_t)mThrottle * PMSM_SINTABLE[PMSM_SinTableIndex][0]/255);
	pwmG = (uint16_t)((uint32_t)mThrottle * PMSM_SINTABLE[PMSM_SinTableIndex][1]/255);
	pwmB = (uint16_t)((uint32_t)mThrottle * PMSM_SINTABLE[PMSM_SinTableIndex][2]/255);
	
	//set pwm width for the three phases.
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,pwmY);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,pwmG);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,pwmB);
	
	//managing sineTable index
	// Increment position in sine table
	PMSM_SinTableIndex++;
	if (PMSM_SinTableIndex > PMSM_SINTABLESIZE-1) {
		PMSM_SinTableIndex = 0;
	}
}
void PMSM_setPWMFreq(uint16_t sfreq){
	__HAL_TIM_SetCounter(&htim1,sfreq);
}
uint16_t PMSM_getPWMFreq(uint16_t gfreq){
	//in futur, frequency calculation can be performed here 
	return gfreq;
}
void PMSM_startPWMToYGB(uint16_t mappedThrottle){
	//setting frequency of PWM signal to the Motor
	PMSM_setPWMFreq(PMSM_getPWMFreq(PWM_PERIOD));
	//settting the PWM width of each channel
	PMSM_SetPWMWidthToYGB(mappedThrottle);
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
