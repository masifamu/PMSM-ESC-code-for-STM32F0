#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// Define your board configuration constants and macros here
// Macros to manipulate GPIO pins
#define GPIO_CLEAR_PIN(port, pin)     		((port)->BSRR = (pin << 16u))
#define GPIO_SET_PIN(port, pin)     		((port)->BSRR = (pin))
#define GPIO_READ_PIN(port, pin)     		((port)->IDR & (pin))
#define GPIO_TOGGLE_PIN(port, pin)     		((port)->ODR  ^= (pin))

#define BOARD_NAME 							"stm32f0_custom"
#define CPU_CLOCK_SPEED 					48000000 // 48 MHz
#define UART_BAUD_RATE 						115200
#define HALL_SENSOR_PINS 					(GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7)
#define GREEN_LED							GPIO_PIN_4
#define BLUE_LED							GPIO_PIN_12
#define YELLOW_LED							GPIO_PIN_3
// Add any other board-specific configuration here

#endif // BOARD_CONFIG_H
