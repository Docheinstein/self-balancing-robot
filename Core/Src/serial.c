#include "serial.h"
#include "stm32l4xx_hal.h"

extern UART_HandleTypeDef huart1;

// overriding _putchar of so that printf (of custom printf.h)
// will print to the serial port (UART)
void _putchar(char character)
{
	HAL_UART_Transmit(&huart1, (uint8_t *) &character, 1, HAL_MAX_DELAY);
}
