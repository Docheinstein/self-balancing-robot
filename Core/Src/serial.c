#include "serial.h"

static UART_HandleTypeDef *huart;

void Serial_Init(UART_HandleTypeDef *h) {
	huart = h;
}


// overriding _putchar of so that printf (of custom printf.h)
// will print to the serial port (UART)
void _putchar(char character)
{
	HAL_UART_Transmit(huart, (uint8_t *) &character, 1, HAL_MAX_DELAY);
}
