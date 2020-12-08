#include "serial.h"
#include "stm32l4xx_hal.h"

extern UART_HandleTypeDef huart;

// overriding _write of 'syscalls.c' so that printf
// will print to the serial port (UART)
int _write(int file, char *ptr, int len)
{
	HAL_StatusTypeDef status = HAL_UART_Transmit(
			&huart, (uint8_t *) ptr, len, HAL_MAX_DELAY);
	return (status == HAL_OK ? len : 0);
}
