#include "serial.h"
#include "stm32l4xx_hal_uart.h"

static UART_HandleTypeDef *huart;

void Serial_Init(UART_HandleTypeDef *h)
{
	huart = h;
}

bool Serial_Write(uint8_t *data, uint16_t size)
{
	return HAL_UART_Transmit(huart, data, size, HAL_MAX_DELAY) == HAL_OK;
}

bool Serial_Read(uint8_t *data, uint16_t size)
{
	return HAL_UART_Receive(huart, data, size, HAL_MAX_DELAY) == HAL_OK;
}


/*
 * Reads a '\r' terminated string from UART.
 */
bool Serial_ReadStringCR(char *data, uint16_t size)
{
	uint16_t c;
	size = size - sizeof('\0');

	for (c = 0; c < size; c++) {
		if (HAL_UART_Receive(huart, (uint8_t *) &data[c], 1, HAL_MAX_DELAY) != HAL_OK)
			return false;
		if (data[c] == '\r') {
			data[c] = '\0';
			break;
		}
	}

	return c < size;
}

/*
 * Overriding _putchar of so that printf (of custom printf.h)
 * will print to the serial port (UART)
 */
void _putchar(char character)
{
	HAL_UART_Transmit(huart, (uint8_t *) &character, 1, HAL_MAX_DELAY);
}

// TODO: for ReadAsync
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	println("HAL_UART_RxCpltCallback");
//}

