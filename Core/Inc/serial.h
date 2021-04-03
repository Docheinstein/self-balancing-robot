#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "printf.h"

#define SERIAL_ENDL "\r\n"

#define println(message, ...) printf(message SERIAL_ENDL, ##__VA_ARGS__)

void Serial_Init(UART_HandleTypeDef *huart);
bool Serial_Write(uint8_t *data, uint16_t size);
bool Serial_Read(uint8_t *data, uint16_t size);
bool Serial_ReadStringCR(char *data, uint16_t size);

#endif /* SERIAL_H */
