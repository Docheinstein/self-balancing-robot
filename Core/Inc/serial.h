#ifndef SERIAL_H
#define SERIAL_H

#include "stm32l4xx_hal.h"
#include "printf.h"

#define SERIAL_ENDL "\r\n"
#define println(message, ...) printf(message SERIAL_ENDL, ##__VA_ARGS__)

void Serial_Init(UART_HandleTypeDef *huart);

#endif /* SERIAL_H */
