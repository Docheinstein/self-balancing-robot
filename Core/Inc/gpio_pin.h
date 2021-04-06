#ifndef GPIO_PIN_H
#define GPIO_PIN_H

#include <stdbool.h>
#include <stddef.h>
#include "stm32l475xx.h"


typedef struct GPIO_Pin {
	GPIO_TypeDef *port;
	uint16_t pin;
} GPIO_Pin;

void GPIO_Pin_Toggle(GPIO_Pin pin);
void GPIO_Pin_High(GPIO_Pin pin);
void GPIO_Pin_Low(GPIO_Pin pin);
void GPIO_Pin_Set(GPIO_Pin pin, bool high);

void GPIO_Pin_ToString(GPIO_Pin pin, char *buf, size_t buflen);

#endif /* GPIO_PIN_H */
