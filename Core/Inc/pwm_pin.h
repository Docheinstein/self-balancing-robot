#ifndef PWM_PIN_H
#define PWM_PIN_H

#include <stddef.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"


typedef enum {
	PWM_PIN_TYPE_16_BIT, // default
	PWM_PIN_TYPE_32_BIT
} PWM_PinType;

typedef struct PWM_Pin {
	TIM_HandleTypeDef *tim;
	uint32_t channel;
	PWM_PinType type;
} PWM_Pin;

void PWM_Pin_Start(PWM_Pin pin);
void PWM_Pin_Stop(PWM_Pin pin);

void PWM_Pin_Set(PWM_Pin, float frequency, float percentage);
void PWM_Pin_SetFrequency(PWM_Pin pin, float frequency);
void PWM_Pin_SetDutyCycle(PWM_Pin pin, float percentage);

void PWM_Pin_SetPSC(PWM_Pin pin, uint32_t psc);
void PWM_Pin_SetARR(PWM_Pin pin, uint32_t arr);
void PWM_Pin_SetCCR(PWM_Pin pin, uint32_t ccr);

uint32_t PWM_Pin_GetPSC(PWM_Pin pin);
uint32_t PWM_Pin_GetARR(PWM_Pin pin);
uint32_t PWM_Pin_GetCCR(PWM_Pin pin);

void PWM_Pin_ToString(PWM_Pin pin, char *buf, size_t buflen);

#endif /* PWM_PIN_H */
