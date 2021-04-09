#ifndef PWM_PIN_H
#define PWM_PIN_H

#include <stddef.h>
#include "stm32l4xx_hal.h"


typedef struct PWM_Pin {
	TIM_HandleTypeDef *tim;
	uint32_t channel;
} PWM_Pin;

void PWM_Pin_Start(PWM_Pin pin);
void PWM_Pin_Stop(PWM_Pin pin);
void PWM_Pin_SetDutyCycle(PWM_Pin pin, float percentage);

// Not strictly necessary, can be configured in .ioc at compile time
void PWM_Pin_SetPSC(PWM_Pin pin, uint16_t psc);
void PWM_Pin_SetARR(PWM_Pin pin, uint16_t arr);

void PWM_Pin_ToString(PWM_Pin pin, char *buf, size_t buflen);

#endif /* PWM_PIN_H */
