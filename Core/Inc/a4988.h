#ifndef A4988_H
#define A4988_H

#include "gpio_pin.h"
#include "pwm_pin.h"

typedef struct A4988_Config {
	PWM_Pin step;
	GPIO_Pin direction;
	GPIO_Pin sleep;
	bool inverted;
	uint16_t steps_per_revolution;
} A4988_Config;

typedef struct A4988 {
	A4988_Config config;
} A4988;


void A4988_Init(A4988 *a4988, A4988_Config config);
void A4988_Enable(A4988 *a4988);
void A4988_Disable(A4988 *a4988);
void A4988_Forward(A4988 *a4988, float rpm);
void A4988_Backward(A4988 *a4988, float rpm);


#endif /* A4988_H */
