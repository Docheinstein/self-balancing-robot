#ifndef A4988_H
#define A4988_H

#include "gpio_pin.h"

/*
 * A4988 requires a minimum of us to wait before submit a
 * falling edge to the step pin; actually it should be about 2us,
 * but set this a little bit higher to be sure the step is sent correctly.
 */
#define A4988_MINIMUM_STEP_PULSE_MICROSECONDS 10

/*
 * A4988 requires a minimum of milliseconds to wait
 * after a wake up before use the driver.
 */
#define A4988_MINIMUM_STEP_AFTER_WAKE_UP_MILLISECONDS 2

typedef enum {
	A4988_STEP_DIRECTION_FORWARD,
	A4988_STEP_DIRECTION_BACKWARD
} A4988_StepDirection;

typedef struct A4988_Config {
	GPIO_Pin step;
	GPIO_Pin direction;
	GPIO_Pin sleep;
	bool inverted;
} A4988_Config;

typedef struct A4988 {
	A4988_Config config;
	bool enabled;
} A4988;


void A4988_Init(A4988 *a4988, A4988_Config config);

void A4988_Enable(A4988 *a4988);
void A4988_Disable(A4988 *a4988);

void A4988_StepForward(A4988 *a4988, bool wait_falling_edge);
void A4988_StepBackward(A4988 *a4988, bool wait_falling_edge);
void A4988_StepFallingEdge(A4988 *a498);

#endif /* A4988_H */
