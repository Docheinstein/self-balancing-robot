#ifndef L298N_H
#define L298N_H

#include <stdbool.h>
#include "gpio_pin.h"
#include "pwm_pin.h"
#include "bits.h"

#define L298N_MOTOR_A		BIT(0)
#define L298N_MOTOR_B		BIT(1)
#define L298N_MOTOR_BOTH	(L298N_MOTOR_A | L298N_MOTOR_B)

typedef enum {
	L298N_MOTOR_SPEED_TYPE_FIXED, 	// EN jumper
	L298N_MOTOR_SPEED_TYPE_DIGITAL, // GPIO
	L298N_MOTOR_SPEED_TYPE_ANALOG  // PWM
} L298N_MotorSpeedType;

typedef struct L298N_MotorConfig {
	bool inverted;
	GPIO_Pin direction_1;
	GPIO_Pin direction_2;
	L298N_MotorSpeedType speed_type;
	union {
		GPIO_Pin digital;
		PWM_Pin analog;
	} speed;
} L298N_MotorConfig;

typedef struct L298N_Config {
	L298N_MotorConfig motor_a;
	L298N_MotorConfig motor_b;
} L298N_Config;



void L298N_Init(L298N_Config config);
void L298N_Forward(uint8_t motors, float percentage);
void L298N_Backward(uint8_t motors, float percentage);
void L298N_Stop(uint8_t motors);

#endif /* L298N_H */
