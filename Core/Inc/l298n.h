#ifndef L298N_H
#define L298N_H

#include <stdbool.h>
#include "gpio_pin.h"
#include "pwm_pin.h"

typedef enum {
	L298N_MOTOR_SPEED_TYPE_FIXED, // EN jumper
	L298N_MOTOR_SPEED_TYPE_DIGITAL,
	L298N_MOTOR_SPEED_TYPE_ANALOG,
} L298N_MotorSpeedType;

typedef struct L298N_MotorConfig {
	bool enabled;
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
void L298N_Forward(uint8_t percentage);
void L298N_Backward(uint8_t percentage);
void L298N_Stop();

#endif /* L298N_H */
