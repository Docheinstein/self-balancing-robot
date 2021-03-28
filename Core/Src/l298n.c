#include "l298n.h"
#include "serial.h"

#define L298N_DEBUG 1

#if L298N_DEBUG
#include "serial.h"
#define debug(message, ...) println("{L298N} " message, ##__VA_ARGS__)
#else
#define debug(message, ...)
#endif

static L298N_Config l298n_config;

void L298N_Init(L298N_Config config)
{
	l298n_config = config;
#if GPIO_PIN_DEBUG
#define BUFSIZE 8
	char a_dir_1_pin[BUFSIZE] = {'\0'};
	char a_dir_2_pin[BUFSIZE] = {'\0'};
	char a_speed_type[BUFSIZE] = {'\0'};
	char a_speed_pin[BUFSIZE] = {'\0'};
	char b_dir_1_pin[BUFSIZE] = {'\0'};
	char b_dir_2_pin[BUFSIZE] = {'\0'};
	char b_speed_type[BUFSIZE] = {'\0'};
	char b_speed_pin[BUFSIZE] = {'\0'};
	if (config.motor_a.enabled) {
		GPIO_Pin_ToString(config.motor_a.direction_1, a_dir_1_pin, BUFSIZE);
		GPIO_Pin_ToString(config.motor_a.direction_2, a_dir_2_pin, BUFSIZE);
		if (config.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_FIXED) {
			snprintf(a_speed_type, BUFSIZE, "%s", "FIXED");
 		} else if (config.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL) {
			GPIO_Pin_ToString(config.motor_a.speed.digital, a_speed_pin, BUFSIZE);
			snprintf(a_speed_type, BUFSIZE, "%s", "DIGITAL");
		} else if (config.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG) {
			PWM_Pin_ToString(config.motor_a.speed.analog, a_speed_pin, BUFSIZE);
			snprintf(a_speed_type, BUFSIZE, "%s", "ANALOG");
		}
	}
	if (config.motor_b.enabled) {
		GPIO_Pin_ToString(config.motor_b.direction_1, b_dir_1_pin, BUFSIZE);
		GPIO_Pin_ToString(config.motor_b.direction_2, b_dir_2_pin, BUFSIZE);
		if (config.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_FIXED) {
			snprintf(b_speed_type, BUFSIZE, "%s", "FIXED");
 		} else if (config.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL) {
			GPIO_Pin_ToString(config.motor_b.speed.digital, b_speed_pin, BUFSIZE);
			snprintf(b_speed_type, BUFSIZE, "%s", "DIGITAL");
		} else if (config.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG) {
			PWM_Pin_ToString(config.motor_b.speed.analog, b_speed_pin, BUFSIZE);
			snprintf(b_speed_type, BUFSIZE, "%s", "ANALOG");
		}
	}
#undef BUFSIZE
	debug(
		"Initialized" SERIAL_ENDL
		"Motor A" SERIAL_ENDL
		"- enabled: %d" SERIAL_ENDL
		"- direction_1: %s" SERIAL_ENDL
		"- direction_2: %s" SERIAL_ENDL
		"- speed: %s (%s)" SERIAL_ENDL
		"Motor B" SERIAL_ENDL
		"- enabled: %d" SERIAL_ENDL
		"- direction_1: %s" SERIAL_ENDL
		"- direction_2: %s" SERIAL_ENDL
		"- speed: %s (%s)" SERIAL_ENDL
		,
		config.motor_a.enabled,
		a_dir_1_pin,
		a_dir_2_pin,
		a_speed_pin, a_speed_type,
		config.motor_b.enabled,
		b_dir_1_pin,
		b_dir_2_pin,
		b_speed_pin, b_speed_type
	);
#endif // GPIO_PIN_DEBUG
}

void L298N_Forward(uint8_t percentage)
{
	debug("Forward %u%%", percentage);

	// TODO: HAL_GPIO_WritePin supports multiple pins in a single write
	if (l298n_config.motor_a.enabled) {
		if (l298n_config.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL)
			GPIO_Pin_High(l298n_config.motor_a.speed.digital);
		else if (l298n_config.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG) {
			PWM_Pin_Start(l298n_config.motor_a.speed.analog);
			PWM_Pin_SetDutyCycle(l298n_config.motor_a.speed.analog, percentage);
		}
		GPIO_Pin_High(l298n_config.motor_a.direction_1);
		GPIO_Pin_Low(l298n_config.motor_a.direction_2);
	}

	// TODO: probably is inverted
	if (l298n_config.motor_b.enabled) {
		if (l298n_config.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL)
			GPIO_Pin_High(l298n_config.motor_b.speed.digital);
		else if (l298n_config.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG) {
			PWM_Pin_Start(l298n_config.motor_b.speed.analog);
			PWM_Pin_SetDutyCycle(l298n_config.motor_b.speed.analog, percentage);
		}
		GPIO_Pin_High(l298n_config.motor_b.direction_1);
		GPIO_Pin_Low(l298n_config.motor_b.direction_2);
	}
}

void L298N_Backward(uint8_t percentage)
{
	debug("Backward %u%%", percentage);

	if (l298n_config.motor_a.enabled) {
		if (l298n_config.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL)
			GPIO_Pin_High(l298n_config.motor_a.speed.digital);
		else if (l298n_config.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG) {
			PWM_Pin_Start(l298n_config.motor_a.speed.analog);
			PWM_Pin_SetDutyCycle(l298n_config.motor_a.speed.analog, percentage);
		}
		GPIO_Pin_Low(l298n_config.motor_a.direction_1);
		GPIO_Pin_High(l298n_config.motor_a.direction_2);
	}

	// TODO: probably is inverted
	if (l298n_config.motor_b.enabled) {
		if (l298n_config.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL)
			GPIO_Pin_High(l298n_config.motor_b.speed.digital);
		else if (l298n_config.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG) {
			PWM_Pin_Start(l298n_config.motor_b.speed.analog);
			PWM_Pin_SetDutyCycle(l298n_config.motor_b.speed.analog, percentage);
		}
		GPIO_Pin_Low(l298n_config.motor_b.direction_1);
		GPIO_Pin_High(l298n_config.motor_b.direction_2);
	}
}

void L298N_Stop()

{
	debug("Stopping");

	if (l298n_config.motor_a.enabled) {
		if (l298n_config.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL)
			GPIO_Pin_Low(l298n_config.motor_a.speed.digital);
		else if (l298n_config.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG)
			PWM_Pin_Stop(l298n_config.motor_a.speed.analog);
		GPIO_Pin_Low(l298n_config.motor_a.direction_1);
		GPIO_Pin_Low(l298n_config.motor_a.direction_2);
	}

	// TODO: probably is inverted
	if (l298n_config.motor_b.enabled) {
		if (l298n_config.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL)
			GPIO_Pin_Low(l298n_config.motor_b.speed.digital);
		else if (l298n_config.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG)
			PWM_Pin_Stop(l298n_config.motor_b.speed.analog);
		GPIO_Pin_Low(l298n_config.motor_b.direction_1);
		GPIO_Pin_Low(l298n_config.motor_b.direction_2);
	}
}
