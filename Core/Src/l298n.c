#include <std.h>
#include "l298n.h"
#include "serial.h"
#include "verbose.h"

#define VERBOSE_FMT(fmt) "{L298N} " fmt

static L298N_Config l298n_config;

void L298N_Init(L298N_Config config)
{
	l298n_config = config;
#if VERBOSE
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
	verboseln(
		"Initialized" SERIAL_ENDL
		"Motor A" SERIAL_ENDL
		"- enabled: %s" SERIAL_ENDL
		"- inverted: %s" SERIAL_ENDL
		"- direction_1: %s" SERIAL_ENDL
		"- direction_2: %s" SERIAL_ENDL
		"- speed: %s (%s)" SERIAL_ENDL
		"Motor B" SERIAL_ENDL
		"- enabled: %s" SERIAL_ENDL
		"- inverted: %s" SERIAL_ENDL
		"- direction_1: %s" SERIAL_ENDL
		"- direction_2: %s" SERIAL_ENDL
		"- speed: %s (%s)"
		,
		BOOL_TO_STRING(config.motor_a.enabled),
		BOOL_TO_STRING(config.motor_a.inverted),
		a_dir_1_pin,
		a_dir_2_pin,
		a_speed_pin, a_speed_type,
		BOOL_TO_STRING(config.motor_b.enabled),
		BOOL_TO_STRING(config.motor_b.inverted),
		b_dir_1_pin,
		b_dir_2_pin,
		b_speed_pin, b_speed_type
	);
#endif // DEBUG
}

static void L298N_MotorSetSpeed(L298N_MotorConfig motor, bool active, uint8_t percentage)
{
	if (!active) {
		if (motor.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL)
			GPIO_Pin_Low(motor.speed.digital);
		else if (motor.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG)
			PWM_Pin_Stop(motor.speed.analog);
	} else {
		if (motor.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL)
			GPIO_Pin_High(motor.speed.digital);
		else if (motor.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG) {
			PWM_Pin_Start(motor.speed.analog);
			PWM_Pin_SetDutyCycle(motor.speed.analog, percentage);
		}
	}

}

static void L298N_MotorSetDirection(
		L298N_MotorConfig motor, bool direction_1_high, bool direction_2_high)
{
	if (!motor.enabled)
		return;

	if (direction_1_high)
		GPIO_Pin_High(motor.direction_1);
	else
		GPIO_Pin_Low(motor.direction_1);

	if (direction_2_high)
		GPIO_Pin_High(motor.direction_2);
	else
		GPIO_Pin_Low(motor.direction_2);
}

static void L298N_MotorForward(L298N_MotorConfig motor, uint8_t percentage)
{
	L298N_MotorSetDirection(motor, motor.inverted, !motor.inverted);
	L298N_MotorSetSpeed(motor, true, percentage);
}

static void L298N_MotorBackward(L298N_MotorConfig motor, uint8_t percentage)
{
	L298N_MotorSetDirection(motor, !motor.inverted, motor.inverted);
	L298N_MotorSetSpeed(motor, true, percentage);
}

void L298N_Forward(uint8_t percentage)
{
	verboseln("Forward %u%%", percentage);
	L298N_MotorForward(l298n_config.motor_a, percentage);
	L298N_MotorForward(l298n_config.motor_b, percentage);
}

void L298N_Backward(uint8_t percentage)
{
	verboseln("Backward %u%%", percentage);
	L298N_MotorBackward(l298n_config.motor_a, percentage);
	L298N_MotorBackward(l298n_config.motor_b, percentage);
}

void L298N_Stop()
{
	verboseln("Stop");
	L298N_MotorSetSpeed(l298n_config.motor_a, false, 0);
	L298N_MotorSetSpeed(l298n_config.motor_b, false, 0);
}
