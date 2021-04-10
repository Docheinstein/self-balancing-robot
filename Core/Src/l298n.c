#include <std.h>
#include "l298n.h"
#include "serial.h"
#include "verbose.h"

#define VERBOSE_FMT(fmt) "{L298N} " fmt


L298N_Config config;

void L298N_Init(L298N_Config c)
{
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
	GPIO_Pin_ToString(c.motor_a.direction_1, a_dir_1_pin, BUFSIZE);
	GPIO_Pin_ToString(c.motor_a.direction_2, a_dir_2_pin, BUFSIZE);
	if (c.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_FIXED) {
		snprintf(a_speed_type, BUFSIZE, "%s", "FIXED");
	} else if (c.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL) {
		GPIO_Pin_ToString(c.motor_a.speed.digital, a_speed_pin, BUFSIZE);
		snprintf(a_speed_type, BUFSIZE, "%s", "DIGITAL");
	} else if (c.motor_a.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG) {
		PWM_Pin_ToString(c.motor_a.speed.analog, a_speed_pin, BUFSIZE);
		snprintf(a_speed_type, BUFSIZE, "%s", "ANALOG");
	}
	GPIO_Pin_ToString(c.motor_b.direction_1, b_dir_1_pin, BUFSIZE);
	GPIO_Pin_ToString(c.motor_b.direction_2, b_dir_2_pin, BUFSIZE);
	if (c.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_FIXED) {
		snprintf(b_speed_type, BUFSIZE, "%s", "FIXED");
	} else if (c.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL) {
		GPIO_Pin_ToString(c.motor_b.speed.digital, b_speed_pin, BUFSIZE);
		snprintf(b_speed_type, BUFSIZE, "%s", "DIGITAL");
	} else if (c.motor_b.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG) {
		PWM_Pin_ToString(c.motor_b.speed.analog, b_speed_pin, BUFSIZE);
		snprintf(b_speed_type, BUFSIZE, "%s", "ANALOG");
	}
#undef BUFSIZE
	verboseln(
		"Initialized" SERIAL_ENDL
		"Motor A" SERIAL_ENDL
		"- inverted: %s" SERIAL_ENDL
		"- direction_1: %s" SERIAL_ENDL
		"- direction_2: %s" SERIAL_ENDL
		"- speed: %s (%s)" SERIAL_ENDL
		"Motor B" SERIAL_ENDL
		"- inverted: %s" SERIAL_ENDL
		"- direction_1: %s" SERIAL_ENDL
		"- direction_2: %s" SERIAL_ENDL
		"- speed: %s (%s)"
		,
		BOOL_TO_STR(c.motor_a.inverted),
		a_dir_1_pin,
		a_dir_2_pin,
		a_speed_pin, a_speed_type,
		BOOL_TO_STR(c.motor_b.inverted),
		b_dir_1_pin,
		b_dir_2_pin,
		b_speed_pin, b_speed_type
	);
#endif // VERBOSE

	config = c;
}


static void L298N_MotorSetEnabled(L298N_MotorConfig motor, bool enable)
{
	if (enable) {
		if (motor.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL)
			GPIO_Pin_High(motor.speed.digital);
		else if (motor.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG) {
			PWM_Pin_Start(motor.speed.analog);
		}
	} else {
		if (motor.speed_type == L298N_MOTOR_SPEED_TYPE_DIGITAL)
			GPIO_Pin_Low(motor.speed.digital);
		else if (motor.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG)
			PWM_Pin_Stop(motor.speed.analog);
	}
}

static void L298N_MotorSetSpeed(L298N_MotorConfig motor, float percentage)
{
	if (motor.speed_type == L298N_MOTOR_SPEED_TYPE_ANALOG)
		PWM_Pin_SetDutyCycle(
				motor.speed.analog,
				rangef(motor.speed_factor * percentage, 0.0f, 100.0f));
}

static void L298N_MotorSetDirection(
		L298N_MotorConfig motor, bool dir_1_high, bool dir_2_high)
{
	GPIO_Pin_Write(motor.direction_1, dir_1_high);
	GPIO_Pin_Write(motor.direction_2, dir_2_high);
}

static void L298N_MotorForward(L298N_MotorConfig motor, float percentage)
{
	L298N_MotorSetDirection(motor, motor.inverted, !motor.inverted);
	L298N_MotorSetSpeed(motor, percentage);
	L298N_MotorSetEnabled(motor, true);
}

static void L298N_MotorBackward(L298N_MotorConfig motor, float percentage)
{
	L298N_MotorSetDirection(motor, !motor.inverted, motor.inverted);
	L298N_MotorSetSpeed(motor, percentage);
	L298N_MotorSetEnabled(motor, true);
}

static void L298N_MotorStop(L298N_MotorConfig motor)
{
	L298N_MotorSetDirection(motor, false, false);
	L298N_MotorSetEnabled(motor, false);
}

void L298N_Forward(uint8_t motors, float percentage)
{
	if (motors & L298N_MOTOR_A) {
		verboseln("A: Forward %.2f%%", percentage);
		L298N_MotorForward(config.motor_a, percentage);
	}
	if (motors & L298N_MOTOR_B) {
		verboseln("B: Forward %.2f%%", percentage);
		L298N_MotorForward(config.motor_b, percentage);
	}
}

void L298N_Backward(uint8_t motors, float percentage)
{
	if (motors & L298N_MOTOR_A) {
		verboseln("A: Backward %.2f%%", percentage);
		L298N_MotorBackward(config.motor_a, percentage);
	}
	if (motors & L298N_MOTOR_B) {
		verboseln("B: Backward %.2f%%", percentage);
		L298N_MotorBackward(config.motor_b, percentage);
	}
}

void L298N_Stop(uint8_t motors)
{
	if (motors & L298N_MOTOR_A) {
		verboseln("A: Stop");
		L298N_MotorStop(config.motor_a);
	}
	if (motors & L298N_MOTOR_B) {
		verboseln("B: Stop");
		L298N_MotorStop(config.motor_b);
	}
}
