#include "a4988.h"
#include "verbose.h"
#include "utime.h"

#define VERBOSE_FMT(fmt) "{A4988} " fmt

void A4988_Init(A4988 *a4988, A4988_Config config)
{
#if 1
#define BUFSIZE 8
	char dir[BUFSIZE] = {'\0'};
	char step[BUFSIZE] = {'\0'};
	char sleep[BUFSIZE] = {'\0'};
	GPIO_Pin_ToString(config.direction, dir, BUFSIZE);
	GPIO_Pin_ToString(config.step, step, BUFSIZE);
	GPIO_Pin_ToString(config.sleep, sleep, BUFSIZE);
#undef BUFSIZE
	averboseln(
		"Initialized" SERIAL_ENDL
		"- direction: %s" SERIAL_ENDL
		"- step: %s" SERIAL_ENDL
		"- sleep: %s"
		,
		dir,
		step,
		sleep
	);
#endif

	a4988->config = config;
	a4988->enabled = false;
}

void A4988_Enable(A4988 *a4988)
{
	averboseln("Enabling...");
	a4988->enabled = true;
	GPIO_Pin_High(a4988->config.sleep);
}

void A4988_Disable(A4988 *a4988)
{
	averboseln("Disabling...");
	a4988->enabled = false;
	GPIO_Pin_Low(a4988->config.sleep);
}

void A4988_StepForward(A4988 *a4988, bool wait_falling_edge)
{
	averboseln("Forward");
	GPIO_Pin_Write(a4988->config.direction, a4988->config.inverted);
	GPIO_Pin_High(a4988->config.step);
	if (wait_falling_edge) {
		UTime_SleepMicroseconds(A4988_MINIMUM_STEP_PULSE_MICROSECONDS);
		A4988_StepFallingEdge(a4988);
	}
}

void A4988_StepBackward(A4988 *a4988, bool wait_falling_edge)
{
	averboseln("Backward");
	GPIO_Pin_Write(a4988->config.direction, !a4988->config.inverted);
	GPIO_Pin_High(a4988->config.step);
	if (wait_falling_edge) {
		UTime_SleepMicroseconds(A4988_MINIMUM_STEP_PULSE_MICROSECONDS);
		A4988_StepFallingEdge(a4988);
	}
}

void A4988_StepFallingEdge(A4988 *a4988)
{
	averboseln("Falling Edge");
	GPIO_Pin_Low(a4988->config.step);
}

