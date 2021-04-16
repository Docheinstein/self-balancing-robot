#include "a4988.h"
#include "verbose.h"

#define VERBOSE_FMT(fmt) "{A4988} " fmt

void A4988_Init(A4988 *a4988, A4988_Config config)
{
#if 1
#define BUFSIZE 8
	char dir[BUFSIZE] = {'\0'};
	char step[BUFSIZE] = {'\0'};
	GPIO_Pin_ToString(config.direction, dir, BUFSIZE);
	PWM_Pin_ToString(config.step, step, BUFSIZE);
#undef BUFSIZE
	verboseln(
		"Initialized" SERIAL_ENDL
		"- direction: %s" SERIAL_ENDL
		"- step: %s",
		dir,
		step
	);
#endif

	a4988->config = config;
}

void A4988_Enable(A4988 *a4988)
{
	verboseln("Enabling...");
	GPIO_Pin_High(a4988->config.sleep);
	PWM_Pin_Start(a4988->config.step);
}

void A4988_Disable(A4988 *a4988)
{
	verboseln("Disabling...");
	GPIO_Pin_Low(a4988->config.sleep);
	PWM_Pin_Stop(a4988->config.step);
}

static float A4988_RpmToFreq(A4988 *a4988, float rpm)
{
	return rpm * (a4988->config.steps_per_revolution / 60.0f);
}

void A4988_Forward(A4988 *a4988, float rpm)
{
	float freq = A4988_RpmToFreq(a4988, rpm);
	verboseln("Forward %.2f rpm (%f Hz)", rpm, freq);
	GPIO_Pin_Write(a4988->config.direction, a4988->config.inverted);
	PWM_Pin_Set(a4988->config.step, freq, 50.0f);
}

void A4988_Backward(A4988 *a4988, float rpm)
{
	float freq = A4988_RpmToFreq(a4988, rpm);
	verboseln("Backward %.2f rpm (%f Hz)", rpm, freq);
	GPIO_Pin_Write(a4988->config.direction, !a4988->config.inverted);
	PWM_Pin_Set(a4988->config.step, freq, 50.0f);
}

