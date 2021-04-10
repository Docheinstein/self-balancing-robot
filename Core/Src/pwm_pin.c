#include "pwm_pin.h"
#include "printf.h"
#include "math.h"

#include "verbose.h"

#define VERBOSE_FMT(fmt) "{PWM_Pin} " fmt

void PWM_Pin_Start(PWM_Pin pin)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	verboseln("Start     (%s)", s_pin);
#endif
	HAL_TIM_PWM_Start(pin.tim, pin.channel);
}

void PWM_Pin_Stop(PWM_Pin pin)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	verboseln("Stop      (%s)", s_pin);
#endif
	HAL_TIM_PWM_Stop(pin.tim, pin.channel);
}

void PWM_Pin_SetDutyCycle(PWM_Pin pin, float percentage)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	verboseln("DutyCycle (%s) := %.2f%%", s_pin, 100 * percentage);
#endif
	__HAL_TIM_SET_COMPARE(
			pin.tim, pin.channel,
			lroundf(__HAL_TIM_GET_AUTORELOAD(pin.tim) * percentage / 100)
	);
}

void PWM_Pin_SetPSC(PWM_Pin pin, uint16_t psc)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	verboseln("PSC (%s) := %u%%", s_pin, psc);
#endif
	__HAL_TIM_SET_PRESCALER(pin.tim, psc);
}

void PWM_Pin_SetARR(PWM_Pin pin, uint16_t arr)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	verboseln("ARR (%s) := %u%%", s_pin, arr);
#endif
	__HAL_TIM_SET_AUTORELOAD(pin.tim, arr);
}


static const char * PWM_Pin_TIMToString(TIM_HandleTypeDef *tim)
{
	if (tim->Instance == TIM1)
		return "TIM1";
	if (tim->Instance == TIM2)
		return "TIM2";
	if (tim->Instance == TIM3)
		return "TIM3";
	if (tim->Instance == TIM4)
		return "TIM4";
	if (tim->Instance == TIM5)
		return "TIM5";
	if (tim->Instance == TIM6)
		return "TIM6";
	if (tim->Instance == TIM7)
		return "TIM7";
	if (tim->Instance == TIM8)
		return "TIM8";
	if (tim->Instance == TIM15)
		return "TIM15";
	if (tim->Instance == TIM16)
		return "TIM16";
	if (tim->Instance == TIM17)
		return "TIM17";
	return "?";
}

static const char * PWM_Pin_ChannelToString(uint32_t channel)
{
	if (channel == TIM_CHANNEL_1)
		return "CH1";
	if (channel == TIM_CHANNEL_2)
		return "CH2";
	if (channel == TIM_CHANNEL_3)
		return "CH3";
	if (channel == TIM_CHANNEL_4)
		return "CH4";
	if (channel == TIM_CHANNEL_5)
		return "CH5";
	if (channel == TIM_CHANNEL_6)
		return "CH6";
	return "?";
}

void PWM_Pin_ToString(PWM_Pin pin, char *buf, size_t buflen)
{
	const char *s_tim = PWM_Pin_TIMToString(pin.tim);
	const char *s_ch = PWM_Pin_ChannelToString(pin.channel);
	snprintf(buf, buflen, "%s%s", s_tim, s_ch);
}
