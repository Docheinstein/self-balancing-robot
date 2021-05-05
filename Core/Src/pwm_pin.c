#include "pwm_pin.h"
#include "printf.h"
#include "math.h"

#include "verbose.h"
#include "serial.h"

#define VERBOSE_FMT(fmt) "{PWM_Pin} " fmt

void PWM_Pin_Start(PWM_Pin pin)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	averboseln("Start     (%s)", s_pin);
#endif
	HAL_TIM_PWM_Start(pin.tim, pin.channel);
}

void PWM_Pin_Stop(PWM_Pin pin)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	averboseln("Stop      (%s)", s_pin);
#endif
	HAL_TIM_PWM_Stop(pin.tim, pin.channel);
}

void PWM_Pin_Set(PWM_Pin pin, float frequency, float percentage)
{
	PWM_Pin_SetFrequency(pin, frequency);
	PWM_Pin_SetDutyCycle(pin, percentage);
	__HAL_TIM_SET_COUNTER(pin.tim, 0);
}

void PWM_Pin_SetFrequency(PWM_Pin pin, float frequency)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	averboseln("Frequency (%s) := %.2fHz", s_pin, frequency);
#endif

	/**
	 * Change the frequency means set the right value of PSC and ARR so that:
	 * F = CLK / ((PSC + 1) * (ARR + 1))
	 * (PSC + 1) * (ARR + 1) = CLK / F
	 *
	 * Therefore
	 * (PSC + 1) = (CLK / F) * (1 / (ARR + 1))

	 * PSC       = (CLK / F) * (1 / (ARR + 1)) - 1
	 */

	float K = SystemCoreClock / frequency;
	if (frequency > K)
		return;
	uint32_t M = pin.type == PWM_PIN_TYPE_32_BIT ? UINT32_MAX : UINT16_MAX;

	// Take the ceil so that the equation yields arr < M
	uint32_t psc =  ceil((K / (M)) - 1);
	uint32_t arr = floor((K / (psc + 1))    - 1);

	averboseln("PSC=%u | ARR=%u | F=%.3fHz",
			  psc, arr, (double) SystemCoreClock / ((psc + 1) * (arr + 1)));

	PWM_Pin_SetPSC(pin, psc);
	PWM_Pin_SetARR(pin, arr);
}

void PWM_Pin_SetDutyCycle(PWM_Pin pin, float percentage)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	averboseln("DutyCycle (%s) := %.2f%%", s_pin, percentage);
#endif
	PWM_Pin_SetCCR(
			pin,
			lroundf(PWM_Pin_GetARR(pin) * percentage / 100)
	);
}

void PWM_Pin_SetPSC(PWM_Pin pin, uint32_t psc)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	averboseln("PSC (%s) := %u", s_pin, psc);
#endif
	__HAL_TIM_SET_PRESCALER(pin.tim, psc);
}

void PWM_Pin_SetARR(PWM_Pin pin, uint32_t arr)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	averboseln("ARR (%s) := %u", s_pin, arr);
#endif
	__HAL_TIM_SET_AUTORELOAD(pin.tim, arr);
}


void PWM_Pin_SetCCR(PWM_Pin pin, uint32_t ccr)
{
#if VERBOSE
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	averboseln("CCR (%s) := %u", s_pin, ccr);
#endif
	__HAL_TIM_SET_COMPARE(pin.tim, pin.channel, ccr);
}

uint32_t PWM_Pin_GetPSC(PWM_Pin pin)
{
	// TODO
	// Currently there is not __HAL_TIM_GET_PRESCALER in stm32l4xx_hal_tim.h.
	// Use it if it will ever be added.
	return pin.tim->Instance->PSC;
}

uint32_t PWM_Pin_GetARR(PWM_Pin pin)
{
	return __HAL_TIM_GET_AUTORELOAD(pin.tim);
}

uint32_t PWM_Pin_GetCCR(PWM_Pin pin)
{
	return __HAL_TIM_GET_COMPARE(pin.tim, pin.channel);
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
