#include "pwm_pin.h"
#include "printf.h"

#define PWM_PIN_DEBUG 1

#if PWM_PIN_DEBUG
#include "serial.h"
#define debug(message, ...) println("{PWM_PIN} " message, ##__VA_ARGS__)
#else
#define debug(message, ...)
#endif

void PWM_Pin_Start(PWM_Pin pin)
{
#if PWM_PIN_DEBUG
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	debug("Start     (%s)", s_pin);
#endif
	HAL_TIM_PWM_Start(pin.tim, pin.channel);
}

void PWM_Pin_Stop(PWM_Pin pin)
{
#if PWM_PIN_DEBUG
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	debug("Stop      (%s)", s_pin);
#endif
	HAL_TIM_PWM_Stop(pin.tim, pin.channel);
}

void PWM_Pin_SetDutyCycle(PWM_Pin pin, uint8_t percentage)
{
#if PWM_PIN_DEBUG
	char s_pin[8];
	PWM_Pin_ToString(pin, s_pin, 8);
	debug("DutyCycle (%s) := %u%%", s_pin, percentage);
#endif
	__HAL_TIM_SET_COMPARE(
			pin.tim, pin.channel,
			__HAL_TIM_GET_AUTORELOAD(pin.tim) * percentage / 100
	);
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
	return NULL;
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
	return NULL;
}

void PWM_Pin_ToString(PWM_Pin pin, char *buf, size_t buflen)
{
	const char *s_tim = PWM_Pin_TIMToString(pin.tim);
	if (!s_tim) {
		snprintf(buf, buflen, "?");
		return;
	}
	const char *s_ch = PWM_Pin_ChannelToString(pin.channel);
	if (!s_ch) {
		snprintf(buf, buflen, "?");
		return;
	}

	snprintf(buf, buflen, "%s%s", s_tim, s_ch);
}
