#include "utime.h"

static TIM_HandleTypeDef *utim;

void UTime_Init(TIM_HandleTypeDef *tim)
{
	utim = tim;
	utim->Instance->PSC = (SystemCoreClock / 1000000) - 1;
	utim->Instance->ARR = UINT32_MAX;
	utim->Instance->EGR = TIM_EGR_UG; // trigger update event
}

void UTime_Start()
{
	HAL_TIM_Base_Start(utim);
}

uint32_t UTime_GetMicroseconds()
{
	return utim->Instance->CNT;
}

void UTime_SleepMicroseconds(uint32_t us)
{
	uint32_t sleep_start = UTime_GetMicroseconds();
	while (UTime_GetMicroseconds() - sleep_start < us);
}
