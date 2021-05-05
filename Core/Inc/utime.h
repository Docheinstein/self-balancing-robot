#ifndef UTIME_H
#define UTIME_H

#include <stdint.h>
#include "stm32l4xx_hal.h"

// ===== ALIASES =====

#define GetMilliseconds() 		HAL_GetTick()
#define GetMicroseconds() 		UTime_GetMicroseconds()
#define SleepMicroseconds(us) 	UTime_SleepMicroseconds(us)

// ===== FUNCTIONS =====

void UTime_Init(TIM_HandleTypeDef *tim);

/* Start the underlying TIM */
void UTime_Start();

/* Get the current microseconds (from UTime_Start()) */
uint32_t UTime_GetMicroseconds();

/* Sleep for the given amount of microseconds (BLOCKING!) */
void UTime_SleepMicroseconds(uint32_t us);

#endif /* UTIME_H */
