#include "serial.h"
#include "stm32l4xx_hal.h"
#include "motors.h"


// =========== FUNCTIONS ===============

void rotateCounterClockwise(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

void rotateClockwise(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

void invertRotation(){
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
}

void stopRotation(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

