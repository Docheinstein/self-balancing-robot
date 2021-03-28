#include "gpio_pin.h"
#include "stm32l4xx.h"
#include "printf.h"

#define GPIO_PIN_DEBUG 1

#if GPIO_PIN_DEBUG
#include "serial.h"
#define debug(message, ...) println("{GPIO_PIN} " message, ##__VA_ARGS__)
#else
#define debug(message, ...)
#endif


// TODO: HAL_GPIO_WritePin supports multiple pins in a single write, supporting it here too?


void GPIO_Pin_Toggle(GPIO_Pin pin)
{
#if GPIO_PIN_DEBUG
	char s_pin[8];
	GPIO_Pin_ToString(pin, s_pin, 8);
	debug("Toggle (%s)", s_pin);
#endif
	HAL_GPIO_TogglePin(pin.port, pin.pin);
}

void GPIO_Pin_High(GPIO_Pin pin)
{
#if GPIO_PIN_DEBUG
	char s_pin[8];
	GPIO_Pin_ToString(pin, s_pin, 8);
	debug("High   (%s)", s_pin);
#endif
	HAL_GPIO_WritePin(pin.port, pin.pin, GPIO_PIN_SET);
}

void GPIO_Pin_Low(GPIO_Pin pin)
{
#if GPIO_PIN_DEBUG
	char s_pin[8];
	GPIO_Pin_ToString(pin, s_pin, 8);
	debug("Low    (%s)", s_pin);
#endif
	HAL_GPIO_WritePin(pin.port, pin.pin, GPIO_PIN_RESET);
}

static const char * GPIO_Pin_PortToString(GPIO_TypeDef *port)
{
	if (port == GPIOA)
		return "A";
	if (port == GPIOB)
		return "B";
	if (port == GPIOC)
		return "C";
	if (port == GPIOD)
		return "D";
	if (port == GPIOE)
		return "E";
	if (port == GPIOF)
		return "F";
	if (port == GPIOH)
		return "H";
	return NULL;
}

static const char * GPIO_Pin_PinToString(uint16_t pin)
{
	// TODO: support GPIO_PIN_0 | GPIO_PIN_2 | ...
	if (pin == GPIO_PIN_0)
		return "0";
	if (pin == GPIO_PIN_1)
		return "1";
	if (pin == GPIO_PIN_2)
		return "2";
	if (pin == GPIO_PIN_3)
		return "3";
	if (pin == GPIO_PIN_4)
		return "4";
	if (pin == GPIO_PIN_5)
		return "5";
	if (pin == GPIO_PIN_6)
		return "6";
	if (pin == GPIO_PIN_7)
		return "7";
	if (pin == GPIO_PIN_8)
		return "8";
	if (pin == GPIO_PIN_9)
		return "9";
	if (pin == GPIO_PIN_10)
		return "10";
	if (pin == GPIO_PIN_11)
		return "11";
	if (pin == GPIO_PIN_12)
		return "12";
	if (pin == GPIO_PIN_13)
		return "13";
	if (pin == GPIO_PIN_14)
		return "14";
	if (pin == GPIO_PIN_15)
		return "15";
	return NULL;
}


void GPIO_Pin_ToString(GPIO_Pin pin, char *buf, size_t buflen)
{
	const char *s_port = GPIO_Pin_PortToString(pin.port);
	if (!s_port) {
		snprintf(buf, buflen, "?");
		return;
	}
	const char *s_pin = GPIO_Pin_PinToString(pin.pin);
	if (!s_pin) {
		snprintf(buf, buflen, "?");
		return;
	}

	snprintf(buf, buflen, "P%s%s", s_port, s_pin);
}

