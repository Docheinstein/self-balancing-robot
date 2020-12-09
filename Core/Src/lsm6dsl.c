#include "lsm6dsl.h"

#define LSM6DSL_DEBUG 1
#define LSM6DSL_RW_MAX_DELAY 100

#if LSM6DSL_DEBUG
#include "serial.h"
#define debug(message, ...) println("[LSM6DSL] " message, ##__VA_ARGS__)
#else
#define debug(message, ...)
#endif

extern I2C_HandleTypeDef *i2c;

HAL_StatusTypeDef LSM6DSL_Read_Register(uint8_t reg, uint8_t *out)
{

	debug("r[%x] ...", reg);

	HAL_StatusTypeDef status;

	if ((status = HAL_I2C_Master_Transmit(
			i2c, LSM6DSL_I2C_ADDR, &reg, 1, LSM6DSL_RW_MAX_DELAY)) != HAL_OK) {
		debug("Failed to read (HAL_I2C_Master_Transmit failed)");
		return status;
	}

	if ((status = HAL_I2C_Master_Receive(
			i2c, LSM6DSL_I2C_ADDR, out, 1, LSM6DSL_RW_MAX_DELAY)) != HAL_OK) {
		debug("Failed to read (HAL_I2C_Master_Receive failed)");
		return status;
	}

	debug("r[%x] = %x", reg, *out);

	return status;
}

HAL_StatusTypeDef LSM6DSL_Read_Registers_2(uint8_t reg_h, uint8_t reg_l, uint16_t *out)
{
	HAL_StatusTypeDef status;
	uint8_t val_h, val_l;

	if ((status = LSM6DSL_Read_Register(reg_h, &val_h)) != HAL_OK)
		return status;

	if ((status = LSM6DSL_Read_Register(reg_h, &val_l)) != HAL_OK)
		return status;

	*out = (val_h << 8) | val_l;

	return status;
}

HAL_StatusTypeDef LSM6DSL_Write_Register(uint8_t reg, uint8_t value)
{
	debug("w[%x] <- %x ...", reg, value);

	uint8_t sendbuf[] = {reg, value};

	HAL_StatusTypeDef status;
	if ((status = HAL_I2C_Master_Transmit(
			i2c, LSM6DSL_I2C_ADDR, sendbuf, 2, LSM6DSL_RW_MAX_DELAY)) != HAL_OK) {
		debug("Failed to write");
		return status;
	}

	debug("w[%x] <- %x", reg, value);

	return status;
}

void LSM6DSL_Assert_Healthy()
{
	uint8_t value;
	if (LSM6DSL_Read_Register(LSM6DSL_REG_WHO_AM_AI, &value) != HAL_OK ||
		value != 0x6A) {
		debug("LSM6DSL_Assert_Healthy failed: WHO_AM_I != 0x6A");
		// exit(255);
	}
}
