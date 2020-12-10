#include "lsm6dsl.h"

#define LSM6DSL_DEBUG 0

#if LSM6DSL_DEBUG
#include "serial.h"
#define debug(message, ...) println("{LSM6DSL} " message, ##__VA_ARGS__)
#else
#define debug(message, ...)
#endif

// TODO: not hardcoded?
#define LSM6DSL_RW_MAX_DELAY 100

// Block Data Update: avoid read L and H parts of different samples
#define LSM6DSL_BDU 1

#define HAL_CHECK(op) \
if ((status = op) != HAL_OK) {\
	return status;\
}

#define HAL_CHECK_X(op, msg) \
if ((status = op) != HAL_OK) {\
	debug(msg);\
	return status;\
}

extern I2C_HandleTypeDef *i2c;

static LSM6DSL_Accelerometer_Scale xl_scale_mask;
static LSM6DSL_Gyroscope_Scale g_scale_mask;

static int xl_scale;
static int g_scale;


static void _set_accelerometer_scale(LSM6DSL_Accelerometer_Scale scale)
{
	xl_scale_mask = scale;
	switch(xl_scale_mask)
	{
		case LSM6DSL_FS_XL_2_G:  xl_scale = 2; break;
		case LSM6DSL_FS_XL_4_G:  xl_scale = 4; break;
		case LSM6DSL_FS_XL_8_G:  xl_scale = 8; break;
		case LSM6DSL_FS_XL_16_G: xl_scale = 16; break;
	}
}

static void _set_gyroscope_scale(LSM6DSL_Accelerometer_Scale scale)
{
	g_scale_mask = scale;
	switch(g_scale_mask)
	{
		case LSM6DSL_FS_G_250_DPS:  g_scale = 250; break;
		case LSM6DSL_FS_G_500_DPS:  g_scale = 500; break;
		case LSM6DSL_FS_G_1000_DPS: g_scale = 1000; break;
		case LSM6DSL_FS_G_2000_DPS: g_scale = 2000; break;
	}
}


HAL_StatusTypeDef LSM6DSL_Read_Register(uint8_t reg, uint8_t *out)
{
	HAL_StatusTypeDef status;

	debug("r[%x] ...", reg);

	HAL_CHECK_X(
		HAL_I2C_Master_Transmit(
			i2c, LSM6DSL_I2C_ADDR, &reg, 1, LSM6DSL_RW_MAX_DELAY),
		"Failed to read (HAL_I2C_Master_Transmit failed)"
	)

	HAL_CHECK_X(
		HAL_I2C_Master_Receive(
			i2c, LSM6DSL_I2C_ADDR, out, 1, LSM6DSL_RW_MAX_DELAY),
		"Failed to read (HAL_I2C_Master_Receive failed)"
	)

	debug("r[%x] = %x", reg, *out);

	return status;
}

HAL_StatusTypeDef LSM6DSL_Read_Registers_2(uint8_t reg_l, uint8_t reg_h, uint16_t *out)
{
	HAL_StatusTypeDef status;
	uint8_t val_l, val_h;

	HAL_CHECK(LSM6DSL_Read_Register(reg_l, &val_l))
	HAL_CHECK(LSM6DSL_Read_Register(reg_h, &val_h))

	*out = (val_h << 8) | val_l;

	return status;
}

HAL_StatusTypeDef LSM6DSL_Write_Register(uint8_t reg, uint8_t value)
{
	HAL_StatusTypeDef status;
	uint8_t sendbuf[] = {reg, value};

	debug("w[%x] <- %x ...", reg, value);

	HAL_CHECK_X(
		HAL_I2C_Master_Transmit(
			i2c, LSM6DSL_I2C_ADDR, sendbuf, 2, LSM6DSL_RW_MAX_DELAY),
		"Failed to write"
	)

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

HAL_StatusTypeDef LSM6DSL_Enable_Accelerometer(
		LSM6DSL_Frequency frequency,
		LSM6DSL_Accelerometer_Scale scale)
{
	HAL_StatusTypeDef status;
	debug("Enabling accelerometer");

	_set_accelerometer_scale(scale);
	uint8_t value = frequency | scale;

#if LSM6DSL_BDU
	HAL_CHECK(
			LSM6DSL_Write_Register(
		LSM6DSL_REG_CTRL3_C, LSM6DSL_REG_CTRL3_C_BIT_BDU))
#endif

	HAL_CHECK(
		LSM6DSL_Write_Register(LSM6DSL_REG_CTRL1_XL, value)
	)

	return status;
}

HAL_StatusTypeDef LSM6DSL_Enable_Gyroscope(
		LSM6DSL_Frequency frequency,
		LSM6DSL_Gyroscope_Scale scale)
{
	HAL_StatusTypeDef status;
	debug("Enabling gyroscope");

	_set_gyroscope_scale(scale);
	uint8_t value = frequency | scale;

#if LSM6DSL_BDU
	HAL_CHECK(LSM6DSL_Write_Register(
		LSM6DSL_REG_CTRL3_C, LSM6DSL_REG_CTRL3_C_BIT_BDU))
#endif

	HAL_CHECK(
		LSM6DSL_Write_Register(LSM6DSL_REG_CTRL2_G, value)
	)

	return status;
}

HAL_StatusTypeDef LSM6DSL_Disable_Accelerometer()
{
	debug("Disabling accelerometer");
	return LSM6DSL_Write_Register(LSM6DSL_REG_CTRL1_XL, LSM6DSL_POWER_DOWN);
}

HAL_StatusTypeDef LSM6DSL_Disable_Gyroscope()
{
	debug("Disabling gyroscope");
	return LSM6DSL_Write_Register(LSM6DSL_REG_CTRL2_G, LSM6DSL_POWER_DOWN);
}

HAL_StatusTypeDef LSM6DSL_Read_Status(uint8_t *value)
{
	debug("Reading status...");
	return LSM6DSL_Read_Register(LSM6DSL_REG_STATUS, value);
}

bool LSM6DSL_Is_Accelerometer_Data_Ready()
{
	uint8_t lsm6dsl_status;
	HAL_StatusTypeDef status = LSM6DSL_Read_Status(&lsm6dsl_status);

	return status == HAL_OK && (lsm6dsl_status & LSM6DSL_REG_STATUS_BIT_XLDA);
}

bool LSM6DSL_Is_Gyroscope_Data_Ready()
{
	uint8_t lsm6dsl_status;
	HAL_StatusTypeDef status = LSM6DSL_Read_Status(&lsm6dsl_status);

	return status == HAL_OK && (lsm6dsl_status & LSM6DSL_REG_STATUS_BIT_GDA);
}

bool LSM6DSL_Is_Temperature_Data_Ready()
{
	uint8_t lsm6dsl_status;
	HAL_StatusTypeDef status = LSM6DSL_Read_Status(&lsm6dsl_status);

	return status == HAL_OK && (lsm6dsl_status & LSM6DSL_REG_STATUS_BIT_TDA);
}

HAL_StatusTypeDef LSM6DSL_Read_Accelerometer(int16_t *x, int16_t *y, int16_t *z)
{
	// 0 -> 0g

	HAL_StatusTypeDef status;
	uint16_t _x, _y, _z;

	HAL_CHECK(
		LSM6DSL_Read_Registers_2(
			LSM6DSL_REG_OUTX_L_XL, LSM6DSL_REG_OUTX_H_XL, &_x)
	)

	HAL_CHECK(
		LSM6DSL_Read_Registers_2(
			LSM6DSL_REG_OUTY_L_XL, LSM6DSL_REG_OUTY_H_XL, &_y)
	)

	HAL_CHECK(
		LSM6DSL_Read_Registers_2(
			LSM6DSL_REG_OUTZ_L_XL, LSM6DSL_REG_OUTZ_H_XL, &_z)
	)

	debug("LSM6DSL_Read_Accelerometer RAW: (x=%u, y=%u, z=%u) [scale = %dg]",
			_x, _y, _z, xl_scale);

	*x = (int16_t) _x;
	*y = (int16_t) _y;
	*z = (int16_t) _z;

	return status;
}

HAL_StatusTypeDef LSM6DSL_Read_Accelerometer_g(float *x, float *y, float *z)
{
	// Mapping: [2^-15:2^15] -> [-FS:+FS]

	HAL_StatusTypeDef status;
	int16_t _x, _y, _z;

	HAL_CHECK(LSM6DSL_Read_Accelerometer(&_x, &_y, &_z));

	*x = (float) _x * xl_scale / (1 << 15);
	*y = (float) _y * xl_scale / (1 << 15);
	*z = (float) _z * xl_scale / (1 << 15);

	return status;
}

HAL_StatusTypeDef LSM6DSL_Read_Gyroscope(int16_t *x, int16_t *y, int16_t *z)
{
	// 0 -> 0dps

	HAL_StatusTypeDef status;
	uint16_t _x, _y, _z;

	HAL_CHECK(
		LSM6DSL_Read_Registers_2(
			LSM6DSL_REG_OUTX_L_G, LSM6DSL_REG_OUTX_H_G, &_x)
	)

	HAL_CHECK(
		LSM6DSL_Read_Registers_2(
			LSM6DSL_REG_OUTY_L_G, LSM6DSL_REG_OUTY_H_G, &_y)
	)

	HAL_CHECK(
		LSM6DSL_Read_Registers_2(
			LSM6DSL_REG_OUTZ_L_G, LSM6DSL_REG_OUTZ_H_G, &_z)
	)

	debug("LSM6DSL_Read_Gyroscope RAW: (x=%u, y=%u, z=%u) [scale = %ddps]",
			_x, _y, _z, g_scale);

	*x = (int16_t) _x;
	*y = (int16_t) _y;
	*z = (int16_t) _z;

	return status;
}

HAL_StatusTypeDef LSM6DSL_Read_Gyroscope_dps(float *x, float *y, float *z)
{
	// Mapping: [2^-15:2^15] -> [-FS:+FS]

	HAL_StatusTypeDef status;
	int16_t _x, _y, _z;

	HAL_CHECK(LSM6DSL_Read_Gyroscope(&_x, &_y, &_z));

	*x = (float) _x * g_scale / (1 << 15);
	*y = (float) _y * g_scale / (1 << 15);
	*z = (float) _z * g_scale / (1 << 15);

	return status;
}

HAL_StatusTypeDef LSM6DSL_Read_Temperature_C(float *temperature)
{
	// Mapping: [0:255] -> [0 C°:1 C°]
	// 0 -> 25C°

	HAL_StatusTypeDef status;
	uint16_t value;

	HAL_CHECK(
		LSM6DSL_Read_Registers_2(
			LSM6DSL_REG_OUT_TEMP_L, LSM6DSL_REG_OUT_TEMP_H, &value)
	)

	debug("LSM6DSL_Read_Temperature RAW: %u", value);

	*temperature = 25.0f + ((int16_t) value / 256.0f);

	return status;
}

