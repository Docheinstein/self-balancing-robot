#include "lsm6dsl.h"
#include "debug.h"

#define DEBUG_FMT(fmt) "{LSM6DSL} " fmt

// TODO: not hardcoded?
#define LSM6DSL_RW_MAX_DELAY 100


#define HAL_CHECK(op) \
if ((status = op) != HAL_OK) {\
	return status;\
}

#define HAL_CHECK_MSG(op, msg) \
if ((status = op) != HAL_OK) {\
	debugln(msg);\
	return status;\
}

static I2C_HandleTypeDef *i2c;

static LSM6DSL_AccelerometerScale xl_scale_mask;
static LSM6DSL_GyroscopeScale g_scale_mask;

static int xl_scale;
static int g_scale;



HAL_StatusTypeDef LSM6DSL_Init(I2C_HandleTypeDef *i)
{
	HAL_StatusTypeDef status;

	debugln("Initializing...");
	i2c = i;

	// Resets LSM6DSL registers to default value; requires ~50 us
	HAL_CHECK(
		LSM6DSL_WriteRegister(
			LSM6DSL_REG_CTRL3_C,
			LSM6DSL_REG_CTRL3_C_BIT_SW_RESET)
	)

	HAL_CHECK(
		LSM6DSL_WriteRegister(
			LSM6DSL_REG_CTRL3_C,
			LSM6DSL_REG_CTRL3_C_BIT_BDU)
	)

	// TODO: is this needed?
//	HAL_CHECK(
//		LSM6DSL_WriteRegister(
//			LSM6DSL_REG_MASTER_CONFIG,
//			LSM6DSL_REG_MASTER_CONFIG_BIT_DRDY_ON_INT1)
//	)

	debugln("Initialized");

	return status;
}


HAL_StatusTypeDef LSM6DSL_ReadRegister(uint8_t reg, uint8_t *out)
{
	HAL_StatusTypeDef status;

	debugln("r[0x%02X] ...", reg);

	HAL_CHECK_MSG(
		HAL_I2C_Master_Transmit(
			i2c, LSM6DSL_I2C_ADDR, &reg, 1, LSM6DSL_RW_MAX_DELAY),
		"Failed to read (HAL_I2C_Master_Transmit failed)"
	)

	HAL_CHECK_MSG(
		HAL_I2C_Master_Receive(
			i2c, LSM6DSL_I2C_ADDR, out, 1, LSM6DSL_RW_MAX_DELAY),
		"Failed to read (HAL_I2C_Master_Receive failed)"
	)

	debugln("r[0x%02X] = 0x%02X", reg, *out);

	return status;
}

HAL_StatusTypeDef LSM6DSL_ReadRegisters_2(uint8_t reg_l, uint8_t reg_h, uint16_t *out)
{
	HAL_StatusTypeDef status;
	uint8_t val_l, val_h;

	HAL_CHECK(LSM6DSL_ReadRegister(reg_l, &val_l))
	HAL_CHECK(LSM6DSL_ReadRegister(reg_h, &val_h))

	*out = (val_h << 8) | val_l;

	return status;
}

HAL_StatusTypeDef LSM6DSL_WriteRegister(uint8_t reg, uint8_t value)
{
	HAL_StatusTypeDef status;
	uint8_t sendbuf[] = {reg, value};

	debugln("w[0x%02X] <- 0x%02X ...", reg, value);

	HAL_CHECK_MSG(
		HAL_I2C_Master_Transmit(
			i2c, LSM6DSL_I2C_ADDR, sendbuf, 2, LSM6DSL_RW_MAX_DELAY),
		"Failed to write"
	)

	debugln("w[0x%02X] = 0x%02X", reg, value);

	return status;
}

bool LSM6DSL_IsHealthy()
{
	uint8_t value;

	if (LSM6DSL_ReadRegister(LSM6DSL_REG_WHO_AM_AI, &value) != HAL_OK ||
		value != 0x6A) {
		debugln("LSM6DSL_AssertHealthy failed: WHO_AM_I != 0x6A");
		return false;
	}

	return true;
}

HAL_StatusTypeDef LSM6DSL_EnableAccelerometer(
		LSM6DSL_Frequency frequency,
		LSM6DSL_AccelerometerScale scale)
{
	HAL_StatusTypeDef status;

	debugln("Enabling accelerometer...");

	xl_scale_mask = scale;
	switch(xl_scale_mask)
	{
		case LSM6DSL_FS_XL_2_G:  xl_scale = 2; break;
		case LSM6DSL_FS_XL_4_G:  xl_scale = 4; break;
		case LSM6DSL_FS_XL_8_G:  xl_scale = 8; break;
		case LSM6DSL_FS_XL_16_G: xl_scale = 16; break;
	}

	uint8_t value = frequency | scale;

	HAL_CHECK(
		LSM6DSL_WriteRegister(LSM6DSL_REG_CTRL1_XL, value)
	)

	debugln("Accelerometer enabled");

	return status;
}

HAL_StatusTypeDef LSM6DSL_EnableGyroscope(
		LSM6DSL_Frequency frequency,
		LSM6DSL_GyroscopeScale scale)
{
	HAL_StatusTypeDef status;
	debugln("Enabling gyroscope...");

	g_scale_mask = scale;
	switch(g_scale_mask)
	{
		case LSM6DSL_FS_G_250_DPS:  g_scale = 250; break;
		case LSM6DSL_FS_G_500_DPS:  g_scale = 500; break;
		case LSM6DSL_FS_G_1000_DPS: g_scale = 1000; break;
		case LSM6DSL_FS_G_2000_DPS: g_scale = 2000; break;
	}

	uint8_t value = frequency | scale;


	HAL_CHECK(
		LSM6DSL_WriteRegister(LSM6DSL_REG_CTRL2_G, value)
	)

	debugln("Gyroscope enabled");

	return status;
}

HAL_StatusTypeDef LSM6DSL_DisableAccelerometer()
{
	HAL_StatusTypeDef status;

	debugln("Disabling accelerometer...");

	HAL_CHECK(
		LSM6DSL_WriteRegister(
			LSM6DSL_REG_CTRL1_XL,
			LSM6DSL_POWER_DOWN)
	)

	debugln("Accelerometer disabled");

	return status;

}

HAL_StatusTypeDef LSM6DSL_DisableGyroscope()
{
	HAL_StatusTypeDef status;

	debugln("Disabling gyroscope...");

	HAL_CHECK(
		LSM6DSL_WriteRegister(
			LSM6DSL_REG_CTRL2_G,
			LSM6DSL_POWER_DOWN)
	)

	debugln("Gyroscope disabled");

	return status;
}


HAL_StatusTypeDef LSM6DSL_SetAccelerometerInterrupt(LSM6DSL_InterruptPin interrupt)
{
	HAL_StatusTypeDef status;
	uint8_t value;

	if (interrupt == LSM6DSL_INT_1) {
		debugln("Setting accelerometer data-ready signal to INT_1");
		HAL_CHECK(
			LSM6DSL_ReadRegister(LSM6DSL_REG_INT1_CTRL, &value)
		)
		value |= LSM6DSL_REG_INT1_CTRL_BIT_INT1_DRDY_XL;
		HAL_CHECK(
			LSM6DSL_WriteRegister(
				LSM6DSL_REG_INT1_CTRL,
				value)
		)
	}
	else if (interrupt == LSM6DSL_INT_2) {
		debugln("Setting accelerometer data-ready signal to INT_2");
		HAL_CHECK(
			LSM6DSL_ReadRegister(LSM6DSL_REG_INT2_CTRL, &value)
		)
		value |= LSM6DSL_REG_INT2_CTRL_BIT_INT2_DRDY_XL;
		HAL_CHECK(
			LSM6DSL_WriteRegister(
				LSM6DSL_REG_INT2_CTRL,
				value)
		)
	}


	return status;
}

HAL_StatusTypeDef LSM6DSL_SetGyroscopeInterrupt(LSM6DSL_InterruptPin interrupt)
{
	HAL_StatusTypeDef status;
	uint8_t value;

	if (interrupt == LSM6DSL_INT_1) {
		debugln("Setting gyroscope data-ready signal to INT_1");
		HAL_CHECK(
			LSM6DSL_ReadRegister(LSM6DSL_REG_INT1_CTRL, &value)
		)
		value |= LSM6DSL_REG_INT1_CTRL_BIT_INT1_DRDY_G;
		HAL_CHECK(
			LSM6DSL_WriteRegister(
				LSM6DSL_REG_INT1_CTRL,
				value)
		)
	}
	else if (interrupt == LSM6DSL_INT_2) {
		debugln("Setting gyroscope data-ready signal to INT_2");
		HAL_CHECK(
			LSM6DSL_ReadRegister(LSM6DSL_REG_INT2_CTRL, &value)
		)
		value |= LSM6DSL_REG_INT2_CTRL_BIT_INT2_DRDY_G;
		HAL_CHECK(
			LSM6DSL_WriteRegister(
				LSM6DSL_REG_INT2_CTRL,
				value)
		)
	}


	return status;
}

HAL_StatusTypeDef LSM6DSL_UnsetAccelerometerInterrupt(LSM6DSL_InterruptPin interrupt)
{
	HAL_StatusTypeDef status;
	uint8_t value;

	if (interrupt == LSM6DSL_INT_1) {
		debugln("Unsetting accelerometer data-ready signal from INT_1");
		HAL_CHECK(
			LSM6DSL_ReadRegister(LSM6DSL_REG_INT1_CTRL, &value)
		)
		value &= ~LSM6DSL_REG_INT1_CTRL_BIT_INT1_DRDY_XL;
		HAL_CHECK(
			LSM6DSL_WriteRegister(
				LSM6DSL_REG_INT1_CTRL,
				value)
		)
	}
	else if (interrupt == LSM6DSL_INT_2) {
		debugln("Unsetting accelerometer data-ready signal from INT_2");
		HAL_CHECK(
			LSM6DSL_ReadRegister(LSM6DSL_REG_INT2_CTRL, &value)
		)
		value &= ~LSM6DSL_REG_INT2_CTRL_BIT_INT2_DRDY_XL;
		HAL_CHECK(
			LSM6DSL_WriteRegister(
				LSM6DSL_REG_INT2_CTRL,
				value)
		)
	}

	return status;
}

HAL_StatusTypeDef LSM6DSL_UnsetGyroscopeInterrupt(LSM6DSL_InterruptPin interrupt)
{
	HAL_StatusTypeDef status;
	uint8_t value;

	if (interrupt == LSM6DSL_INT_1) {
		debugln("Unsetting gyroscope data-ready signal from INT_1");
		HAL_CHECK(
			LSM6DSL_ReadRegister(LSM6DSL_REG_INT1_CTRL, &value)
		)
		value &= ~LSM6DSL_REG_INT1_CTRL_BIT_INT1_DRDY_G;
		HAL_CHECK(
			LSM6DSL_WriteRegister(
				LSM6DSL_REG_INT1_CTRL,
				value)
		)
	}
	else if (interrupt == LSM6DSL_INT_2) {
		debugln("Unsetting gyroscope data-ready signal from INT_2");
		HAL_CHECK(
			LSM6DSL_ReadRegister(LSM6DSL_REG_INT2_CTRL, &value)
		)
		value &= ~LSM6DSL_REG_INT2_CTRL_BIT_INT2_DRDY_G;
		HAL_CHECK(
			LSM6DSL_WriteRegister(
				LSM6DSL_REG_INT2_CTRL,
				value)
		)
	}

	return status;
}

HAL_StatusTypeDef LSM6DSL_ReadStatus(uint8_t *value)
{
	debugln("Reading status...");
	return LSM6DSL_ReadRegister(LSM6DSL_REG_STATUS, value);
}

bool LSM6DSL_IsAccelerometerDataReady()
{
	uint8_t lsm6dsl_status;
	HAL_StatusTypeDef status = LSM6DSL_ReadStatus(&lsm6dsl_status);

	return status == HAL_OK && (lsm6dsl_status & LSM6DSL_REG_STATUS_BIT_XLDA);
}

bool LSM6DSL_IsGyroscopeDataReady()
{
	uint8_t lsm6dsl_status;
	HAL_StatusTypeDef status = LSM6DSL_ReadStatus(&lsm6dsl_status);

	return status == HAL_OK && (lsm6dsl_status & LSM6DSL_REG_STATUS_BIT_GDA);
}

bool LSM6DSL_Is_TemperatureDataReady()
{
	uint8_t lsm6dsl_status;
	HAL_StatusTypeDef status = LSM6DSL_ReadStatus(&lsm6dsl_status);

	return status == HAL_OK && (lsm6dsl_status & LSM6DSL_REG_STATUS_BIT_TDA);
}

HAL_StatusTypeDef LSM6DSL_ReadAccelerometer(int16_t *x, int16_t *y, int16_t *z)
{
	// 0 -> 0g

	HAL_StatusTypeDef status;
	uint16_t _x, _y, _z;

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTX_L_XL, LSM6DSL_REG_OUTX_H_XL, &_x)
	)

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTY_L_XL, LSM6DSL_REG_OUTY_H_XL, &_y)
	)

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTZ_L_XL, LSM6DSL_REG_OUTZ_H_XL, &_z)
	)

	debugln("LSM6DSL_ReadAccelerometer RAW: (x=%u, y=%u, z=%u) [scale = %dg]",
			_x, _y, _z, xl_scale);

	*x = (int16_t) _x;
	*y = (int16_t) _y;
	*z = (int16_t) _z;

	return status;
}

HAL_StatusTypeDef LSM6DSL_ReadAccelerometer_g(float *x, float *y, float *z)
{
	// Mapping: [2^-15:2^15] -> [-FS:+FS]

	HAL_StatusTypeDef status;
	int16_t _x, _y, _z;

	HAL_CHECK(LSM6DSL_ReadAccelerometer(&_x, &_y, &_z));

	*x = (float) _x * xl_scale / (1 << 15);
	*y = (float) _y * xl_scale / (1 << 15);
	*z = (float) _z * xl_scale / (1 << 15);

	return status;
}

HAL_StatusTypeDef LSM6DSL_ReadGyroscope(int16_t *x, int16_t *y, int16_t *z)
{
	// 0 -> 0dps

	HAL_StatusTypeDef status;
	uint16_t _x, _y, _z;

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTX_L_G, LSM6DSL_REG_OUTX_H_G, &_x)
	)

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTY_L_G, LSM6DSL_REG_OUTY_H_G, &_y)
	)

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTZ_L_G, LSM6DSL_REG_OUTZ_H_G, &_z)
	)

	debugln("LSM6DSL_ReadGyroscope RAW: (x=%u, y=%u, z=%u) [scale = %ddps]",
			_x, _y, _z, g_scale);

	*x = (int16_t) _x;
	*y = (int16_t) _y;
	*z = (int16_t) _z;

	return status;
}

HAL_StatusTypeDef LSM6DSL_ReadGyroscope_dps(float *x, float *y, float *z)
{
	// Mapping: [2^-15:2^15] -> [-FS:+FS]

	HAL_StatusTypeDef status;
	int16_t _x, _y, _z;

	HAL_CHECK(LSM6DSL_ReadGyroscope(&_x, &_y, &_z));

	*x = (float) _x * g_scale / (1 << 15);
	*y = (float) _y * g_scale / (1 << 15);
	*z = (float) _z * g_scale / (1 << 15);

	return status;
}

HAL_StatusTypeDef LSM6DSL_ReadTemperature_C(float *temperature)
{
	// Mapping: [0:255] -> [0 C°:1 C°]
	// 0 -> 25C°

	HAL_StatusTypeDef status;
	uint16_t value;

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUT_TEMP_L, LSM6DSL_REG_OUT_TEMP_H, &value)
	)

	debugln("LSM6DSL_Read_Temperature RAW: %u", value);

	*temperature = 25.0f + ((int16_t) value / 256.0f);

	return status;
}

