#include "lsm6dsl.h"

#include "verbose.h"

#define VERBOSE_FMT(fmt) "{LSM6DSL} " fmt

// TODO: not hardcoded?
#define LSM6DSL_RW_MAX_DELAY 100


#define HAL_CHECK(op) \
if ((status = op) != HAL_OK) {\
	return status;\
}

#define HAL_CHECK_MSG(op, msg, ...) \
if ((status = op) != HAL_OK) {\
	verboseln(msg, ##__VA_ARGS__);\
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

	verboseln("Initializing...");
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

	verboseln("Initialized");

	return status;
}


HAL_StatusTypeDef LSM6DSL_ReadRegister(uint8_t reg, uint8_t *out)
{
	HAL_StatusTypeDef status;

	verboseln("r[0x%02X] ...", reg);

	HAL_CHECK_MSG(
		HAL_I2C_Master_Transmit(
			i2c, LSM6DSL_I2C_ADDR, &reg, 1, LSM6DSL_RW_MAX_DELAY),
		"Failed to read (HAL_I2C_Master_Transmit failed with error %d)",
		status
	)

	HAL_CHECK_MSG(
		HAL_I2C_Master_Receive(
			i2c, LSM6DSL_I2C_ADDR, out, 1, LSM6DSL_RW_MAX_DELAY),
		"Failed to read (HAL_I2C_Master_Receive failed with error %d)",
		status
	)

	verboseln("r[0x%02X] = 0x%02X", reg, *out);

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

	verboseln("w[0x%02X] <- 0x%02X ...", reg, value);

	HAL_CHECK_MSG(
		HAL_I2C_Master_Transmit(
			i2c, LSM6DSL_I2C_ADDR, sendbuf, 2, LSM6DSL_RW_MAX_DELAY),
		"Failed to write (HAL_I2C_Master_Transmit failed with error %d)",
		status
	)

	verboseln("w[0x%02X] = 0x%02X", reg, value);

	return status;
}

bool LSM6DSL_IsHealthy()
{
	uint8_t value;

	if (LSM6DSL_ReadRegister(LSM6DSL_REG_WHO_AM_AI, &value) != HAL_OK ||
		value != 0x6A) {
		verboseln("LSM6DSL_AssertHealthy failed: WHO_AM_I != 0x6A");
		return false;
	}

	return true;
}

HAL_StatusTypeDef LSM6DSL_EnableAccelerometer(
		LSM6DSL_Frequency frequency,
		LSM6DSL_AccelerometerScale scale)
{
	HAL_StatusTypeDef status;

	verboseln("Enabling accelerometer...");

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

	verboseln("Accelerometer enabled");

	return status;
}

HAL_StatusTypeDef LSM6DSL_EnableGyroscope(
		LSM6DSL_Frequency frequency,
		LSM6DSL_GyroscopeScale scale)
{
	HAL_StatusTypeDef status;
	verboseln("Enabling gyroscope...");

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

	verboseln("Gyroscope enabled");

	return status;
}

HAL_StatusTypeDef LSM6DSL_DisableAccelerometer()
{
	HAL_StatusTypeDef status;

	verboseln("Disabling accelerometer...");

	HAL_CHECK(
		LSM6DSL_WriteRegister(
			LSM6DSL_REG_CTRL1_XL,
			LSM6DSL_POWER_DOWN)
	)

	verboseln("Accelerometer disabled");

	return status;

}

HAL_StatusTypeDef LSM6DSL_DisableGyroscope()
{
	HAL_StatusTypeDef status;

	verboseln("Disabling gyroscope...");

	HAL_CHECK(
		LSM6DSL_WriteRegister(
			LSM6DSL_REG_CTRL2_G,
			LSM6DSL_POWER_DOWN)
	)

	verboseln("Gyroscope disabled");

	return status;
}


HAL_StatusTypeDef LSM6DSL_SetAccelerometerInterrupt(LSM6DSL_InterruptPin interrupt)
{
	HAL_StatusTypeDef status;
	uint8_t value;

	if (interrupt == LSM6DSL_INT_1) {
		verboseln("Setting accelerometer data-ready signal to INT_1");
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
		verboseln("Setting accelerometer data-ready signal to INT_2");
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
		verboseln("Setting gyroscope data-ready signal to INT_1");
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
		verboseln("Setting gyroscope data-ready signal to INT_2");
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
		verboseln("Unsetting accelerometer data-ready signal from INT_1");
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
		verboseln("Unsetting accelerometer data-ready signal from INT_2");
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
		verboseln("Unsetting gyroscope data-ready signal from INT_1");
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
		verboseln("Unsetting gyroscope data-ready signal from INT_2");
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
	verboseln("Reading status...");
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

HAL_StatusTypeDef LSM6DSL_ReadAccelerometer(dim3_i16 *xl)
{
	// 0 -> 0g

	HAL_StatusTypeDef status;
	uint16_t x, y, z;

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTX_L_XL, LSM6DSL_REG_OUTX_H_XL, &x)
	)

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTY_L_XL, LSM6DSL_REG_OUTY_H_XL, &y)
	)

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTZ_L_XL, LSM6DSL_REG_OUTZ_H_XL, &z)
	)

	verboseln("LSM6DSL_ReadAccelerometer RAW: (x=%u, y=%u, z=%u) [scale = %dg]",
			x, y, z, xl_scale);

	xl->x = (int16_t) x;
	xl->y = (int16_t) y;
	xl->z = (int16_t) z;

	return status;
}

HAL_StatusTypeDef LSM6DSL_ReadAccelerometer_g(dim3_f *xl)
{
	// Mapping: [2^-15:2^15] -> [-FS:+FS]

	HAL_StatusTypeDef status;
	dim3_i16 _xl;

	HAL_CHECK(LSM6DSL_ReadAccelerometer(&_xl))

	xl->x = (float) _xl.x * xl_scale / (1 << 15);
	xl->y = (float) _xl.y * xl_scale / (1 << 15);
	xl->z = (float) _xl.z * xl_scale / (1 << 15);

	return status;
}

HAL_StatusTypeDef LSM6DSL_ReadGyroscope(dim3_i16 *g)
{
	// 0 -> 0dps

	HAL_StatusTypeDef status;
	uint16_t x, y, z;

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTX_L_G, LSM6DSL_REG_OUTX_H_G, &x)
	)

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTY_L_G, LSM6DSL_REG_OUTY_H_G, &y)
	)

	HAL_CHECK(
		LSM6DSL_ReadRegisters_2(
			LSM6DSL_REG_OUTZ_L_G, LSM6DSL_REG_OUTZ_H_G, &z)
	)

	verboseln("LSM6DSL_ReadGyroscope RAW: (x=%u, y=%u, z=%u) [scale = %ddps]",
			x, y, z, g_scale);

	g->x = (int16_t) x;
	g->y = (int16_t) y;
	g->z = (int16_t) z;

	return status;
}

HAL_StatusTypeDef LSM6DSL_ReadGyroscope_dps(dim3_f *g)
{
	// Mapping: [2^-15:2^15] -> [-FS:+FS]

	HAL_StatusTypeDef status;
	dim3_i16 _g;

	HAL_CHECK(LSM6DSL_ReadGyroscope(&_g))

	g->x = (float) _g.x * g_scale / (1 << 15);
	g->y = (float) _g.y * g_scale / (1 << 15);
	g->z = (float) _g.z * g_scale / (1 << 15);

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

	verboseln("LSM6DSL_Read_Temperature RAW: %u", value);

	*temperature = 25.0f + ((int16_t) value / 256.0f);

	return status;
}

float LSM6DSL_Frequency_ToSampleRate(LSM6DSL_Frequency freq)
{
	switch (freq) {
	case LSM6DSL_12_HZ:
		return 12.5;
	case LSM6DSL_26_HZ:
		return 26;
	case LSM6DSL_52_HZ:
		return 52;
	case LSM6DSL_104_HZ:
		return 104;
	case LSM6DSL_208_HZ:
		return 208;
	case LSM6DSL_416_HZ:
		return 416;
	case LSM6DSL_833_HZ:
		return 833;
	case LSM6DSL_1666_HZ:
		return 1666;
	case LSM6DSL_3333_HZ:
		return 3333;
	case LSM6DSL_6666_HZ:
		return 6666;
	default:
		return 0;
	}
}

