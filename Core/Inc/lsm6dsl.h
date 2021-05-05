#ifndef LSM6DSL_H
#define LSM6DSL_H

#include <bits.h>
#include "stdbool.h"
#include "stm32l4xx_hal.h"
#include "dim3.h"

// ============== I2C ==================

#define LSM6DSL_I2C_ADDR 			(0x6A << 1)
#define LSM6DSL_I2C_WRITE_ADDR 		0xD4
#define LSM6DSL_I2C_READ_ADDR 	 	0xD5

// =========== REGISTERS ===============

// Control

#define LSM6DSL_REG_CTRL1_XL 		0x10
#define LSM6DSL_REG_CTRL2_G 		0x11
#define LSM6DSL_REG_CTRL3_C 		0x12
#define LSM6DSL_REG_CTRL4_C 		0x13
#define LSM6DSL_REG_CTRL5_C 		0x14
#define LSM6DSL_REG_CTRL6_C 		0x15
#define LSM6DSL_REG_CTRL7_G 		0x16
#define LSM6DSL_REG_CTRL8_XL 		0x17
#define LSM6DSL_REG_CTRL9_XL 		0x18
#define LSM6DSL_REG_CTRL10_C 		0x19

#define LSM6DSL_REG_MASTER_CONFIG 	0x1A

// Interrupt

#define LSM6DSL_REG_INT1_CTRL 		0x0D
#define LSM6DSL_REG_INT2_CTRL 		0x0E

// Other

#define LSM6DSL_REG_STATUS			0x1E
#define LSM6DSL_REG_WHO_AM_AI		0x0F

#define TIMESTAMP0_REG				0x40
#define TIMESTAMP1_REG				0x41
#define TIMESTAMP2_REG				0x42

// Temperature

#define LSM6DSL_REG_OUT_TEMP_L 		0x20
#define LSM6DSL_REG_OUT_TEMP_H 		0x21

// Gyroscope

#define LSM6DSL_REG_OUTX_L_G 		0x22
#define LSM6DSL_REG_OUTX_H_G 		0x23
#define LSM6DSL_REG_OUTY_L_G 		0x24
#define LSM6DSL_REG_OUTY_H_G 		0x25
#define LSM6DSL_REG_OUTZ_L_G 		0x26
#define LSM6DSL_REG_OUTZ_H_G 		0x27

// Accelerometer

#define LSM6DSL_REG_OUTX_L_XL 		0x28
#define LSM6DSL_REG_OUTX_H_XL 		0x29
#define LSM6DSL_REG_OUTY_L_XL 		0x2A
#define LSM6DSL_REG_OUTY_H_XL 		0x2B
#define LSM6DSL_REG_OUTZ_L_XL 		0x2C
#define LSM6DSL_REG_OUTZ_H_XL 		0x2D

// ======== BITS OF REGISTERS =========

#define LSM6DSL_REG_MASTER_CONFIG_BIT_DRDY_ON_INT1 	BIT(7)

#define LSM6DSL_REG_STATUS_BIT_XLDA 				BIT(0)
#define LSM6DSL_REG_STATUS_BIT_GDA 					BIT(1)
#define LSM6DSL_REG_STATUS_BIT_TDA 					BIT(2)

#define LSM6DSL_REG_CTRL3_C_BIT_SW_RESET			BIT(0)
#define LSM6DSL_REG_CTRL3_C_BIT_IF_INC				BIT(2)
#define LSM6DSL_REG_CTRL3_C_BIT_H_LACTIVE			BIT(5)
#define LSM6DSL_REG_CTRL3_C_BIT_BDU					BIT(6)

#define LSM6DSL_REG_INT1_CTRL_BIT_INT1_DRDY_XL 		BIT(0)
#define LSM6DSL_REG_INT1_CTRL_BIT_INT1_DRDY_G 		BIT(1)

#define LSM6DSL_REG_INT2_CTRL_BIT_INT2_DRDY_XL 		BIT(0)
#define LSM6DSL_REG_INT2_CTRL_BIT_INT2_DRDY_G 		BIT(1)

// ============ DATA TYPES ============

typedef enum {
	LSM6DSL_POWER_DOWN  = 0x00,	// Power-Down mode
	LSM6DSL_12_HZ   	= 0x10,	// Low-Power mode
	LSM6DSL_26_HZ    	= 0x20,
	LSM6DSL_52_HZ    	= 0x30,
	LSM6DSL_104_HZ    	= 0x40,	// Normal mode
	LSM6DSL_208_HZ  	= 0x50,
	LSM6DSL_416_HZ  	= 0x60,	// High-Performance mode
	LSM6DSL_833_HZ  	= 0x70,
	LSM6DSL_1666_HZ  	= 0x80,
	LSM6DSL_3333_HZ  	= 0x90,
	LSM6DSL_6666_HZ  	= 0xA0,
} LSM6DSL_Frequency;

typedef enum {
	LSM6DSL_FS_XL_2_G 	= 0x00,
	LSM6DSL_FS_XL_16_G 	= 0x04,
	LSM6DSL_FS_XL_4_G 	= 0x08,
	LSM6DSL_FS_XL_8_G 	= 0x0C
} LSM6DSL_AccelerometerScale;

typedef enum {
	LSM6DSL_FS_G_250_DPS 	= 0x00,
	LSM6DSL_FS_G_500_DPS 	= 0x04,
	LSM6DSL_FS_G_1000_DPS 	= 0x08,
	LSM6DSL_FS_G_2000_DPS 	= 0x0C
} LSM6DSL_GyroscopeScale;

typedef enum {
	LSM6DSL_INT_1,
	LSM6DSL_INT_2
} LSM6DSL_InterruptPin;

// =========== FUNCTIONS ===============

HAL_StatusTypeDef LSM6DSL_Init(I2C_HandleTypeDef *i2c);

bool LSM6DSL_IsHealthy();

HAL_StatusTypeDef LSM6DSL_ReadRegister(uint8_t reg, uint8_t *out);
HAL_StatusTypeDef LSM6DSL_ReadRegisters_2(uint8_t reg_h, uint8_t reg_l, uint16_t *out);
HAL_StatusTypeDef LSM6DSL_WriteRegister(uint8_t reg, uint8_t value);


HAL_StatusTypeDef LSM6DSL_EnableAccelerometer(
	LSM6DSL_Frequency frequency,
	LSM6DSL_AccelerometerScale scale
);
HAL_StatusTypeDef LSM6DSL_EnableGyroscope(
	LSM6DSL_Frequency frequency,
	LSM6DSL_GyroscopeScale scale
);

HAL_StatusTypeDef LSM6DSL_DisableAccelerometer();
HAL_StatusTypeDef LSM6DSL_DisableGyroscope();

HAL_StatusTypeDef LSM6DSL_SetAccelerometerInterrupt(LSM6DSL_InterruptPin interrupt);
HAL_StatusTypeDef LSM6DSL_SetGyroscopeInterrupt(LSM6DSL_InterruptPin interrupt);

HAL_StatusTypeDef LSM6DSL_UnsetAccelerometerInterrupt(LSM6DSL_InterruptPin interrupt);
HAL_StatusTypeDef LSM6DSL_UnsetGyroscopeInterrupt(LSM6DSL_InterruptPin interrupt);

HAL_StatusTypeDef LSM6DSL_ReadStatus(uint8_t *value);

bool LSM6DSL_IsAccelerometerDataReady();
bool LSM6DSL_IsGyroscopeDataReady();
bool LSM6DSL_IsTemperatureDataReady();

HAL_StatusTypeDef LSM6DSL_ReadAccelerometer(dim3_i16 *xl);
HAL_StatusTypeDef LSM6DSL_ReadAccelerometer_g(dim3_f *xl); // g
HAL_StatusTypeDef LSM6DSL_ReadGyroscope(dim3_i16 *g);
HAL_StatusTypeDef LSM6DSL_ReadGyroscope_dps(dim3_f *g); // dps: degrees per second
HAL_StatusTypeDef LSM6DSL_ReadTemperature_C(float *temperature); // C°

int LSM6DSL_Frequency_ToInt(LSM6DSL_Frequency freq); // Hz
LSM6DSL_Frequency LSM6DSL_Frequency_FromInt(int freq);

#endif /* LSM6DSL_H */
