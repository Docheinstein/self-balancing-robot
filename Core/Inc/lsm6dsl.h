#ifndef LSM6DSL_H
#define LSM6DSL_H

#include "stm32l4xx_hal.h"

// ============== I2C ==================

#define LSM6DSL_I2C_ADDR 		(0x6A << 1)
#define LSM6DSL_I2C_WRITE_ADDR 	0xD4
#define LSM6DSL_I2C_READ_ADDR  	0xD5


// =========== REGISTERS ===============

// Control/Interrupt/Misc

#define LSM6DSL_REG_CTRL1_XL 	0x10
#define LSM6DSL_REG_CTRL2_G 	0x11
#define LSM6DSL_REG_CTRL3_C 	0x12
#define LSM6DSL_REG_CTRL4_C 	0x13
#define LSM6DSL_REG_CTRL5_C 	0x14
#define LSM6DSL_REG_CTRL6_C 	0x15
#define LSM6DSL_REG_CTRL7_G 	0x16
#define LSM6DSL_REG_CTRL8_XL 	0x17
#define LSM6DSL_REG_CTRL9_XL 	0x18
#define LSM6DSL_REG_CTRL10_C 	0x19

#define LSM6DSL_REG_INT1_CTRL 	0x0D
#define LSM6DSL_REG_INT2_CTRL 	0x0E

#define LSM6DSL_REG_STATUS		0x1E
#define LSM6DSL_REG_WHO_AM_AI	0x0F

#define TIMESTAMP0_REG			0x40
#define TIMESTAMP1_REG			0x41
#define TIMESTAMP2_REG			0x42

// Temperature

#define LSM6DSL_REG_OUT_TEMP_L 	0x20
#define LSM6DSL_REG_OUT_TEMP_H 	0x21

// Gyroscope

#define LSM6DSL_REG_OUTX_L_G 	0x22
#define LSM6DSL_REG_OUTX_H_G 	0x23
#define LSM6DSL_REG_OUTY_L_G 	0x24
#define LSM6DSL_REG_OUTY_H_G 	0x25
#define LSM6DSL_REG_OUTZ_L_G 	0x26
#define LSM6DSL_REG_OUTZ_H_G 	0x27

// Accelerometer

#define LSM6DSL_REG_OUTX_L_XL 	0x28
#define LSM6DSL_REG_OUTX_H_XL 	0x29
#define LSM6DSL_REG_OUTY_L_XL 	0x2A
#define LSM6DSL_REG_OUTY_H_XL 	0x2B
#define LSM6DSL_REG_OUTZ_L_XL 	0x2C
#define LSM6DSL_REG_OUTZ_H_XL 	0x2D


// =========== FUNCTIONS ===============

HAL_StatusTypeDef LSM6DSL_Read_Register(uint8_t reg, uint8_t *out);
HAL_StatusTypeDef LSM6DSL_Read_Registers_2(uint8_t reg_h, uint8_t reg_l, uint16_t *out);
HAL_StatusTypeDef LSM6DSL_Write_Register(uint8_t reg, uint8_t value);

void LSM6DSL_Assert_Healthy();


#endif /* LSM6DSL_H */
