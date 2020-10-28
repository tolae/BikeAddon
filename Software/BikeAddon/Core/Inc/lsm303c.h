#ifndef _LSM303C_H_
#define _LSM303C_H_

#include "main.h"

/* LSM303C Accelerometer Register Addresses */
#define LSM303C_WHO_AM_I_A_ADRR			0x0F
#define LSM303C_ACT_THS_A_ADDR			0x1E
#define LSM303C_DUR_A_ADDR				0x1F
#define LSM303C_CTRL_REG4_A_ADDR		0x23

/* Defaults */
#define LSM303C_WHO_AM_I_A_DEFAULT		0x41

/* LSM303C Magnometer Register Addresses */
#define LSM303_WHO_AM_I_M_ADDR			0x0F

void lsm303c_init(SPI_HandleTypeDef* spix);
void lsm303c_write(SPI_HandleTypeDef* spix, uint8_t* in_buffer, uint8_t addr, uint8_t size);
void lsm303c_read(SPI_HandleTypeDef* spix, uint8_t* out_buffer, uint8_t addr, uint8_t size);

#endif
