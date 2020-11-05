/*
 * mpu6050.c
 *
 *  Created on: Nov 5, 2020
 *      Author: Ethan
 */

#include "mpu6050.h"

typedef struct
{
	union
	{
		uint16_t addr;
		struct
		{
			/** This is an artifical bit shift as the
			 * first bit is reserved in the I2C HAL.
			 */
			uint8_t _res1: 1;
			uint8_t _addr: 7;
			uint8_t rw: 1;
			uint8_t _res2: 7;
		};
	};
} mpu6050_i2caddr_t;

typedef enum
{
	MPU6050_I2C_WRITE = 0,
	MPU6050_I2C_READ = 1
} MPU_6050_RW;

I2C_HandleTypeDef* mpu6050_i2c;

void mpu6050_prep_dev_addr(mpu6050_i2caddr_t* out_addr, uint8_t dev_addr, uint32_t rw);

void mpu6050_init(I2C_HandleTypeDef* hi2c1)
{
	uint8_t config;
	HAL_StatusTypeDef test;
	mpu6050_i2c = hi2c1;

	test = HAL_I2C_IsDeviceReady(mpu6050_i2c, 0xD0, 300, 0xFFFF);


	mpu6050_read(&config, MPU6050_DEV_ADDR_1, MPU6050_WHO_AM_I_ADDR, 1);
}

void mpu6050_read(uint8_t* out_buffer, uint16_t dev_addr, uint8_t data_addr, uint8_t size)
{
	uint16_t prepped_dev_addr;
	mpu6050_prep_dev_addr((mpu6050_i2caddr_t*)(&prepped_dev_addr), dev_addr, MPU6050_I2C_WRITE);
	HAL_I2C_Master_Transmit(mpu6050_i2c, prepped_dev_addr, &data_addr, 1, 0xFFFF);

	mpu6050_prep_dev_addr((mpu6050_i2caddr_t*)(&prepped_dev_addr), dev_addr, MPU6050_I2C_READ);
	HAL_I2C_Master_Receive(mpu6050_i2c, prepped_dev_addr, out_buffer, size, 0xFFFF);
}

void mpu6050_prep_dev_addr(mpu6050_i2caddr_t* out_addr, uint8_t dev_addr, uint32_t rw)
{
	/* I2C uses 7 bit addressing */
	out_addr->_addr = dev_addr & 0x7F;
	out_addr->rw = rw;
}
