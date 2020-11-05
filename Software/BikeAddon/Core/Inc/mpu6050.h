/*
 * mpu6050.h
 *
 *  Created on: Nov 5, 2020
 *      Author: Ethan
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include "xyz_axis.h"

#define MPU6050_DEV_ADDR_1		0x68
#define MPU6050_DEV_ADDR_2		0x69

#define MPU6050_WHO_AM_I_ADDR	0x75

void mpu6050_init(I2C_HandleTypeDef* hi2c1);
void mpu6050_write(uint8_t* in_buffer, uint16_t dev_addr, uint8_t data_addr, uint8_t size);
void mpu6050_read(uint8_t* out_buffer, uint16_t dev_addr, uint8_t data_addr, uint8_t size);

IMPLEMENT_READ_XYZ_VALUES(mpu6050);

#endif /* INC_MPU6050_H_ */
