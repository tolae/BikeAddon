/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#include <stdint.h>
#include "i2c.h"

/* Registers */
#define SELF_TEST_X_REG			0x0D
#define SELF_TEST_Y_REG			0x0E
#define SELF_TEST_Z_REG			0x0F
#define SELF_TEST_A_REG			0x10
#define SMPLRT_DIV_REG_REG		0x19
#define CONFIG_REG				0x1A
#define GYRO_CONFIG_REG			0x1B
#define ACCEL_CONFIG_REG		0x1C
#define MOT_THR_REG				0x1F
#define FIFO_EN_REG				0x23
#define I2C_MST_CTRL_REG		0x24
#define I2C_SLV0_ADDR_REG		0x25
#define I2C_SLV0_REG_REG		0x26
#define I2C_SLV0_CTRL_REG		0x27
#define I2C_SLV1_ADDR_REG		0x28
#define I2C_SLV1_REG_REG		0x29
#define I2C_SLV1_CTRL_REG		0x2A
#define I2C_SLV2_ADDR_REG		0x2B
#define I2C_SLV2_REG_REG		0x2C
#define I2C_SLV2_CTRL_REG		0x2D
#define I2C_SLV3_ADDR_REG		0x2E
#define I2C_SLV3_REG_REG		0x2F
#define I2C_SLV3_CTRL_REG		0x30
#define I2C_SLV4_ADDR_REG		0x31
#define I2C_SLV4_REG_REG		0x32
#define I2C_SLV4_DO_REG			0x33
#define I2C_SLV4_CTRL_REG		0x34
#define I2C_SLV4_DI_REG			0x35
#define I2C_MST_STATUS_REG		0x36
#define INT_PIN_CFG_REG			0x37
#define INT_ENABLE_REG			0x38
#define INT_STATUS_REG			0x3A
#define ACCEL_XOUT_H_REG		0x3B
#define ACCEL_XOUT_L_REG		0x3C
#define ACCEL_YOUT_H_REG		0x3D
#define ACCEL_YOUT_L_REG		0x3E
#define ACCEL_ZOUT_H_REG		0x3F
#define ACCEL_ZOUT_L_REG		0x40
#define TEMP_OUT_H_REG			0x41
#define TEMP_OUT_L_REG			0x42
#define GYRO_XOUT_H_REG			0x43
#define GYRO_XOUT_L_REG			0x44
#define GYRO_YOUT_H_REG			0x45
#define GYRO_YOUT_L_REG			0x46
#define GYRO_ZOUT_H_REG			0x47
#define GYRO_ZOUT_L_REG			0x48
#define EXT_SENS_DATA_00_REG	0x49
#define EXT_SENS_DATA_01_REG	0x4A
#define EXT_SENS_DATA_02_REG	0x4B
#define EXT_SENS_DATA_03_REG	0x4C
#define EXT_SENS_DATA_04_REG	0x4D
#define EXT_SENS_DATA_05_REG	0x4E
#define EXT_SENS_DATA_06_REG	0x4F
#define EXT_SENS_DATA_07_REG	0x50
#define EXT_SENS_DATA_08_REG	0x51
#define EXT_SENS_DATA_09_REG	0x52
#define EXT_SENS_DATA_10_REG	0x53
#define EXT_SENS_DATA_11_REG	0x54
#define EXT_SENS_DATA_12_REG	0x55
#define EXT_SENS_DATA_13_REG	0x56
#define EXT_SENS_DATA_14_REG	0x57
#define EXT_SENS_DATA_15_REG	0x58
#define EXT_SENS_DATA_16_REG	0x59
#define EXT_SENS_DATA_17_REG	0x5A
#define EXT_SENS_DATA_18_REG	0x5B
#define EXT_SENS_DATA_19_REG	0x5C
#define EXT_SENS_DATA_20_REG	0x5D
#define EXT_SENS_DATA_21_REG	0x5E
#define EXT_SENS_DATA_22_REG	0x5F
#define EXT_SENS_DATA_23_REG	0x60
#define I2C_SLV0_DO_REG			0x63
#define I2C_SVL1_DO_REG			0x64
#define I2C_SVL2_DO_REG			0x65
#define I2C_SVL3_DO_REG			0x66
#define I2C_MST_DELAY_CTRL_REG	0x67
#define SIGNAL_PATH_RESET_REG	0x68
#define MOT_DETECT_CTRL_REG		0x69
#define USER_CTRL_REG			0x6A
#define PWR_MGMT_1_REG			0x6B
#define PWR_MGMT_2_REG			0x6C
#define FIFO_COUNT_H_REG		0x72
#define FIFO_COUNT_L_REG		0x73
#define FIFO_R_W_REG			0x74
#define WHO_AM_I_REG            0x75

// MPU6050 structure
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    int16_t Temperature_RAW;
    float Temperature;

    double AccXError;
	double AccYError;
    double AccX;
    double AccY;

    double GyroXError;
	double GyroYError;
	double GyroZError;
    double GyroX;
    double GyroY;
    double GyroZ;
} MPU6050_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Compute_Angles(MPU6050_t *DataStruct, double dt);

#endif /* INC_GY521_H_ */
