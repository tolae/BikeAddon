/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2019
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */


#include <math.h>
#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

void _MPU6050_Handle_Accel_Raw(MPU6050_t *DataStruct, uint8_t *Rec_Data);
void _MPU6050_Convert_Accel(MPU6050_t *DataStruct);
void _MPU6050_Handle_Gyro_Raw(MPU6050_t *DataStruct, uint8_t *Rec_Data);
void _MPU6050_Convert_Gyro(MPU6050_t *DataStruct);
void _MPU6050_Handle_Temp_Raw(MPU6050_t *DataStruct, uint8_t *Rec_Data);
void _MPU6050_Convert_Temp(MPU6050_t *DataStruct);

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 0x68)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);
    _MPU6050_Handle_Accel_Raw(DataStruct, Rec_Data);
    _MPU6050_Convert_Accel(DataStruct);
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);
    _MPU6050_Handle_Gyro_Raw(DataStruct, Rec_Data);
    _MPU6050_Convert_Gyro(DataStruct);
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[2];
    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);
    _MPU6050_Handle_Temp_Raw(DataStruct, Rec_Data);
    _MPU6050_Convert_Temp(DataStruct);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[14];
    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    _MPU6050_Handle_Accel_Raw(DataStruct, Rec_Data);
    _MPU6050_Handle_Temp_Raw(DataStruct, &(Rec_Data[6]));
    _MPU6050_Handle_Gyro_Raw(DataStruct, &(Rec_Data[8]));

    _MPU6050_Convert_Accel(DataStruct);
    _MPU6050_Convert_Temp(DataStruct);
	_MPU6050_Convert_Gyro(DataStruct);
}

void MPU6050_Compute_Angles(MPU6050_t *DataStruct, double dt)
{
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
    	DataStruct->AccX = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG + DataStruct->AccXError;
    } else {
    	DataStruct->AccX = 0.0;
    }
    DataStruct->AccY = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG + DataStruct->AccYError;

	DataStruct->GyroX += DataStruct->Gx * dt;
	DataStruct->GyroY += DataStruct->Gy * dt;
	DataStruct->GyroZ += DataStruct->Gz * dt;
}

void _MPU6050_Handle_Accel_Raw(MPU6050_t *DataStruct, uint8_t *Rec_Data)
{
	DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
}

void _MPU6050_Convert_Accel(MPU6050_t *DataStruct)
{
	DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
	DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void _MPU6050_Handle_Gyro_Raw(MPU6050_t *DataStruct, uint8_t *Rec_Data)
{
	DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
}

void _MPU6050_Convert_Gyro(MPU6050_t *DataStruct)
{
	DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0 + DataStruct->GyroXError;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0 + DataStruct->GyroYError;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0 + DataStruct->GyroZError;
}

void _MPU6050_Handle_Temp_Raw(MPU6050_t *DataStruct, uint8_t *Rec_Data)
{
    DataStruct->Temperature_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
}

void _MPU6050_Convert_Temp(MPU6050_t *DataStruct)
{
    DataStruct->Temperature = (float) (DataStruct->Temperature_RAW / (float) 340.0 + (float) 36.53);
}
