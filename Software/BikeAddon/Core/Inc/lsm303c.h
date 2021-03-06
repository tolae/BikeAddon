#ifndef _LSM303C_H_
#define _LSM303C_H_

#include "main.h"
#include "xyz_axis.h"

/* LSM303C Accelerometer Register Addresses */
#define LSM303C_WHO_AM_I_A_ADDR			0x0F
#define LSM303C_ACT_THS_A_ADDR			0x1E
#define LSM303C_DUR_A_ADDR				0x1F
#define LSM303C_CTRL_REG1_A_ADDR		0x20
#define LSM303C_CTRL_REG2_A_ADDR		0x21
#define LSM303C_CTRL_REG3_A_ADDR		0x22
#define LSM303C_CTRL_REG4_A_ADDR		0x23
#define LSM303C_CTRL_REG5_A_ADDR		0x24
#define LSM303C_CTRL_REG6_A_ADDR		0x25
#define LSM303C_CTRL_REG7_A_ADDR		0x26
#define LSM303C_STATUS_REG_A_ADDR		0x27
#define LSM303C_OUT_X_L_A_ADDR			0x28
#define LSM303C_OUT_X_H_A_ADDR			0x29
#define LSM303C_OUT_Y_L_A_ADDR			0x2A
#define LSM303C_OUT_Y_H_A_ADDR			0x2B
#define LSM303C_OUT_Z_L_A_ADDR			0x2C
#define LSM303C_OUT_Z_H_A_ADDR			0x2D
#define LSM303C_FIFO_CTRL_A_ADDR		0x2E
#define LSM303C_FIFO_SRC_A_ADDR			0x2F
#define LSM303C_IG_CFG1_A_ADDR			0x30
#define LSM303C_IG_SRC1_A_ADDR			0x31
#define LSM303C_IG_THS_X1_A_ADDR		0x32
#define LSM303C_IG_THS_Y1_A_ADDR		0x33
#define LSM303C_IG_THS_Z1_A_ADDR		0x34
#define LSM303C_IG_DUR1_A_ADDR			0x35
#define LSM303C_IG_CFG2_A_ADDR			0x36
#define LSM303C_SRC2_A_ADDR				0x37
#define LSM303C_THS2_A_ADDR				0x38
#define LSM303C_IG_DUR2_A_ADDR			0x39
#define LSM303C_XL_REFERENCE_A_ADDR		0x3A
#define LSM303C_XH_REFERENCE_A_ADDR		0x3B
#define LSM303C_YL_REFERENCE_A_ADDR		0x3C
#define LSM303C_YH_REFERENCE_A_ADDR		0x3D
#define LSM303C_ZL_REFERENCE_A_ADDR		0x3E
#define LSM303C_ZH_REFERENCE_A_ADDR		0x3F

/* Defaults */
#define LSM303C_WHO_AM_I_A_DEFAULT		0x41
#define LSM303C_ACT_THS_A_DEFAULT		0x00
#define LSM303C_DUR_A_DEFAULT			0x00
#define LSM303C_CTRL_REG1_A_DEFAULT		0x07
#define LSM303C_CTRL_REG2_A_DEFAULT		0x00
#define LSM303C_CTRL_REG3_A_DEFAULT		0x00
#define LSM303C_CTRL_REG4_A_DEFAULT		0x04
#define LSM303C_CTRL_REG5_A_DEFAULT		0x00
#define LSM303C_CTRL_REG6_A_DEFAULT		0x00
#define LSM303C_CTRL_REG7_A_DEFAULT		0x00
#define LSM303C_FIFO_CTRL_A_DEFAULT		0x00
#define LSM303C_IG_CFG1_A_DEFAULT		0x00
#define LSM303C_IG_THS_X1_A_DEFAULT		0x00
#define LSM303C_IG_THS_Y1_A_DEFAULT		0x00
#define LSM303C_IG_THS_Z1_A_DEFAULT		0x00
#define LSM303C_IG_DUR1_A_DEFAULT		0x00
#define LSM303C_IG_CFG2_A_DEFAULT		0x00
#define LSM303C_THS2_A_DEFAULT			0x00
#define LSM303C_IG_DUR2_A_DEFAULT		0x00
#define LSM303C_XL_REFERENCE_A_DEFAULT	0x00
#define LSM303C_XH_REFERENCE_A_DEFAULT	0x00
#define LSM303C_YL_REFERENCE_A_DEFAULT	0x00
#define LSM303C_YH_REFERENCE_A_DEFAULT	0x00
#define LSM303C_ZL_REFERENCE_A_DEFAULT	0x00
#define LSM303C_ZH_REFERENCE_A_DEFAULT	0x00

/* Sensitivity (g/LSB) */
#define LSM303C_ACCEL_2G_SENSITIVITY	0.000061
#define LSM303C_ACCEL_4G_SENSITIVITY	0.000122
#define LSM303C_ACCEL_8G_SENSITIVITY	0.000244

/* LSM303C Magnometer Register Addresses */
#define LSM303_WHO_AM_I_M_ADDR			0x0F

typedef struct
{
	xyz_axis_t xyz_raw;

	double acc_x;
	double acc_y;

	double acc_x_error;
	double acc_y_error;
} lsm303c_t;

void lsm303c_init(SPI_HandleTypeDef* spix);
void lsm303c_write(uint8_t* in_buffer, uint8_t addr, uint8_t size);
void lsm303c_read(uint8_t* out_buffer, uint8_t addr, uint8_t size);

void lsm303c_compute_angles(lsm303c_t* data);

IMPLEMENT_READ_XYZ_VALUES(lsm303c);

#endif
