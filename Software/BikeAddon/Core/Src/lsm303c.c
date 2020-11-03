#include "lsm303c.h"

typedef struct
{
	union
	{
		uint8_t addr;
		struct
		{
			uint8_t _addr : 7;
			uint8_t rw : 1;
		};
	};
} lsm303c_spiaddr_t;

typedef enum
{
	LSM303C_SPI_WRITE = 0,
	LSM303C_SPI_READ = 1
} LSM303C_RW;

void lsm303c_prep_addr(lsm303c_spiaddr_t* inout_addr, uint32_t rw);

SPI_HandleTypeDef* lsm303c_spi;

void lsm303c_init(SPI_HandleTypeDef* spix)
{
	lsm303c_spi = spix;

	uint8_t config;
	uint8_t whoami;
	/* The following weirdness is required... */
	lsm303c_read(&config, LSM303C_CTRL_REG4_A_ADDR, 1);
	/* Enable R/W mode for accelerometer */
	/* Enable full scale to 8g */
	/* Disable I2C comms (using SPI) */
	config = LSM303C_CTRL_REG4_A_DEFAULT | 0x33;
	lsm303c_write(&config, LSM303C_CTRL_REG4_A_ADDR, 1);
	/* Set ODR to 100 Hz Low Pass Filter */
	config = LSM303C_CTRL_REG1_A_DEFAULT | 0x30;
	lsm303c_write(&config, LSM303C_CTRL_REG1_A_ADDR, 1);

	/* Dummy check to ensure the system is working */
	do
	{
		lsm303c_read(&whoami, LSM303C_WHO_AM_I_A_ADDR, 1);
	} while (whoami != LSM303C_WHO_AM_I_A_DEFAULT);
}

void lsm303c_write(uint8_t* in_buffer, uint8_t addr, uint8_t size)
{
	lsm303c_prep_addr((lsm303c_spiaddr_t*)(&addr), LSM303C_SPI_WRITE);

	HAL_GPIO_WritePin(LSM303C_XL_CS_GPIO_Port, LSM303C_XL_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lsm303c_spi, &addr, size, 0xFFFF);
	HAL_SPI_Transmit(lsm303c_spi, in_buffer, size, 0xFFFF);
	HAL_GPIO_WritePin(LSM303C_XL_CS_GPIO_Port, LSM303C_XL_CS_Pin, GPIO_PIN_SET);
}

void lsm303c_read(uint8_t* out_buffer, uint8_t addr, uint8_t size)
{
	lsm303c_prep_addr((lsm303c_spiaddr_t*)(&addr), LSM303C_SPI_READ);

	HAL_GPIO_WritePin(LSM303C_XL_CS_GPIO_Port, LSM303C_XL_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lsm303c_spi, &addr, size, 0xFFFF);
	HAL_SPI_Receive(lsm303c_spi, out_buffer, size, 0xFFFF);
	HAL_GPIO_WritePin(LSM303C_XL_CS_GPIO_Port, LSM303C_XL_CS_Pin, GPIO_PIN_SET);
}

void lsm303c_prep_addr(lsm303c_spiaddr_t* inout_addr, uint32_t rw)
{
	inout_addr->rw = rw;
}

IMPLEMENT_READ_XYZ_VALUES(lsm303c)
{
	/* Intermediately consider everything as signed 16 bit integer */
	int16_t x;
	int16_t y;
	int16_t z;

	lsm303c_read((uint8_t*)(&(x)), LSM303C_OUT_X_L_A_ADDR, 2);
	lsm303c_read((uint8_t*)(&(y)), LSM303C_OUT_Y_L_A_ADDR, 2);
	lsm303c_read((uint8_t*)(&(z)), LSM303C_OUT_Z_L_A_ADDR, 2);

	x = x * LSM303C_ACCEL_8G_SENSITIVITY;
	y = y * LSM303C_ACCEL_8G_SENSITIVITY;
	z = z * LSM303C_ACCEL_8G_SENSITIVITY;

	inout_xyz_axis->x = x;
	inout_xyz_axis->y = y;
	inout_xyz_axis->z = z;
}
